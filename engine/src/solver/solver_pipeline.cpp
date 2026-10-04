#include "solver_pipeline.h"

#include "data_model.h"
#include "phys_messages.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdarg>
#include <cstring>
#include <new>
#include <vector>

// SolverPipeline: FleX's Solver::Init, Update, InitParams, UpdateSubstep and the stages it calls
// (reference/flex/src/Solver.cpp), ShapeData::RebuildShapeData (reference/flex/src/Data.cpp) and the
// half-spring part of Solver::SetSprings. Each function names the FleX function it ports; the slot
// assignments of every dispatch are FleX's.
//
// What is different from the reference, and why:
//
// - The data buffers are the domain's. mPositions, mVelocities, mPhases, mNormals, mRestPositions,
//   mActiveIndices, mParticleBounds, the spring, rigid, triangle, inflatable, shape and diffuse arrays are
//   looked up through the public interfaces of the domain at the start of every step.
// - The diffuse particles are not swapped with their compacted twins: the domain's buffers do not change
//   identity, so the compacted arrays are copied back into them.
// - The grid (mCellBucketStarts / mCellBucketEnds) belongs to the solver, not to the library.
// - The half-springs have a radix sort of their own. FleX reuses the sort of the cell hashes, which gives
//   the same order.
// - The members of KernelParams that InitParams leaves uninitialised (no kernel reads them) are zero.
// - Every dispatch that uses KernelParams passes it. FleX's CalculateDensity passes no constant buffer and
//   reads the one the dispatch before it left bound, which is the same block.

namespace ephys::detail {

namespace {

constexpr uint32_t divCeil(uint32_t n, uint32_t d) { return (n + d - 1) / d; }
constexpr uint32_t alignUp(uint32_t n, uint32_t a) { return (n + a - 1) / a * a; }

// The hash of a cell has 21 bits (CalculateParticleHash.hlsl: 7 bits per axis).
constexpr uint32_t c_HashSize = 0x200000;

// The bounds reduction has two levels of 128 and one final group of 128 (Solver::ComputeBounds).
constexpr uint32_t c_MaxSolverParticles = 128u * 128u * 128u;

// BasicMath.h.
inline float flexPi() { return 3.141592654f; }
inline float sqr(float x) { return x * x; }

// b0 of TransformShapeBounds and of the Springs* kernels: one count in a 16-byte register.
struct CountParams {
    uint32_t count = 0;
    uint32_t _pad[3] = {0, 0, 0};
};
static_assert(sizeof(CountParams) == 16);

}  // namespace

// ---- Random.h, SPH.cpp ---------------------------------------------------------------------------------

namespace {

// FleX keeps the two seeds in globals of its DLL, and so does this module: the table of random numbers a
// solver gets at creation depends on what ran before it in the process. They are not synchronised; like
// the device, solvers of one module are created from one thread at a time.
uint32_t g_Seed1 = 0;
uint32_t g_Seed2 = 0;

uint32_t flexRand() {
    g_Seed1 = (g_Seed2 * g_Seed1) ^ ((g_Seed1 >> 27) | (32 * g_Seed1)) ^ g_Seed2;
    g_Seed2 = ((g_Seed2 >> 20) | (g_Seed2 << 12)) ^ g_Seed1;
    return g_Seed1;
}

float W(float r, float h) { return 15.f / (flexPi() * h * h * h) * sqr(1.f - r / h); }

float dWdr(float r, float h) { return -30.f / (flexPi() * h * h * h * h) * (1.f - r / h); }

struct Float3 {
    float x, y, z;
};

// XMVector3Length.
float length(const Float3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

uint32_t tightPack3D(float radius, float separation, Float3* points, uint32_t maxPoints) {
    const int dim = int(std::ceil(radius / separation));
    uint32_t c = 0;
    for (int z = -dim; z <= dim; ++z) {
        for (int y = -dim; y <= dim; ++y) {
            for (int x = -dim; x <= dim; ++x) {
                float offset;
                if ((y + z) & 1)
                    offset = separation * 0.5f;
                else
                    offset = 0.f;
                const float ypos = float(y) * std::sqrt(0.75f) * separation;
                const float zpos = float(z) * std::sqrt(0.75f) * separation;
                const Float3 pos = {float(x) * separation + offset, ypos, zpos};
                const float len = length(pos);
                if (len != 0.f && c < maxPoints && radius >= len) points[c++] = pos;
            }
        }
    }
    return c;
}

}  // namespace

float flexRandf(float min, float max) noexcept {
    const float t = float(flexRand()) * (1.f / float(0xffffffffu));
    return (1.f - t) * min + t * max;
}

void sphCalculateRestDensity(float restDistance, float h, float* rho, float* rhoDeriv,
                             float* surfaceDeriv) noexcept {
    g_Seed1 = 39384234;
    g_Seed2 = 481343;

    static Float3 samples[0x800];
    const uint32_t n = tightPack3D(h, restDistance, samples, 0x800);
    float rho_ = 0.f;
    float rhoDeriv_ = 0.f;
    float a = 0.f, b = 0.f;
    for (uint32_t i = 0; i < n; ++i) {
        const Float3 sample = samples[i];
        const float r = length(sample);
        rho_ += W(r, h);
        const float dwdr = dWdr(r, h);
        rhoDeriv_ += sqr(dwdr);

        if (sample.y <= 0.f) {
            const float cosTheta = sample.y / r;
            a += dwdr * cosTheta;
            b -= r * cosTheta;
        }
    }

    *rho = rho_;
    *rhoDeriv = rhoDeriv_;
    *surfaceDeriv = a / b;
}

// ---- Construction ----------------------------------------------------------------------------------------

SolverPipeline::SolverPipeline(IPhysDevice* device, KernelLibraryHost* host, IPhysDomain* domain,
                               const SolverDesc& desc)
    : m_Device(device),
      m_Host(host),
      m_Kernels(device->getKernels()),
      m_Domain(domain),
      m_Desc(desc),
      m_Timers(device, host) {
    m_Variant = desc.baselineReductions ? KernelVariant::Baseline : KernelVariant::Default;
    m_MaxParticles = domain->getParticles()->getMaxParticles();
    if (IDiffuseParticleState* diffuse = domain->getDiffuseParticles()) m_MaxDiffuseParticles = diffuse->getMaxParticles();
}

SolverPipeline::~SolverPipeline() {
    // The kernel library caches the binding sets of recent dispatches, and those hold the scratch.
    m_Kernels->clearBindingCache();
}

void SolverPipeline::reportError(nvrhi::FRESULT code, const char* format, ...) const {
    (void)code;
    char text[512];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    report(m_Device->getDesc().messageCallback, PhysMessageSeverity::Error, "%s", text);
}

nvrhi::FRESULT SolverPipeline::createBuffer(const char* name, BufferUse use, uint32_t stride, uint32_t count,
                                            const void* fill, PhysBufferHandle& out) {
    PhysBufferDesc desc;
    desc.elementCount = count;
    desc.elementStride = stride;
    desc.use = use;
    desc.debugName = name;
    PhysBufferHandle buffer;
    nvrhi::FRESULT result = m_Device->createBuffer(desc, &buffer);
    if (NVRHI_FAILED(result)) return result;
    if (!buffer) return nvrhi::FE_GENERIC_ERROR;

    // FleX's CreateWithZero (or Create and Write): written in pieces, so that no single upload is large.
    const uint32_t chunkElements = std::max<uint32_t>(1u, (1u << 20) / stride);
    std::vector<uint8_t> chunk(size_t(std::min(count, chunkElements)) * stride, uint8_t(0));
    if (fill)
        for (size_t i = 0; i + stride <= chunk.size(); i += stride) std::memcpy(&chunk[i], fill, stride);
    for (uint32_t offset = 0; offset < count; offset += chunkElements) {
        const uint32_t n = std::min(chunkElements, count - offset);
        result = m_Device->writeBuffer(m_FillList, buffer, chunk.data(), offset, n);
        if (NVRHI_FAILED(result)) return result;
    }
    out = buffer;
    return nvrhi::FS_OK;
}

nvrhi::FRESULT SolverPipeline::init() noexcept {
    try {
        return initImpl();
    } catch (const std::bad_alloc&) {
        reportError(nvrhi::FE_OUT_OF_MEMORY, "createSolver: out of memory");
        return nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        return nvrhi::FE_GENERIC_ERROR;
    }
}

// Solver::Init.
nvrhi::FRESULT SolverPipeline::initImpl() {
    const uint32_t maxParticles = m_MaxParticles;
    const uint32_t maxParticlesAligned = alignUp(maxParticles, 256);
    const uint32_t maxContacts = m_Desc.maxContactsPerParticle;
    const uint32_t maxNeighbors = m_Desc.maxNeighborsPerParticle;

    nvrhi::FRESULT result = nvrhi::FS_OK;
    auto structured = [&](const char* name, uint32_t stride, uint32_t count, PhysBufferHandle& out) {
        if (NVRHI_SUCCEEDED(result)) result = createBuffer(name, BufferUse::Structured, stride, count, nullptr, out);
    };
    auto raw = [&](const char* name, uint32_t count, PhysBufferHandle& out) {
        if (NVRHI_SUCCEEDED(result)) result = createBuffer(name, BufferUse::Raw, 16, count, nullptr, out);
    };

    structured("ephys solver new positions", 16, maxParticles, m_NewPositions);
    structured("ephys solver sorted normals", 16, maxParticles, m_SortedNormals);
    raw("ephys solver normals temp", maxParticles, m_NormalsTemp);
    raw("ephys solver deltas", maxParticles, m_Deltas);
    structured("ephys solver neighbors", 4, maxParticlesAligned * maxNeighbors, m_Neighbors);
    structured("ephys solver neighbor counts", 4, maxParticles, m_NeighborCounts);
    structured("ephys solver sorted positions", 16, maxParticles, m_SortedPositions);
    structured("ephys solver sorted new positions", 16, maxParticles, m_SortedNewPositions);
    structured("ephys solver sorted new velocities", 16, maxParticles, m_SortedNewVelocities);
    structured("ephys solver contact planes", 16, maxContacts * maxParticles, m_ContactPlanes);
    structured("ephys solver contact velocities", 16, maxContacts * maxParticles, m_ContactVelocities);
    structured("ephys solver contact counts", 4, maxParticles, m_ContactCounts);
    structured("ephys solver densities", 4, maxParticles, m_Densities);
    structured("ephys solver sorted phases", 4, maxParticles, m_SortedPhases);
    structured("ephys solver potentials", 4, maxParticles, m_Potentials);
    structured("ephys solver curl", 16, maxParticles, m_Curl);

    {
        static const float c_Axes[3][4] = {{1.f, 0.f, 0.f, 0.1f}, {0.f, 1.f, 0.f, 0.1f}, {0.f, 0.f, 1.f, 0.1f}};
        static const char* const c_Names[3] = {"ephys solver anisotropy 1", "ephys solver anisotropy 2",
                                               "ephys solver anisotropy 3"};
        for (int i = 0; i < 3 && NVRHI_SUCCEEDED(result); ++i)
            result = createBuffer(c_Names[i], BufferUse::Structured, 16, maxParticles, c_Axes[i], m_Anisotropy[i]);
    }

    structured("ephys solver sorted densities", 4, maxParticles, m_SortedDensities);

    const uint32_t boundsBlocks = divCeil(maxParticles, 128);
    structured("ephys solver bounds lower", 16, boundsBlocks, m_BoundsLower);
    structured("ephys solver bounds upper", 16, boundsBlocks, m_BoundsUpper);
    structured("ephys solver group bounds lower", 16, divCeil(boundsBlocks, 128), m_GroupBoundsLower);
    structured("ephys solver group bounds upper", 16, divCeil(boundsBlocks, 128), m_GroupBoundsUpper);
    if (NVRHI_FAILED(result)) return result;

    result = m_Sort.create(m_Device, maxParticles);

    structured("ephys solver reverse lookup", 4, maxParticles, m_ReverseLookup);

    if (NVRHI_SUCCEEDED(result)) {
        float random[0x100];
        for (float& v : random) v = flexRandf(-1.f, 1.f);
        PhysBufferDesc desc;
        desc.elementCount = 0x100;
        desc.elementStride = 4;
        desc.use = BufferUse::Structured;
        desc.debugName = "ephys solver random";
        result = m_Device->createBuffer(desc, &m_Random);
        if (NVRHI_SUCCEEDED(result)) result = m_Device->writeBuffer(nullptr, m_Random, random, 0, 0x100);
    }

    if (m_MaxDiffuseParticles) {
        structured("ephys solver diffuse positions new", 16, m_MaxDiffuseParticles, m_DiffusePositionsNew);
        structured("ephys solver diffuse velocities new", 16, m_MaxDiffuseParticles, m_DiffuseVelocitiesNew);
        structured("ephys solver diffuse count new", 4, 1, m_NumDiffuseParticlesNew);
        m_DiffuseRate = 0.05f / 3.f;
        m_DiffuseTimer = 0.f;
    }

    structured("ephys solver sorted velocities", 16, maxParticles, m_SortedVelocities);
    structured("ephys solver smooth positions", 16, maxParticles, m_SmoothPositions);

    // Library::Init: the grid.
    structured("ephys solver cell starts", 4, c_HashSize, m_CellStarts);
    structured("ephys solver cell ends", 4, c_HashSize, m_CellEnds);
    return result;
}

nvrhi::FRESULT SolverPipeline::setCallback(SolverStage stage, IPhysSolverStageCallback* callback,
                                           IPhysSolverStageCallback** ppPrevious) noexcept {
    if (ppPrevious) *ppPrevious = nullptr;
    if (uint32_t(stage) >= uint32_t(SolverStage::Count)) {
        reportError(nvrhi::FE_INVALID_ARGS, "IPhysSolver::setStageCallback: not a stage");
        return nvrhi::FE_INVALID_ARGS;
    }
    PhysSolverStageCallbackHandle& slot = m_Callbacks[uint32_t(stage)];
    PhysSolverStageCallbackHandle previous = slot;
    slot = callback;
    if (ppPrevious) *ppPrevious = previous.Detach();
    return nvrhi::FS_OK;
}

IPhysBuffer* SolverPipeline::output(SolverOutput which) const noexcept {
    switch (which) {
        case SolverOutput::SmoothedPositions: return m_SmoothPositions;
        case SolverOutput::Densities: return m_Densities;
        case SolverOutput::Anisotropy1: return m_Anisotropy[0];
        case SolverOutput::Anisotropy2: return m_Anisotropy[1];
        case SolverOutput::Anisotropy3: return m_Anisotropy[2];
        case SolverOutput::ContactPlanes: return m_ContactPlanes;
        case SolverOutput::ContactVelocities: return m_ContactVelocities;
        case SolverOutput::ContactCounts: return m_ContactCounts;
        case SolverOutput::Neighbors: return m_Neighbors;
        case SolverOutput::NeighborCounts: return m_NeighborCounts;
        case SolverOutput::ApiToInternal: return m_ReverseLookup;
        case SolverOutput::InternalToApi: return m_Sort.getValuesBuffer();
        default: return nullptr;
    }
}

// ---- Seams: the PBD defaults -----------------------------------------------------------------------------

nvrhi::FRESULT SolverPipeline::beginConstraintIterations(const StepState&) { return nvrhi::FS_OK; }
void SolverPipeline::configureSolveSprings(const StepState&, KernelDispatchDesc&) {}
void SolverPipeline::configureInflatableVolume(const StepState&, KernelDispatchDesc&) {}
void SolverPipeline::configureSolveShapes(const StepState&, bool, KernelDispatchDesc&) {}
nvrhi::FRESULT SolverPipeline::onHalfSpringCapacity(uint32_t) { return nvrhi::FS_OK; }
nvrhi::FRESULT SolverPipeline::onInflatableCapacity(uint32_t) { return nvrhi::FS_OK; }

// ---- Dispatch helpers ------------------------------------------------------------------------------------

KernelDispatchDesc SolverPipeline::begin(KernelId kernel, uint32_t groups) const {
    KernelDispatchDesc dispatch;
    dispatch.kernel = kernel;
    dispatch.variant = m_Variant;
    dispatch.gridDim[0] = groups;
    dispatch.rootConstants = &m_KernelParams;
    dispatch.rootConstantsSize = sizeof(m_KernelParams);
    return dispatch;
}

void SolverPipeline::run(const StepState& state, const KernelDispatchDesc& dispatch) {
    if (NVRHI_FAILED(m_Result)) return;
    m_Result = m_Kernels->dispatch(state.commandList, dispatch);
}

// Library::ClearBufferInt.
void SolverPipeline::clearInt(nvrhi::ICommandList* commandList, IPhysBuffer* buffer, uint32_t count, int32_t value) {
    if (!buffer || NVRHI_FAILED(m_Result)) return;
    ClearIntParams params;
    params.length = int32_t(count);
    params.value = value;

    KernelDispatchDesc dispatch;
    dispatch.kernel = KernelId::ClearInt;
    dispatch.gridDim[0] = divCeil(count, 256);
    dispatch.rootConstants = &params;
    dispatch.rootConstantsSize = sizeof(params);
    dispatch.readWrite[0] = buffer->getRHIBuffer();
    m_Result = m_Kernels->dispatch(commandList, dispatch);
}

// ---- The domain ------------------------------------------------------------------------------------------

void SolverPipeline::refreshDomainView() {
    DomainView view;

    IParticleState* particles = m_Domain->getParticles();
    view.numParticles = particles->getActiveCount();
    view.positions = rhi(particles->getArray(ParticleArray::Position));
    view.velocities = rhi(particles->getArray(ParticleArray::Velocity));
    view.phases = rhi(particles->getArray(ParticleArray::Phase));
    view.normals = rhi(particles->getArray(ParticleArray::Normal));
    view.restPositions = rhi(particles->getArray(ParticleArray::RestPosition));
    view.activeIndices = rhi(particles->getArray(ParticleArray::ActiveIndex));
    view.bounds = rhi(particles->getBounds());

    view.numSprings = m_Domain->getSprings()->getNumSprings();

    IShapeMatchSet* rigids = m_Domain->getShapeMatches();
    view.numRigids = rigids->getNumRigids();
    view.numRigidIndices = rigids->getNumIndices();
    view.plastic = rigids->hasPlasticDeformation();
    view.rigidOffsets = rhi(rigids->getOffsets());
    view.rigidIndices = rhi(rigids->getIndices());
    view.rigidRestPositions = rhi(rigids->getRestPositions());
    view.rigidRestNormals = rhi(rigids->getRestNormals());
    view.rigidStiffness = rhi(rigids->getStiffness());
    view.rigidThresholds = rhi(rigids->getPlasticThresholds());
    view.rigidCreeps = rhi(rigids->getPlasticCreeps());
    view.rigidRotations = rhi(rigids->getRotations());
    view.rigidTranslations = rhi(rigids->getTranslations());

    ITriangleSet* triangles = m_Domain->getTriangles();
    view.numTriangles = triangles->getNumTriangles();
    view.triangleIndices = rhi(triangles->getIndices());
    view.triangleNormals = rhi(triangles->getNormals());

    IInflatableSet* inflatables = m_Domain->getInflatables();
    view.numInflatables = inflatables->getNumInflatables();
    view.inflatables = rhi(inflatables->getInflatables());

    ICollisionShapeSet* shapes = m_Domain->getShapes();
    view.numShapes = shapes->getNumShapes();
    view.shapeGeometry = rhi(shapes->getGeometry());
    view.shapePositions = rhi(shapes->getPositions());
    view.shapeRotations = rhi(shapes->getRotations());
    view.shapePrevPositions = rhi(shapes->getPrevPositions());
    view.shapePrevRotations = rhi(shapes->getPrevRotations());
    view.shapeFlags = rhi(shapes->getFlags());

    if (IDiffuseParticleState* diffuse = m_Domain->getDiffuseParticles()) {
        view.diffusePositions = rhi(diffuse->getPositions());
        view.diffuseVelocities = rhi(diffuse->getVelocities());
        view.diffuseCount = rhi(diffuse->getCountBuffer());
    }

    m_View = view;
}

// Solver::SetParams. FleX derives the rest density when the parameters are set; the domain owns the
// parameters here, so the solver does it when it first steps with a new revision of them. The result is a
// function of the radius and the fluid rest distance alone.
void SolverPipeline::refreshParams() {
    const uint64_t revision = m_Domain->getParamsRevision();
    if (revision == m_ParamsRevision) return;
    const PhysParams& params = m_Domain->getParams();

    if (params.fluidRestDistance <= 0.f) {
        m_FluidRestDensity = 0.f;
        m_FluidDensityConstraintScale = 0.f;
        m_FluidSurfaceConstraintScale = 0.f;
    } else if (params.radius != m_LastRadius || params.fluidRestDistance != m_LastFluidRestDistance) {
        sphCalculateRestDensity(params.fluidRestDistance, params.radius, &m_FluidRestDensity,
                                &m_FluidDensityConstraintScale, &m_FluidSurfaceConstraintScale);
    }
    m_LastRadius = params.radius;
    m_LastFluidRestDistance = params.fluidRestDistance;
    m_Params = params;
    m_ParamsRevision = revision;
}

// Solver::InitParams. The types of the intermediate results are FleX's: where FleX computes in double (a
// literal without the f suffix), so does this.
void SolverPipeline::initParams(const StepState& state) {
    const PhysParams& mParams = m_Params;
    const float dt = state.dta;
    const int numSubsteps = int(state.numSubsteps);
    KernelParams p = {};

    std::memcpy(p.kPlanes, mParams.planes, sizeof(mParams.planes));
    const float h = mParams.radius;
    p.kNumPlanes = mParams.numPlanes;
    p.kRadius = h;
    p.kRadiusSq = h * h;
    p.kInvRadius = 1.f / h;
    p.kRestDensity = m_FluidRestDensity;
    p.kInvRestDensity = 1.f / m_FluidRestDensity;
    p.kLambdaScale = 1.f / m_FluidDensityConstraintScale;
    p.kFluidRestDistance = mParams.fluidRestDistance;
    p.kSolidRestDistance = mParams.solidRestDistance;
    p.kVorticityConfinement = mParams.vorticityConfinement;
    p.kSolidPressure = mParams.solidPressure;
    p.kBuoyancy = mParams.buoyancy;
    p.kViscosity = mParams.viscosity * p.kInvRestDensity;
    p.kFreeSurfaceDrag = mParams.freeSurfaceDrag;
    p.kPoly6 = 315.f / (flexPi() * 64.f * h * h * h * h * h * h * h * h * h);
    p.kSpiky1 = 15.f / (flexPi() * h * h * h);
    p.kSpiky2 = 30.f / (flexPi() * h * h * h * h);
    const float rest = mParams.fluidRestDistance / mParams.radius;
    p.kCohesion1 = float(-(rest + 1.0) / sqr(rest));
    p.kCohesion2 = float(((sqr(rest) + rest) + 1.0) / sqr(rest));
    p.kSurfaceTension = (float)(p.kInvRestDensity * mParams.surfaceTension) / m_FluidSurfaceConstraintScale;
    p.kCohesion = mParams.cohesion * h;
    p.kAdhesion = mParams.adhesion;
    p.kGravity = {mParams.gravity[0], mParams.gravity[1], mParams.gravity[2]};
    p.kDynamicFriction = mParams.dynamicFriction;
    p.kStaticFriction = mParams.staticFriction;
    p.kParticleFriction = mParams.particleFriction;
    p.kDiffuseThreshold = m_MaxDiffuseParticles ? mParams.diffuseThreshold : 0.f;
    p.kDiffuseBuoyancy = mParams.diffuseBuoyancy;
    p.kDiffuseDrag = mParams.diffuseDrag;
    p.kDiffuseBallistic = mParams.diffuseBallistic;
    p.kDiffuseLifetime = mParams.diffuseLifetime;
    p.kAnisotropy = mParams.anisotropyScale;
    p.kAnisotropyMin = mParams.anisotropyMin * mParams.radius;
    p.kAnisotropyMax = mParams.anisotropyMax * mParams.radius;
    p.kSmoothing = mParams.smoothing;
    p.kDissipation = mParams.dissipation;
    p.kDamping = mParams.damping;
    p.kDrag = float(mParams.drag * 60.0);
    p.kLift = float(mParams.lift * 60.0);
    p.kRestitution = mParams.restitution;
    p.kCollisionDistance = mParams.collisionDistance;
    p.kCollisionThreshold = mParams.shapeCollisionMargin;
    p.kCollisionMargin = p.kCollisionDistance + p.kCollisionThreshold;
    p.kCollisionMarginSq = sqr(p.kCollisionDistance + p.kCollisionThreshold);
    p.kSleepThreshold = mParams.sleepThreshold;
    p.kSleepThresholdSq = sqr(mParams.sleepThreshold);
    p.kShockPropagation = mParams.shockPropagation;
    p.kSOR = float(1.0 / (float)(mParams.relaxationFactor + 1.0));
    p.kRelaxationFactor = mParams.relaxationFactor;
    p.kRelaxationMode = int(mParams.relaxationMode);
    p.kNumParticles = int(m_View.numParticles);
    p.kNumParticlesAligned = int(m_View.numParticles);  // Solver::SetActiveCount: not aligned
    p.kMaxParticles = int(m_MaxParticles);
    p.kDt = dt;
    p.kInvDt = float(1.0 / dt);
    p.kMaxSpeed = mParams.maxSpeed;
    p.kMaxAcceleration = mParams.maxAcceleration;
    p.kMaxVelocityDelta = mParams.maxAcceleration * dt;
    p.kSubStepLength = float(1.0 / (float)numSubsteps);
    p.kMaxNeighborsPerParticle = int(m_Desc.maxNeighborsPerParticle);
    p.kNumShapes = int(m_View.numShapes);
    p.kShapesCount = 0;  // mSDFDeviceShapes.size(): FleX never fills that list
    p.kInvCellEdge = float(1.0 / (float)(mParams.radius + mParams.particleCollisionMargin));
    p.kCollideParticlesRadiusSq = sqr(mParams.radius + mParams.particleCollisionMargin);
    p.kNumRigids = int(m_View.numRigids);
    p.kNumSprings = int(m_View.numSprings);
    p.kNumTriangles = int(m_View.numTriangles);
    p.kWind = {mParams.wind[0], mParams.wind[1], mParams.wind[2]};
    p.kMaxDiffuseParticles = int(m_MaxDiffuseParticles);
    p.kDiffuseDt = dt * (float)numSubsteps;
    p.kDiffuseMaxVelocity = FLT_MAX;
    p.kMaxContactsPerParticle = int(m_Desc.maxContactsPerParticle);

    m_KernelParams = p;
}

// ---- Derived data ----------------------------------------------------------------------------------------

// The second half of Solver::SetSprings: the springs seen from each of their two particles, sorted by
// particle. FleX does it when the springs are set; here the spring set is data and knows no kernels, so the
// solver does it in the first step after the springs changed.
void SolverPipeline::rebuildHalfSprings(const StepState& state) {
    ISpringSet* springs = m_Domain->getSprings();
    const uint64_t revision = springs->getContentRevision();
    if (revision == m_SpringRevision) return;
    m_SpringRevision = revision;

    const uint32_t numSprings = springs->getNumSprings();
    if (2 * numSprings > m_MaxHalfSprings) {
        const uint32_t elementCount = slack(2 * numSprings);
        nvrhi::FRESULT result = createScratch("ephys solver half spring opposites", 4, elementCount, m_HalfSpringOpposites);
        if (NVRHI_SUCCEEDED(result))
            result = createScratch("ephys solver half spring lengths", 4, elementCount, m_HalfSpringLengths);
        if (NVRHI_SUCCEEDED(result))
            result = createScratch("ephys solver half spring stiffness", 4, elementCount, m_HalfSpringStiffness);
        if (NVRHI_SUCCEEDED(result))
            result = createScratch("ephys solver half spring begin", 4, m_MaxParticles, m_HalfSpringParticleBegin);
        if (NVRHI_SUCCEEDED(result))
            result = createScratch("ephys solver half spring end", 4, m_MaxParticles, m_HalfSpringParticleEnd);
        if (NVRHI_SUCCEEDED(result)) result = onHalfSpringCapacity(elementCount);
        keep(result);
        if (NVRHI_FAILED(result)) return;
        m_MaxHalfSprings = elementCount;
    }
    if (numSprings == 0) return;

    if (m_SpringSort.getMaxKeys() < 2 * numSprings) {
        keep(m_SpringSort.create(m_Device, 2 * numSprings));
        if (NVRHI_FAILED(m_Result)) return;
    }

    CountParams params;
    params.count = numSprings;
    const uint32_t groups = divCeil(2 * numSprings, 256);
    auto beginSprings = [&](KernelId kernel) {
        KernelDispatchDesc dispatch;
        dispatch.kernel = kernel;
        dispatch.gridDim[0] = groups;
        dispatch.rootConstants = &params;
        dispatch.rootConstantsSize = sizeof(params);
        return dispatch;
    };

    // The keys are the particle indices of the springs, the values the indices of the half-springs.
    state.commandList->copyBuffer(m_SpringSort.getKeys(), 0, rhi(springs->getIndices()), 0,
                                  uint64_t(2) * sizeof(int32_t) * numSprings);

    // Solver::SpringGenerateIndices.
    KernelDispatchDesc dispatch = beginSprings(KernelId::SpringsGenerateIndices);
    dispatch.readWrite[0] = m_SpringSort.getValues();
    run(state, dispatch);

    keep(m_SpringSort.sort(m_Kernels, state.commandList, 2 * numSprings, 32));

    clearInt(state.commandList, m_HalfSpringParticleBegin, m_MaxParticles, 0);
    clearInt(state.commandList, m_HalfSpringParticleEnd, m_MaxParticles, 0);

    // Solver::SpringFindParticleRange.
    dispatch = beginSprings(KernelId::SpringsParticleRange);
    dispatch.readOnly[0] = m_SpringSort.getKeys();
    dispatch.readWrite[0] = rhi(m_HalfSpringParticleBegin);
    dispatch.readWrite[1] = rhi(m_HalfSpringParticleEnd);
    run(state, dispatch);

    // Solver::SpringRecordData.
    dispatch = beginSprings(KernelId::SpringsReorder);
    dispatch.readOnly[0] = m_SpringSort.getValues();
    dispatch.readOnly[1] = rhi(springs->getIndices());
    dispatch.readOnly[2] = rhi(springs->getRestLengths());
    dispatch.readOnly[3] = rhi(springs->getStiffness());
    dispatch.readWrite[0] = rhi(m_HalfSpringOpposites);
    dispatch.readWrite[1] = rhi(m_HalfSpringLengths);
    dispatch.readWrite[2] = rhi(m_HalfSpringStiffness);
    run(state, dispatch);
}

// ShapeData::RebuildShapeData: the world-space bounds of the shapes over the step (from the previous to
// the current transform), and the hierarchy over them. FleX rebuilds after every NvFlexSetShapes with a
// count; here that is a new content revision of the shape set.
void SolverPipeline::rebuildShapeData(const StepState& state) {
    ICollisionShapeSet* shapes = m_Domain->getShapes();
    const uint64_t revision = shapes->getContentRevision();
    if (revision == m_ShapeRevision) return;
    m_ShapeRevision = revision;

    const uint32_t numShapes = shapes->getNumShapes();
    if (numShapes == 0) return;

    if (shapes->getMaxShapes() > m_MaxShapes) {
        const uint32_t capacity = shapes->getMaxShapes();
        nvrhi::FRESULT result = createScratch("ephys solver shape bounds lower", 16, capacity, m_ShapeAabbMin);
        if (NVRHI_SUCCEEDED(result))
            result = createScratch("ephys solver shape bounds upper", 16, capacity, m_ShapeAabbMax);
        keep(result);
        if (NVRHI_FAILED(result)) return;
        m_MaxShapes = capacity;
    }

    StageScope scope(*this, state, "Rebuild shapes");
    IGeometryLibrary* geometry = shapes->getGeometryLibrary();

    CountParams params;
    params.count = numShapes;
    KernelDispatchDesc dispatch;
    dispatch.kernel = KernelId::TransformShapeBounds;
    dispatch.gridDim[0] = divCeil(numShapes, 256);
    dispatch.rootConstants = &params;
    dispatch.rootConstantsSize = sizeof(params);
    dispatch.readWrite[0] = rhi(m_ShapeAabbMin);
    dispatch.readWrite[1] = rhi(m_ShapeAabbMax);
    dispatch.readOnly[0] = rhi(shapes->getPositions());
    dispatch.readOnly[1] = rhi(shapes->getRotations());
    dispatch.readOnly[2] = rhi(shapes->getPrevPositions());
    dispatch.readOnly[3] = rhi(shapes->getPrevRotations());
    dispatch.readOnly[4] = rhi(shapes->getFlags());
    dispatch.readOnly[5] = rhi(shapes->getGeometry());
    dispatch.readOnly[6] = rhi(geometry->getConvexTable());
    dispatch.readOnly[7] = rhi(geometry->getTriangleMeshTable());
    run(state, dispatch);
    if (NVRHI_FAILED(m_Result)) return;

    keep(m_BvhBuilder.build(m_Device, state.commandList, m_ShapeBvh, rhi(m_ShapeAabbMin), rhi(m_ShapeAabbMax),
                            numShapes, nullptr, nullptr));
}

// Solver::SetInflatables, the part that is not data: one pressure term per inflatable.
void SolverPipeline::resizeInflatables() {
    const uint32_t capacity = m_Domain->getInflatables()->getMaxInflatables();
    if (capacity <= m_MaxInflatables) return;
    nvrhi::FRESULT result = createScratch("ephys solver inflatable pressures", 4, capacity, m_Pressures);
    if (NVRHI_SUCCEEDED(result)) result = onInflatableCapacity(capacity);
    keep(result);
    if (NVRHI_SUCCEEDED(result)) m_MaxInflatables = capacity;
}

// ---- A step ----------------------------------------------------------------------------------------------

nvrhi::FRESULT SolverPipeline::update(IPhysSolver* self, nvrhi::ICommandList* commandList, float dt,
                                      uint32_t substeps) noexcept {
    m_Self = self;
    nvrhi::FRESULT result;
    try {
        result = updateImpl(commandList, dt, substeps);
    } catch (const std::bad_alloc&) {
        reportError(nvrhi::FE_OUT_OF_MEMORY, "IPhysSolver::step: out of memory");
        result = nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        result = nvrhi::FE_GENERIC_ERROR;
    }
    m_Self = nullptr;
    return result;
}

// Solver::Update.
nvrhi::FRESULT SolverPipeline::updateImpl(nvrhi::ICommandList* commandList, float dt, uint32_t substeps) {
    if (substeps == 0 || !(dt > 0.f) || !std::isfinite(dt)) {
        reportError(nvrhi::FE_INVALID_ARGS, "IPhysSolver::step: dt %g with %u substeps is not a time step", double(dt),
                    substeps);
        return nvrhi::FE_INVALID_ARGS;
    }
    nvrhi::ICommandList* list = m_Host->recordingList(commandList);
    if (!list) {
        reportError(nvrhi::FE_NOT_ALIVE_OBJECT, "IPhysSolver::step: no command list");
        return nvrhi::FE_NOT_ALIVE_OBJECT;
    }

    // The scratch this step grows is cleared on the step's command list.
    struct FillListScope {
        nvrhi::ICommandList*& slot;
        FillListScope(nvrhi::ICommandList*& s, nvrhi::ICommandList* list) : slot(s) { slot = list; }
        ~FillListScope() { slot = nullptr; }
    } fillList(m_FillList, list);

    m_Result = nvrhi::FS_OK;
    refreshParams();

    // The timer queries of the step (when the timers are on): its record opens here and closes after the
    // last stage.
    StepScope timers(*this, list, commandList == nullptr);

    StepState state;
    state.commandList = list;
    state.dt = dt;
    state.dta = dt / (float)substeps;
    state.substepIdx = 0;
    state.numSubsteps = substeps;

    // mLib->mTriangleMeshData->Rebuild.
    {
        StageScope scope(*this, state, "FlattenTriangleMeshes");
        keep(m_Domain->getShapes()->getGeometryLibrary()->commit(list));
    }
    rebuildHalfSprings(state);
    resizeInflatables();
    rebuildShapeData(state);
    refreshDomainView();

    if (m_View.numParticles && NVRHI_SUCCEEDED(m_Result)) {
        state.numBlocks = divCeil(m_View.numParticles, 256);
        state.numBlocksHalf = divCeil(m_View.numParticles, 128);
        initParams(state);
        m_NeighborStride = m_View.numParticles;
        for (uint32_t i = 0; i < substeps && NVRHI_SUCCEEDED(m_Result); ++i) {
            state.substepIdx = i;
            updateSubstep(state);
        }
        markDomainChanged();
    }

    if (NVRHI_FAILED(m_Result))
        reportError(m_Result, "IPhysSolver::step: the step was not recorded completely (code %d)", int(m_Result));
    return m_Result;
}

// What a step wrote in the domain.
void SolverPipeline::markDomainChanged() {
    IParticleState* particles = m_Domain->getParticles();
    particles->getArray(ParticleArray::Position)->markContentChanged();
    particles->getArray(ParticleArray::Velocity)->markContentChanged();
    particles->getArray(ParticleArray::Normal)->markContentChanged();
    particles->getBounds()->markContentChanged();
    particles->markContentChanged();

    if (m_View.numRigids) {
        IShapeMatchSet* rigids = m_Domain->getShapeMatches();
        rigids->getRotations()->markContentChanged();
        rigids->getTranslations()->markContentChanged();
        rigids->getRestPositions()->markContentChanged();
        rigids->markContentChanged();
    }
    if (m_View.numTriangles) {
        ITriangleSet* triangles = m_Domain->getTriangles();
        triangles->getNormals()->markContentChanged();
        triangles->markContentChanged();
    }
    if (IDiffuseParticleState* diffuse = m_Domain->getDiffuseParticles()) {
        diffuse->getPositions()->markContentChanged();
        diffuse->getVelocities()->markContentChanged();
        diffuse->getCountBuffer()->markContentChanged();
        diffuse->markContentChanged();
    }
}

// Solver::UpdateSubstep.
void SolverPipeline::updateSubstep(const StepState& stateIn) {
    StepState state = stateIn;
    predict(state);
    computeBounds(state);
    calculateParticleHash(state);
    sortParticleHash(state);
    createGrid(state);
    reorderParticles(state);
    collideParticles(state);
    executeCallback(state, SolverStage::SubstepBegin, state.dta);
    clearInt(state.commandList, m_ContactCounts, m_MaxParticles, 0);
    collideTriangles(state);
    collideShapes(state);
    continuousShockPropagation(state);

    keep(beginConstraintIterations(state));

    for (int i = 0; i < m_Params.numIterations; ++i) {
        state.iteration = uint32_t(i);
        executeCallback(state, SolverStage::IterationStart, state.dta);
        calculateDensity(state);
        solveDensity(state);
        solveSprings(state);
        calculateAndSolveInflatables(state);
        solveShapes(state);
        applyDeltas(state);
        solveContact(state);
        executeCallback(state, SolverStage::IterationEnd, state.dta);
    }
    state.iteration = 0;

    updateVelocities(state);
    executeCallback(state, SolverStage::SubstepEnd, state.dta);

    const bool lastSubstep = state.substepIdx == state.numSubsteps - 1;
    if (lastSubstep) executeCallback(state, SolverStage::UpdateEnd, state.dt);

    calculateVorticity(state);
    solveVelocities(state);

    if (m_View.numTriangles) {
        updateTriangles(state);
        updateNormals(state);
        normalizeNormals(state);
    }
    finalize(state);

    m_DiffuseTimer += state.dta;

    if (lastSubstep) {
        if (m_MaxDiffuseParticles) {
            createDiffuse(state);
            updateDiffuse(state);
            clampDiffuse(state);
            compactDiffuse(state);
        }

        smoothPositions(state);
        calculateAnisotropy(state);
    }

    lazyClearGrid(state);

    ++m_StepCounter;
}

// ---- The stages ------------------------------------------------------------------------------------------

// Solver::Predict.
void SolverPipeline::predict(const StepState& state) {
    StageScope scope(*this, state, "Predicate");
    KernelDispatchDesc dispatch = begin(KernelId::Predict, state.numBlocks);
    dispatch.readWrite[0] = rhi(m_NewPositions);
    dispatch.readWrite[1] = m_View.velocities;
    dispatch.readOnly[0] = m_View.activeIndices;
    dispatch.readOnly[1] = m_View.positions;
    dispatch.readOnly[2] = m_View.phases;
    run(state, dispatch);
}

// Solver::ComputeBounds.
void SolverPipeline::computeBounds(const StepState& state) {
    StageScope scope(*this, state, "ComputeBounds");
    const uint32_t numGroupBoundsBlocks = divCeil(state.numBlocksHalf, 128);

    KernelDispatchDesc dispatch = begin(KernelId::CalculateBounds, state.numBlocksHalf);
    dispatch.readWrite[0] = rhi(m_BoundsLower);
    dispatch.readWrite[1] = rhi(m_BoundsUpper);
    dispatch.readOnly[0] = m_View.activeIndices;
    dispatch.readOnly[1] = rhi(m_NewPositions);
    run(state, dispatch);

    dispatch = begin(KernelId::CalculateBoundsGroup, numGroupBoundsBlocks);
    dispatch.readWrite[0] = rhi(m_GroupBoundsLower);
    dispatch.readWrite[1] = rhi(m_GroupBoundsUpper);
    dispatch.readOnly[2] = rhi(m_BoundsLower);
    dispatch.readOnly[3] = rhi(m_BoundsUpper);
    run(state, dispatch);

    dispatch = begin(KernelId::CalculateBoundsFinalize, 1);
    dispatch.readWrite[2] = m_View.bounds;
    dispatch.readOnly[2] = rhi(m_GroupBoundsLower);
    dispatch.readOnly[3] = rhi(m_GroupBoundsUpper);
    run(state, dispatch);
}

// Solver::CalculateParticleHash.
void SolverPipeline::calculateParticleHash(const StepState& state) {
    StageScope scope(*this, state, "CalculateParticleHash");
    const uint32_t numBlocks = divCeil(alignUp(m_View.numParticles, 1024), 256);
    KernelDispatchDesc dispatch = begin(KernelId::CalculateParticleHash, numBlocks);
    dispatch.readWrite[0] = m_Sort.getKeys();
    dispatch.readWrite[1] = m_Sort.getValues();
    dispatch.readOnly[0] = m_View.activeIndices;
    dispatch.readOnly[1] = rhi(m_NewPositions);
    dispatch.readOnly[2] = m_View.bounds;
    run(state, dispatch);
}

// Solver::SortParticleHash.
void SolverPipeline::sortParticleHash(const StepState& state) {
    if (!m_View.numParticles || NVRHI_FAILED(m_Result)) return;
    StageScope scope(*this, state, "SortParticleHash");
    keep(m_Sort.sort(m_Kernels, state.commandList, m_View.numParticles, 24));
}

// Solver::CreateGrid.
void SolverPipeline::createGrid(const StepState& state) {
    StageScope scope(*this, state, "CreateGrid");
    KernelDispatchDesc dispatch = begin(KernelId::CreateGrid, state.numBlocks);
    dispatch.readWrite[0] = rhi(m_CellStarts);
    dispatch.readWrite[1] = rhi(m_CellEnds);
    dispatch.readOnly[0] = m_Sort.getKeys();
    run(state, dispatch);
}

// Solver::RecorderParticles.
void SolverPipeline::reorderParticles(const StepState& state) {
    StageScope scope(*this, state, "RecordParticles");
    KernelDispatchDesc dispatch = begin(KernelId::ReorderParticles, state.numBlocks);
    dispatch.readWrite[0] = rhi(m_SortedPositions);
    dispatch.readWrite[1] = rhi(m_SortedNewPositions);
    dispatch.readWrite[2] = rhi(m_SortedVelocities);
    dispatch.readWrite[3] = rhi(m_SortedPhases);
    dispatch.readWrite[4] = rhi(m_ReverseLookup);
    dispatch.readWrite[5] = rhi(m_SortedNormals);
    dispatch.readOnly[0] = m_View.positions;
    dispatch.readOnly[1] = rhi(m_NewPositions);
    dispatch.readOnly[2] = m_View.velocities;
    dispatch.readOnly[3] = m_View.phases;
    dispatch.readOnly[4] = m_Sort.getValues();
    dispatch.readOnly[5] = m_View.normals;
    run(state, dispatch);
}

// Solver::CollideParticles.
void SolverPipeline::collideParticles(const StepState& state) {
    StageScope scope(*this, state, "CollideParticles");
    KernelDispatchDesc dispatch = begin(KernelId::CollideParticles, state.numBlocks);
    dispatch.readWrite[0] = rhi(m_Neighbors);
    dispatch.readWrite[1] = rhi(m_NeighborCounts);
    dispatch.readOnly[0] = rhi(m_CellStarts);
    dispatch.readOnly[1] = rhi(m_CellEnds);
    dispatch.readOnly[2] = m_Sort.getValues();
    dispatch.readOnly[3] = rhi(m_SortedNewPositions);
    dispatch.readOnly[4] = m_View.restPositions;
    dispatch.readOnly[5] = rhi(m_SortedPhases);
    dispatch.readOnly[6] = m_View.bounds;
    run(state, dispatch);
}

// Solver::CollideTriangles.
void SolverPipeline::collideTriangles(const StepState& state) {
    StageScope scope(*this, state, "CollideTriangles");

    SubstepParams substepParams = {};
    substepParams.kSubstepStart = (float)state.substepIdx / (float)state.numSubsteps;
    substepParams.kSubstepEnd = (float)(state.substepIdx + 1) / (float)state.numSubsteps;

    KernelDispatchDesc dispatch = begin(KernelId::CollideTriangles, state.numBlocks);
    dispatch.secondConstants = &substepParams;
    dispatch.secondConstantsSize = sizeof(substepParams);
    dispatch.readWrite[0] = rhi(m_ContactCounts);
    dispatch.readWrite[1] = rhi(m_ContactPlanes);
    dispatch.readWrite[2] = rhi(m_ContactVelocities);

    IGeometryLibrary* geometry = m_Domain->getShapes()->getGeometryLibrary();
    const bool haveMeshes = geometry->getTriangleVertices() != nullptr;
    if (haveMeshes) {
        dispatch.readOnly[0] = rhi(geometry->getTriangleVertices());
        dispatch.readOnly[1] = rhi(geometry->getTriangleIndices());
        dispatch.readOnly[2] = rhi(geometry->getTriangleBvhNodeLowers());
        dispatch.readOnly[3] = rhi(geometry->getTriangleBvhNodeUppers());
        dispatch.readOnly[4] = rhi(geometry->getTriangleBvhRoots());
    }

    dispatch.readOnly[5] = rhi(m_SortedPositions);
    dispatch.readOnly[6] = rhi(m_SortedNewPositions);
    dispatch.readOnly[7] = rhi(m_SortedPhases);

    if (m_View.numShapes > 0) {
        dispatch.readOnly[8] = m_View.shapePrevPositions;
        dispatch.readOnly[9] = m_View.shapePrevRotations;
        dispatch.readOnly[10] = m_View.shapeFlags;
        dispatch.readOnly[11] = m_View.shapePositions;
        dispatch.readOnly[12] = m_View.shapeRotations;
        dispatch.readOnly[13] = m_View.shapeGeometry;
        dispatch.readOnly[14] = rhi(m_ShapeBvh.rootNode);
        dispatch.readOnly[15] = rhi(m_ShapeBvh.nodeLowers);
        dispatch.readOnly[16] = rhi(m_ShapeBvh.nodeUppers);
    }

    if (haveMeshes) dispatch.readOnly[17] = rhi(geometry->getTriangleMeshTable());

    run(state, dispatch);
}

// Solver::CollideShapes.
void SolverPipeline::collideShapes(const StepState& state) {
    StageScope scope(*this, state, "CollideShapes");

    SubstepParams substepParams = {};
    substepParams.kSubstepStart = (float)state.substepIdx / (float)state.numSubsteps;
    substepParams.kSubstepEnd = (float)(state.substepIdx + 1) / (float)state.numSubsteps;

    KernelDispatchDesc dispatch = begin(KernelId::CollideShapes, state.numBlocksHalf);
    dispatch.secondConstants = &substepParams;
    dispatch.secondConstantsSize = sizeof(substepParams);
    dispatch.readWrite[0] = rhi(m_ContactCounts);
    dispatch.readWrite[1] = rhi(m_ContactPlanes);
    dispatch.readWrite[2] = rhi(m_ContactVelocities);

    if (m_View.numShapes > 0) {
        dispatch.readOnly[0] = m_View.shapeGeometry;
        dispatch.readOnly[1] = m_View.shapePositions;
        dispatch.readOnly[2] = m_View.shapeRotations;
        dispatch.readOnly[3] = m_View.shapePrevPositions;
        dispatch.readOnly[4] = m_View.shapePrevRotations;
        dispatch.readOnly[5] = m_View.shapeFlags;
    }
    dispatch.readOnly[6] = rhi(m_SortedPositions);
    dispatch.readOnly[7] = rhi(m_SortedNewPositions);
    dispatch.readOnly[8] = rhi(m_SortedPhases);

    if (m_View.numShapes > 0) {
        dispatch.readOnly[9] = rhi(m_ShapeBvh.rootNode);
        dispatch.readOnly[10] = rhi(m_ShapeBvh.nodeLowers);
        dispatch.readOnly[11] = rhi(m_ShapeBvh.nodeUppers);
    }

    IGeometryLibrary* geometry = m_Domain->getShapes()->getGeometryLibrary();
    if (geometry->getConvexIds(nullptr, 0)) {
        dispatch.readOnly[12] = rhi(geometry->getConvexTable());
        dispatch.readOnly[13] = rhi(geometry->getConvexPlanes());
    }

    if (geometry->getSdfIds(nullptr, 0)) {
        dispatch.readOnly[14] = rhi(geometry->getSdfTable());
        for (uint32_t i = 0; i < c_MaxSdfAssets; ++i) dispatch.readOnlyTextures[15 + i] = geometry->getSdfTexture(i);
    }

    run(state, dispatch);
}

// Solver::ContinuousShockPropagation.
void SolverPipeline::continuousShockPropagation(const StepState& state) {
    if (!(m_Params.shockPropagation > 0.f)) return;
    StageScope scope(*this, state, "ContinuousShockPropagation");
    KernelDispatchDesc dispatch = begin(KernelId::ContinuousShockPropagation, state.numBlocks);
    dispatch.readWrite[0] = rhi(m_SortedNewPositions);
    dispatch.readOnly[0] = m_View.bounds;
    run(state, dispatch);
}

// Solver::CalculateDensity.
void SolverPipeline::calculateDensity(const StepState& state) {
    if (!(m_Params.fluidRestDistance > 0.f)) return;
    StageScope scope(*this, state, "CalculateDensity");
    KernelDispatchDesc dispatch = begin(
        m_Params.surfaceTension <= 0.f ? KernelId::CalculateDensity : KernelId::CalculateDensitySurfaceTension,
        state.numBlocks);
    dispatch.readWrite[0] = rhi(m_Densities);
    dispatch.readWrite[1] = m_View.normals;
    dispatch.readWrite[2] = rhi(m_SortedNormals);
    dispatch.readOnly[0] = rhi(m_Neighbors);
    dispatch.readOnly[1] = rhi(m_NeighborCounts);
    dispatch.readOnly[2] = rhi(m_SortedNewPositions);
    dispatch.readOnly[3] = rhi(m_SortedPhases);
    dispatch.readOnly[4] = m_Sort.getValues();
    run(state, dispatch);
}

// Solver::SolveDensity.
void SolverPipeline::solveDensity(const StepState& state) {
    StageScope scope(*this, state, "SolveDensity");
    KernelId kernel;
    if (m_Desc.featureMode == FeatureMode::SimpleSolids)
        kernel = KernelId::SolveDensitiesNonFluid;
    else if (m_Params.surfaceTension <= 0.f)
        kernel = KernelId::SolveDensities;
    else
        kernel = KernelId::SolveDensitiesSurfaceTension;

    KernelDispatchDesc dispatch = begin(kernel, state.numBlocks);
    dispatch.readWrite[0] = rhi(m_Deltas);
    dispatch.readOnly[0] = rhi(m_Neighbors);
    dispatch.readOnly[1] = rhi(m_NeighborCounts);
    dispatch.readOnly[2] = rhi(m_SortedNewPositions);
    dispatch.readOnly[3] = rhi(m_SortedPhases);
    dispatch.readOnly[4] = rhi(m_Densities);
    dispatch.readOnly[5] = m_View.normals;
    dispatch.readOnly[6] = rhi(m_SortedPositions);
    dispatch.readOnly[7] = rhi(m_SortedNormals);
    run(state, dispatch);
}

// Solver::SolveSprings.
void SolverPipeline::solveSprings(const StepState& state) {
    if (!m_View.numSprings) return;
    StageScope scope(*this, state, "SolveSprings");
    const uint32_t numParticleBlocks = divCeil(8 * m_MaxParticles, 512);
    KernelDispatchDesc dispatch = begin(KernelId::SolveSprings, numParticleBlocks);
    dispatch.readWrite[0] = rhi(m_Deltas);
    // readOnly[0] is FleX's mHalfSpringIndices, which nothing writes and the kernel does not declare.
    dispatch.readOnly[1] = rhi(m_HalfSpringOpposites);
    dispatch.readOnly[2] = rhi(m_HalfSpringLengths);
    dispatch.readOnly[3] = rhi(m_HalfSpringStiffness);
    dispatch.readOnly[4] = rhi(m_HalfSpringParticleBegin);
    dispatch.readOnly[5] = rhi(m_HalfSpringParticleEnd);
    dispatch.readOnly[6] = rhi(m_ReverseLookup);
    dispatch.readOnly[7] = rhi(m_SortedNewPositions);
    configureSolveSprings(state, dispatch);
    run(state, dispatch);
}

// Solver::CalculateAndSolveInflatables.
void SolverPipeline::calculateAndSolveInflatables(const StepState& state) {
    if (!m_View.numInflatables) return;
    StageScope scope(*this, state, "SolveInflatables");
    const uint32_t numInflatables = m_View.numInflatables;

    KernelDispatchDesc dispatch = begin(KernelId::CalculateInflatableVolume, numInflatables);
    dispatch.readWrite[0] = rhi(m_Pressures);
    dispatch.readOnly[0] = m_View.inflatables;
    dispatch.readOnly[1] = rhi(m_SortedNewPositions);
    dispatch.readOnly[2] = rhi(m_ReverseLookup);
    dispatch.readOnly[3] = m_View.triangleIndices;
    configureInflatableVolume(state, dispatch);
    run(state, dispatch);

    dispatch = begin(KernelId::SolveInflatableVolume, numInflatables);
    dispatch.readWrite[0] = rhi(m_Deltas);
    dispatch.readOnly[0] = m_View.inflatables;
    dispatch.readOnly[1] = rhi(m_SortedNewPositions);
    dispatch.readOnly[2] = rhi(m_ReverseLookup);
    dispatch.readOnly[3] = m_View.triangleIndices;
    dispatch.readOnly[4] = rhi(m_Pressures);
    run(state, dispatch);
}

// Solver::SolveShapes. FleX picks among thread group sizes of 32, 64 and 128 on the vendor path only; the
// kernels here have the one size of its baseline (engine/shaders/PORTING.md).
void SolverPipeline::solveShapes(const StepState& state) {
    if (!m_View.numRigids) return;
    StageScope scope(*this, state, "SolveShapes");
    const bool plastic = m_View.plastic && m_View.rigidThresholds && m_View.rigidCreeps;

    KernelDispatchDesc dispatch =
        begin(plastic ? KernelId::SolveShapesPlasticDeformation : KernelId::SolveShapes, m_View.numRigids);
    dispatch.readWrite[0] = rhi(m_Deltas);
    dispatch.readWrite[1] = m_View.rigidRestPositions;
    dispatch.readWrite[2] = rhi(m_SortedNewPositions);
    dispatch.readWrite[3] = m_View.rigidRotations;
    dispatch.readWrite[4] = m_View.rigidTranslations;
    dispatch.readWrite[5] = m_View.normals;
    dispatch.readOnly[0] = m_View.rigidOffsets;
    dispatch.readOnly[1] = m_View.rigidIndices;
    dispatch.readOnly[2] = m_View.rigidStiffness;
    if (plastic) {
        dispatch.readOnly[3] = m_View.rigidThresholds;
        dispatch.readOnly[4] = m_View.rigidCreeps;
    }
    dispatch.readOnly[5] = rhi(m_ReverseLookup);
    dispatch.readOnly[6] = rhi(m_SortedPositions);
    dispatch.readOnly[7] = m_View.rigidRestNormals;
    configureSolveShapes(state, plastic, dispatch);
    run(state, dispatch);
}

// Solver::ApplyDeltas.
void SolverPipeline::applyDeltas(const StepState& state) {
    StageScope scope(*this, state, "ApplyDeltas");
    KernelDispatchDesc dispatch = begin(KernelId::ApplyDeltas, state.numBlocks);
    dispatch.readWrite[0] = rhi(m_SortedNewPositions);
    dispatch.readOnly[0] = rhi(m_Deltas);
    run(state, dispatch);
}

// Solver::SolveContact. FleX loads three contact kernels (sequential, averaged, accumulate) and dispatches
// the sequential one whatever the relaxation mode is; the relaxation mode acts in ApplyDeltas.
void SolverPipeline::solveContact(const StepState& state) {
    StageScope scope(*this, state, "SolveContact");
    KernelDispatchDesc dispatch = begin(KernelId::SolveContactsSequential, state.numBlocks);
    dispatch.readWrite[0] = rhi(m_SortedNewPositions);
    dispatch.readOnly[0] = rhi(m_ContactCounts);
    dispatch.readOnly[1] = rhi(m_ContactPlanes);
    dispatch.readOnly[2] = rhi(m_SortedPositions);
    dispatch.readOnly[3] = rhi(m_ContactVelocities);
    run(state, dispatch);
}

// Solver::UpdateVelocities.
void SolverPipeline::updateVelocities(const StepState& state) {
    StageScope scope(*this, state, "UpdateVelocities");
    KernelDispatchDesc dispatch = begin(KernelId::UpdateVelocities, state.numBlocks);
    dispatch.readWrite[0] = rhi(m_SortedNewVelocities);
    dispatch.readOnly[0] = rhi(m_SortedPositions);
    dispatch.readOnly[1] = rhi(m_SortedNewPositions);
    run(state, dispatch);
}

// Solver::CalculateVorticity.
void SolverPipeline::calculateVorticity(const StepState& state) {
    if (!(m_Params.vorticityConfinement > 0.f)) return;
    StageScope scope(*this, state, "CalculateVorticity");
    KernelDispatchDesc dispatch = begin(KernelId::CalculateVorticity, state.numBlocks);
    dispatch.readWrite[0] = rhi(m_Curl);
    dispatch.readOnly[0] = rhi(m_SortedNewPositions);
    dispatch.readOnly[1] = rhi(m_SortedNewVelocities);
    dispatch.readOnly[2] = rhi(m_SortedPhases);
    dispatch.readOnly[3] = rhi(m_Neighbors);
    dispatch.readOnly[4] = rhi(m_NeighborCounts);
    dispatch.readOnly[5] = rhi(m_ContactPlanes);
    dispatch.readOnly[6] = rhi(m_ContactCounts);
    run(state, dispatch);
}

// Solver::SolveVelocities.
void SolverPipeline::solveVelocities(const StepState& state) {
    StageScope scope(*this, state, "SolveVelocities");
    KernelDispatchDesc dispatch = begin(KernelId::SolveVelocities, state.numBlocks);
    dispatch.readWrite[0] = rhi(m_SortedNewPositions);
    dispatch.readWrite[1] = rhi(m_Deltas);
    dispatch.readWrite[2] = rhi(m_SortedDensities);
    dispatch.readWrite[3] = rhi(m_Potentials);
    dispatch.readOnly[0] = rhi(m_Neighbors);
    dispatch.readOnly[1] = rhi(m_NeighborCounts);
    dispatch.readOnly[2] = rhi(m_ContactPlanes);
    dispatch.readOnly[3] = rhi(m_ContactCounts);
    dispatch.readOnly[4] = rhi(m_SortedPositions);
    dispatch.readOnly[5] = rhi(m_SortedVelocities);
    dispatch.readOnly[6] = rhi(m_SortedNewVelocities);
    dispatch.readOnly[7] = rhi(m_SortedPhases);
    dispatch.readOnly[8] = rhi(m_Curl);
    run(state, dispatch);
}

// Solver::UpdateTriangles.
void SolverPipeline::updateTriangles(const StepState& state) {
    StageScope scope(*this, state, "UpdateTriangles");
    KernelDispatchDesc dispatch = begin(KernelId::UpdateTriangles, divCeil(m_View.numTriangles, 256));
    dispatch.readWrite[0] = rhi(m_Deltas);
    dispatch.readWrite[1] = m_View.triangleNormals;
    dispatch.readOnly[0] = rhi(m_SortedNewPositions);
    dispatch.readOnly[1] = rhi(m_SortedNewVelocities);
    dispatch.readOnly[2] = rhi(m_ReverseLookup);
    dispatch.readOnly[3] = m_View.triangleIndices;
    run(state, dispatch);
}

// Solver::UpdateNormals.
void SolverPipeline::updateNormals(const StepState& state) {
    if (state.substepIdx != state.numSubsteps - 1) return;
    StageScope scope(*this, state, "UpdateNormals");
    KernelDispatchDesc dispatch = begin(KernelId::UpdateVertexNormalsInit, divCeil(m_MaxParticles, 256));
    dispatch.readWrite[0] = rhi(m_NormalsTemp);
    dispatch.readOnly[0] = rhi(m_ReverseLookup);
    dispatch.readOnly[1] = m_View.triangleIndices;
    dispatch.readOnly[2] = m_View.triangleNormals;
    run(state, dispatch);

    dispatch.kernel = KernelId::UpdateVertexNormals;
    dispatch.gridDim[0] = divCeil(m_View.numTriangles, 256);
    run(state, dispatch);
}

// Solver::NormalizeNormals.
void SolverPipeline::normalizeNormals(const StepState& state) {
    if (state.substepIdx != state.numSubsteps - 1) return;
    StageScope scope(*this, state, "NormalizeNormals");
    KernelDispatchDesc dispatch = begin(KernelId::NormalizeVertexNormals, state.numBlocks);
    dispatch.readWrite[0] = m_View.normals;
    dispatch.readOnly[0] = m_Sort.getValues();
    dispatch.readOnly[1] = rhi(m_NormalsTemp);
    run(state, dispatch);
}

// Solver::Finalize.
void SolverPipeline::finalize(const StepState& state) {
    StageScope scope(*this, state, "Finalize");
    KernelDispatchDesc dispatch = begin(KernelId::Finalize, state.numBlocks);
    dispatch.readWrite[0] = m_View.positions;
    dispatch.readWrite[1] = m_View.velocities;
    dispatch.readWrite[2] = rhi(m_Densities);
    dispatch.readOnly[0] = rhi(m_SortedNewPositions);
    dispatch.readOnly[1] = rhi(m_SortedNewVelocities);
    dispatch.readOnly[2] = rhi(m_SortedDensities);
    dispatch.readOnly[3] = rhi(m_Deltas);
    dispatch.readOnly[4] = m_Sort.getValues();
    run(state, dispatch);
}

// Solver::CreateDiffuse.
void SolverPipeline::createDiffuse(const StepState& state) {
    if (!(m_DiffuseTimer >= m_DiffuseRate)) return;
    StageScope scope(*this, state, "CreateDiffuseParticles");
    m_DiffuseTimer = std::fmod(m_DiffuseTimer, m_DiffuseRate);
    KernelDispatchDesc dispatch = begin(KernelId::CreateDiffuseParticles, state.numBlocks);
    dispatch.readWrite[0] = m_View.diffuseCount;
    dispatch.readWrite[1] = m_View.diffusePositions;
    dispatch.readWrite[2] = m_View.diffuseVelocities;
    dispatch.readOnly[0] = rhi(m_Potentials);
    dispatch.readOnly[1] = rhi(m_SortedNewPositions);
    dispatch.readOnly[2] = rhi(m_SortedNewVelocities);
    dispatch.readOnly[3] = rhi(m_Random);
    run(state, dispatch);
}

// Solver::UpdateDiffuse.
void SolverPipeline::updateDiffuse(const StepState& state) {
    StageScope scope(*this, state, "UpdateDiffuseParticles");
    KernelDispatchDesc dispatch = begin(KernelId::UpdateDiffuseParticles, divCeil(m_MaxDiffuseParticles, 256));
    dispatch.readWrite[0] = m_View.diffusePositions;
    dispatch.readWrite[1] = m_View.diffuseVelocities;
    dispatch.readOnly[0] = m_View.diffuseCount;
    dispatch.readOnly[1] = rhi(m_CellStarts);
    dispatch.readOnly[2] = rhi(m_CellEnds);
    dispatch.readOnly[3] = rhi(m_NeighborCounts);
    dispatch.readOnly[4] = rhi(m_SortedNewPositions);
    dispatch.readOnly[5] = rhi(m_SortedNewVelocities);
    dispatch.readOnly[6] = rhi(m_SortedPhases);
    dispatch.readOnly[7] = m_View.bounds;
    run(state, dispatch);
}

// Solver::ClampDiffuse.
void SolverPipeline::clampDiffuse(const StepState& state) {
    StageScope scope(*this, state, "ClampDiffuseParticleCount");
    KernelDispatchDesc dispatch = begin(KernelId::ClampDiffuseParticleCount, 1);
    dispatch.readWrite[0] = m_View.diffuseCount;
    dispatch.readWrite[1] = rhi(m_NumDiffuseParticlesNew);
    run(state, dispatch);
}

// Solver::CompactDiffuse, and the swap of the diffuse arrays with their compacted twins that follows it in
// Solver::UpdateSubstep. The arrays of the domain keep their identity, so the twins are copied into them.
void SolverPipeline::compactDiffuse(const StepState& state) {
    StageScope scope(*this, state, "CompactDiffuseParticles");
    KernelDispatchDesc dispatch = begin(KernelId::CompactDiffuseParticles, divCeil(m_MaxDiffuseParticles, 256));
    dispatch.readWrite[0] = rhi(m_NumDiffuseParticlesNew);
    dispatch.readWrite[1] = rhi(m_DiffusePositionsNew);
    dispatch.readWrite[2] = rhi(m_DiffuseVelocitiesNew);
    dispatch.readOnly[0] = m_View.diffuseCount;
    dispatch.readOnly[1] = m_View.diffusePositions;
    dispatch.readOnly[2] = m_View.diffuseVelocities;
    run(state, dispatch);
    if (NVRHI_FAILED(m_Result)) return;

    const uint64_t bytes = uint64_t(m_MaxDiffuseParticles) * 16;
    state.commandList->copyBuffer(m_View.diffusePositions, 0, rhi(m_DiffusePositionsNew), 0, bytes);
    state.commandList->copyBuffer(m_View.diffuseVelocities, 0, rhi(m_DiffuseVelocitiesNew), 0, bytes);
    state.commandList->copyBuffer(m_View.diffuseCount, 0, rhi(m_NumDiffuseParticlesNew), 0, 4);
}

// Solver::SmoothPositions.
void SolverPipeline::smoothPositions(const StepState& state) {
    if (!(m_Params.anisotropyScale > 0.f || m_Params.smoothing > 0.f)) return;
    StageScope scope(*this, state, "SmoothPositions");
    KernelDispatchDesc dispatch = begin(KernelId::SmoothPositions, state.numBlocks);
    dispatch.readWrite[0] = rhi(m_SortedPositions);
    dispatch.readWrite[1] = rhi(m_SmoothPositions);
    dispatch.readOnly[0] = rhi(m_Neighbors);
    dispatch.readOnly[1] = rhi(m_NeighborCounts);
    dispatch.readOnly[2] = rhi(m_SortedPhases);
    dispatch.readOnly[3] = rhi(m_SortedNewPositions);
    dispatch.readOnly[4] = m_Sort.getValues();
    run(state, dispatch);
}

// Solver::CalculateAnisotropy.
void SolverPipeline::calculateAnisotropy(const StepState& state) {
    if (!(m_Params.anisotropyScale > 0.f)) return;
    StageScope scope(*this, state, "CalculateAnisotropy");
    KernelDispatchDesc dispatch = begin(KernelId::CalculateAnisotropy, state.numBlocks);
    dispatch.readWrite[0] = rhi(m_Anisotropy[0]);
    dispatch.readWrite[1] = rhi(m_Anisotropy[1]);
    dispatch.readWrite[2] = rhi(m_Anisotropy[2]);
    dispatch.readOnly[0] = rhi(m_Neighbors);
    dispatch.readOnly[1] = rhi(m_NeighborCounts);
    dispatch.readOnly[2] = m_Sort.getValues();
    dispatch.readOnly[3] = rhi(m_SortedPhases);
    dispatch.readOnly[4] = rhi(m_SortedPositions);
    run(state, dispatch);
}

// Solver::LazyClearGrid: clears the cells this substep filled, so that the grid is empty for the next one.
void SolverPipeline::lazyClearGrid(const StepState& state) {
    StageScope scope(*this, state, "LazyClearGrid");
    KernelDispatchDesc dispatch = begin(KernelId::ClearCellBuckets, state.numBlocks);
    dispatch.readWrite[0] = rhi(m_CellStarts);
    dispatch.readWrite[1] = rhi(m_CellEnds);
    dispatch.readOnly[0] = m_View.activeIndices;
    dispatch.readOnly[1] = rhi(m_NewPositions);
    dispatch.readOnly[2] = m_View.bounds;
    run(state, dispatch);
}

// Solver::ExecuteCallback. FleX passes the sorted cell hashes as sortedToOriginalMap and the sorted
// original indices as originalToSortedMap; the context here carries the two maps its members name.
void SolverPipeline::executeCallback(const StepState& state, SolverStage stage, float dt) {
    IPhysSolverStageCallback* callback = m_Callbacks[uint32_t(stage)];
    if (!callback || NVRHI_FAILED(m_Result)) return;

    SolverStageContext context;
    context.solver = m_Self;
    context.commandList = state.commandList;
    context.positions = rhi(m_SortedNewPositions);
    context.velocities = rhi(m_SortedNewVelocities);
    context.phases = rhi(m_SortedPhases);
    context.sortedToOriginalMap = m_Sort.getValues();
    context.originalToSortedMap = rhi(m_ReverseLookup);
    context.numActive = m_View.numParticles;
    context.dt = dt;
    context.substep = state.substepIdx;
    context.numSubsteps = state.numSubsteps;
    context.iteration = state.iteration;
    context.stage = stage;
    callback->onSolverStage(context);
}

}  // namespace ephys::detail
