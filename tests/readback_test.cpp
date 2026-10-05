// ethereal_phys_readback_test: IPhysReadback and IPhysTimers (readback.h) on a real nvrhi device, headless.
//
//     ethereal_phys_readback_test <d3d11|d3d12|vulkan> [compute] [--adapter <n>] [--shaders <dir>]
//
// "compute" creates the device with a compute queue and has the engine submit to it (PhysDeviceDesc::
// preferComputeQueue), where the graphics API has one, and runs every check there: the copies, the solver's
// stages and the timers on the compute queue. The device is created as in the other device tests: debug
// runtime, nvrhi validation layer, and every message of either, or of the engine, is a failure unless the
// test provoked it.
//
// What it checks:
//   - the objects: creation, what they refuse (one message each), the references they keep;
//   - every read* method against a reference computed here, independently of the readback path: the
//     neighbour lists against a brute-force search over the positions the solver searched (captured with a
//     stage callback), the densities against the SPH sum over those lists at the positions the solver
//     summed at, the contacts against the collision planes of the scene, the bounds against the extent of
//     the predicted positions, the API-order outputs against the solver's buffers;
//   - FleX's order semantics, on a scene where sorting by grid cell really permutes the particles: the
//     API-order getters are indexed by particle, the internal-order ones by internal index, and FleX's own
//     loop over GetNeighbors (apiToInternal, counts, neighbors, internalToApi) yields the right particles;
//   - partial regions (only the elements of the region are written, at the right place, for every kind of
//     layout: per particle, C per particle, the column-wise neighbour lists, the diffuse particles), and the
//     diffuse count;
//   - the timers: off, no query is recorded; on, every stage that ran has a time, total is the sum of its
//     members (FleX's NvFlexGetTimers), the device latency covers the stages, the detail timers are the
//     stage names in FleX's order with "Total" last, results arrive within a few frames and reading them
//     never waits; a query budget too small for a step measures it in part without failing; a step on an
//     application command list; the XPBD solver.
//
// Exit code: 0 passed, 1 failed, 77 the graphics API has no usable device on this machine (skipped).

#include <ethereal/phys/device.h>
#include <ethereal/phys/readback.h>
#include <ethereal/phys/solver.h>

#include "solver_support.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <vector>

namespace {

using namespace support;
using ephys::BufferUse;
using ephys::SolverOutput;
using ephys::SolverStage;

// ---- Host helpers --------------------------------------------------------------------------------------------

// The whole of a Readback buffer, after everything recorded has run (the caller waited).
template <typename T>
std::vector<T> mapAll(IPhysBuffer* readback) {
    std::vector<T> values(size_t(readback->getElementCount()) * readback->getElementStride() / sizeof(T));
    void* memory = nullptr;
    CHECK(readback->map(&memory) == nvrhi::FS_OK);
    if (memory) {
        std::memcpy(values.data(), memory, values.size() * sizeof(T));
        CHECK(readback->unmap() == nvrhi::FS_OK);
    }
    return values;
}

constexpr uint8_t c_Sentinel = 0xA5;

// A Readback buffer whose every byte is c_Sentinel, so that a test can tell which elements a later copy
// wrote. The fill is copied in and waited for: two copies into one buffer leave it in the same state
// (CopyDest), so nvrhi puts no barrier between them, and a fill still in flight could land after the copy
// under test.
PhysBufferHandle sentinelReadback(const Env& env, uint32_t count, uint32_t stride) {
    PhysBufferHandle readback = env.create(BufferUse::Readback, count, stride);
    const std::vector<uint8_t> bytes(size_t(count) * stride, c_Sentinel);
    PhysBufferHandle source = env.upload(bytes.data(), bytes.size(), stride);
    CopyRegion region;
    region.elementCount = count;
    CHECK(env.phys->copyBuffer(nullptr, readback, source, region) == nvrhi::FS_OK);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    env.device->runGarbageCollection();
    return readback;
}

// True when the `bytes` bytes at `p` are all c_Sentinel.
bool untouched(const void* p, size_t bytes) {
    const uint8_t* b = static_cast<const uint8_t*>(p);
    for (size_t i = 0; i < bytes; ++i)
        if (b[i] != c_Sentinel) return false;
    return true;
}

// FleX's SPHCalculateRestDensity (FleX/src/SPH.cpp), the density part: the rest density of a
// tightly packed fluid. Written here again, from the reference, so that the density check does not use the
// engine's copy.
float restDensity(float restDistance, float h) {
    const float pi = 3.141592654f;
    const int dim = int(std::ceil(h / restDistance));
    float rho = 0.0f;
    uint32_t c = 0;
    for (int z = -dim; z <= dim; ++z)
        for (int y = -dim; y <= dim; ++y)
            for (int x = -dim; x <= dim; ++x) {
                const float offset = ((y + z) & 1) ? restDistance * 0.5f : 0.0f;
                const float px = float(x) * restDistance + offset;
                const float py = float(y) * std::sqrt(0.75f) * restDistance;
                const float pz = float(z) * std::sqrt(0.75f) * restDistance;
                const float r = std::sqrt(px * px + py * py + pz * pz);
                if (r != 0.0f && c < 0x800 && h >= r) {
                    ++c;
                    const float q = 1.0f - r / h;
                    rho += 15.0f / (pi * h * h * h) * (q * q);
                }
            }
    return rho;
}

// Copies of the solver's internal arrays at two stages of the last substep of a step, recorded by the stage
// callback on the step's command list.
//
// SubstepBegin runs right after the neighbour search (CollideParticles) and before the contacts with the
// planes and shapes are made (CollideShapes): its positions are the ones both searched. SubstepEnd runs
// after the velocities were derived, right before SolveVelocities sums the densities FleX's GetDensities
// returns: its positions are the ones that sum used.
class Capture final : public nvrhi::ObjectImpl<ephys::IPhysSolverStageCallback> {
 public:
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(Capture)
    NVRHI_IMPLEMENTS_INTERFACE(ephys::IPhysSolverStageCallback)
    NVRHI_END_INTERFACE_TABLE()

    Capture(const Env& env, uint32_t maxParticles) {
        searchPositions = env.create(BufferUse::Readback, maxParticles, 16);
        sumPositions = env.create(BufferUse::Readback, maxParticles, 16);
        phases = env.create(BufferUse::Readback, maxParticles, 4);
        sortedToOriginal = env.create(BufferUse::Readback, maxParticles, 4);
        originalToSorted = env.create(BufferUse::Readback, maxParticles, 4);
    }

    void onSolverStage(const ephys::SolverStageContext& context) noexcept override {
        if (!armed || context.substep + 1 != context.numSubsteps) return;
        const uint64_t n = context.numActive;
        if (context.stage == SolverStage::SubstepBegin) {
            context.commandList->copyBuffer(searchPositions->getRHIBuffer(), 0, context.positions, 0, n * 16);
            context.commandList->copyBuffer(phases->getRHIBuffer(), 0, context.phases, 0, n * 4);
            context.commandList->copyBuffer(sortedToOriginal->getRHIBuffer(), 0, context.sortedToOriginalMap, 0, n * 4);
            context.commandList->copyBuffer(originalToSorted->getRHIBuffer(), 0, context.originalToSortedMap, 0, n * 4);
            numActive = context.numActive;
            ++searches;
        } else if (context.stage == SolverStage::SubstepEnd) {
            context.commandList->copyBuffer(sumPositions->getRHIBuffer(), 0, context.positions, 0, n * 16);
            ++sums;
        }
    }

    bool armed = false;
    uint32_t numActive = 0;
    int searches = 0;
    int sums = 0;
    PhysBufferHandle searchPositions, sumPositions, phases, sortedToOriginal, originalToSorted;
};

// ---- The objects -----------------------------------------------------------------------------------------------

void testObjects(const Env& env, D3DMessages& d3d) {
    std::printf("the objects\n");
    const scenes::Scene scene = scenes::granular();
    const uint32_t n = scene.numParticles();
    const uint32_t maxParticles = scene.maxParticles();
    Rig rig = loadScene(env, scene);
    CHECK(bool(rig));
    if (!rig) return;

    const nvrhi::FLONG solverRefs = refCount(rig.solver.Get());
    ephys::PhysReadbackHandle readback;
    CHECK(env.phys->createReadback(rig.solver, &readback) == nvrhi::FS_OK);
    CHECK(readback != nullptr);
    if (!readback) return;
    CHECK(readback->getSolver() == rig.solver.Get());
    CHECK(refCount(rig.solver.Get()) == solverRefs + 1);
    {
        ephys::PhysReadbackHandle same;
        CHECK(readback->QueryInterface(NVRHI_IID_PPV_ARGS(&same)) == nvrhi::FS_OK && same == readback);
        ephys::PhysTimersHandle notTimers;
        CHECK(readback->QueryInterface(NVRHI_IID_PPV_ARGS(&notTimers)) == nvrhi::FE_NOINTERFACE);
    }

    ephys::PhysTimersHandle timers;
    CHECK(env.phys->createTimers(rig.solver, &timers) == nvrhi::FS_OK);
    CHECK(timers != nullptr);
    if (!timers) return;
    CHECK(timers->getSolver() == rig.solver.Get());
    CHECK(refCount(rig.solver.Get()) == solverRefs + 2);
    CHECK(timers->getQueryBudget() == 128);
    CHECK(timers->isQueueSupported(nvrhi::CommandQueue::Graphics));
    CHECK(!timers->isQueueSupported(nvrhi::CommandQueue::Copy));
    std::printf("  timestamps: graphics queue %s, compute queue %s\n",
                timers->isQueueSupported(nvrhi::CommandQueue::Graphics) ? "yes" : "no",
                timers->isQueueSupported(nvrhi::CommandQueue::Compute) ? "yes" : "no");

    // Refusals, one message each.
    {
        ephys::PhysReadbackHandle none;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, env.phys->createReadback(nullptr, &none));
        CHECK(none == nullptr);
        ephys::PhysTimersHandle noTimers;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, env.phys->createTimers(nullptr, &noTimers));
        CHECK(noTimers == nullptr);
    }
    PhysBufferHandle wide = env.create(BufferUse::Readback, maxParticles, 16);
    PhysBufferHandle narrow = env.create(BufferUse::Readback, maxParticles, 4);
    PhysBufferHandle fourInts = env.create(BufferUse::Readback, 4, 4);
    std::vector<float> zeros(maxParticles, 0.0f);
    PhysBufferHandle upload = env.upload(zeros, 4);
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, readback->readDensities(nullptr, wide, nullptr));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, readback->readDensities(nullptr, upload, nullptr));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, readback->readDensities(nullptr, nullptr, nullptr));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, readback->readAnisotropy(nullptr, nullptr, nullptr, nullptr, nullptr));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, readback->readAnisotropy(nullptr, wide, narrow, wide, nullptr));
    CopyRegion region;
    region.srcOffset = maxParticles - 1;
    region.elementCount = 2;  // one past the capacity
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, readback->readDensities(nullptr, narrow, &region));
    region.srcOffset = 0;
    region.dstOffset = 3;  // the destination holds 4
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, readback->readDensities(nullptr, fourInts, &region));
    region = CopyRegion();
    region.elementCount = 1;  // before the first step there is no particle of a step: the stride is 0
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                readback->readNeighbors(nullptr, narrow, nullptr, nullptr, nullptr, &region));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, readback->readBounds(nullptr, nullptr, nullptr));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, readback->readBounds(nullptr, narrow, nullptr));

    // FleX's GetSmoothParticles before the first step: the particle positions. And a domain without
    // diffuse particles: nothing, without a message (FleX's GetDiffuseParticles tests its capacity).
    {
        const int errors = env.messages->errors.load();
        PhysBufferHandle smooth = env.create(BufferUse::Readback, maxParticles, 16);
        CHECK(readback->readSmoothedPositions(nullptr, smooth, nullptr) == nvrhi::FS_OK);
        PhysBufferHandle diffusePositions = env.create(BufferUse::Readback, 4, 16);
        CHECK(readback->readDiffuseParticles(nullptr, diffusePositions, nullptr, fourInts, nullptr) == nvrhi::FS_OK);
        CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
        const std::vector<float> positions = mapAll<float>(smooth);
        CHECK(std::equal(scene.positions.begin(), scene.positions.end(), positions.begin()));
        CHECK(env.messages->errors.load() == errors);
    }

    // The objects keep the solver: the rig goes, the solver stays until they do.
    ephys::IPhysSolver* solver = rig.solver;
    rig.solver = nullptr;
    CHECK(refCount(solver) == 2);
    readback = nullptr;
    CHECK(refCount(solver) == 1);
    timers = nullptr;  // the last reference: the solver goes with it
    rig = Rig();
    (void)n;
    CHECK(d3d.problems() == 0);
}

// ---- Readback against independent references ----------------------------------------------------------------

// A pair at the edge of a radius may be in or out by rounding (the GPU may fuse a multiply-add).
bool nearBoundary(float distanceSq, float radiusSq) { return std::fabs(distanceSq - radiusSq) <= radiusSq * 1.0e-5f; }

void testFluid(const Env& env, D3DMessages& d3d) {
    std::printf("fluid: every getter against an independent reference\n");
    const scenes::Scene scene = scenes::fluid();
    const uint32_t n = scene.numParticles();
    const uint32_t P = scene.maxParticles();
    const uint32_t C = scene.maxContacts;
    const uint32_t N = scene.maxNeighbors;
    const ephys::PhysParams& params = scene.params;
    Rig rig = loadScene(env, scene);
    CHECK(bool(rig));
    if (!rig) return;

    // The fluid falls onto the floor of its box first, so that the planes make contacts.
    const uint32_t c_Settle = 20;
    for (uint32_t i = 0; i < c_Settle; ++i) stepScene(env, scene, rig);

    nvrhi::AutoPtr<Capture> capture = MAKE_RC_OBJ_PTR(Capture, env, P);
    CHECK(rig.solver->setStageCallback(SolverStage::SubstepBegin, capture, nullptr) == nvrhi::FS_OK);
    CHECK(rig.solver->setStageCallback(SolverStage::SubstepEnd, capture, nullptr) == nvrhi::FS_OK);
    capture->armed = true;
    stepScene(env, scene, rig);
    capture->armed = false;
    CHECK(rig.solver->setStageCallback(SolverStage::SubstepBegin, nullptr, nullptr) == nvrhi::FS_OK);
    CHECK(rig.solver->setStageCallback(SolverStage::SubstepEnd, nullptr, nullptr) == nvrhi::FS_OK);
    CHECK(capture->searches == 1 && capture->sums == 1 && capture->numActive == n);

    ephys::PhysReadbackHandle readback;
    CHECK(env.phys->createReadback(rig.solver, &readback) == nvrhi::FS_OK);
    if (!readback) return;
    const uint32_t S = rig.solver->getNeighborStride();
    CHECK(S == n);

    auto make = [&](uint32_t count, uint32_t stride) { return env.create(BufferUse::Readback, count, stride); };
    PhysBufferHandle densities = make(P, 4);
    PhysBufferHandle q[3] = {make(P, 16), make(P, 16), make(P, 16)};
    PhysBufferHandle smooth = make(P, 16);
    PhysBufferHandle planes = make(P * C, 16), velocities = make(P * C, 16), indices = make(P, 4), counts = make(P, 4);
    PhysBufferHandle neighbors = make(N * S, 4), neighborCounts = make(P, 4), apiToInternal = make(P, 4),
                     internalToApi = make(P, 4);
    PhysBufferHandle lower = make(1, 12), upper = make(1, 12);
    CHECK(readback->readDensities(nullptr, densities, nullptr) == nvrhi::FS_OK);
    CHECK(readback->readAnisotropy(nullptr, q[0], q[1], q[2], nullptr) == nvrhi::FS_OK);
    CHECK(readback->readSmoothedPositions(nullptr, smooth, nullptr) == nvrhi::FS_OK);
    CHECK(readback->readContacts(nullptr, planes, velocities, indices, counts, nullptr) == nvrhi::FS_OK);
    CHECK(readback->readNeighbors(nullptr, neighbors, neighborCounts, apiToInternal, internalToApi, nullptr) == nvrhi::FS_OK);
    CHECK(readback->readBounds(nullptr, lower, upper) == nvrhi::FS_OK);

    // Partial regions, into buffers that hold the sentinel everywhere else.
    CopyRegion region;
    region.srcOffset = 17;
    region.dstOffset = 5;
    region.elementCount = 100;
    PhysBufferHandle densitiesPart = sentinelReadback(env, 120, 4);
    PhysBufferHandle q2Part = sentinelReadback(env, 120, 16);
    PhysBufferHandle planesPart = sentinelReadback(env, 110 * C, 16);
    PhysBufferHandle countsPart = sentinelReadback(env, 110, 4);
    PhysBufferHandle indicesPart = sentinelReadback(env, 110, 4);
    CHECK(readback->readDensities(nullptr, densitiesPart, &region) == nvrhi::FS_OK);
    CHECK(readback->readAnisotropy(nullptr, nullptr, q2Part, nullptr, &region) == nvrhi::FS_OK);
    CHECK(readback->readContacts(nullptr, planesPart, nullptr, indicesPart, countsPart, &region) == nvrhi::FS_OK);
    CopyRegion columns;
    columns.srcOffset = 30;
    columns.dstOffset = 1;
    columns.elementCount = 50;
    PhysBufferHandle neighborsPart = sentinelReadback(env, (N - 1) * S + 60, 4);
    CHECK(readback->readNeighbors(nullptr, neighborsPart, nullptr, nullptr, nullptr, &columns) == nvrhi::FS_OK);

    // The same getter on a command list of the application.
    PhysBufferHandle densitiesApp = make(P, 4);
    env.phys->flush();  // D3D11: one open command list at a time
    {
        nvrhi::CommandListHandle commandList;
        nvrhi::CommandListParameters listParams;
        listParams.queueType = env.phys->getInfo().executionQueue;
        CHECK(env.device->createCommandList(listParams, &commandList) == nvrhi::FS_OK);
        if (commandList) {
            commandList->open();
            CHECK(readback->readDensities(commandList, densitiesApp, nullptr) == nvrhi::FS_OK);
            commandList->close();
            nvrhi::ICommandList* list = commandList;
            env.device->executeCommandLists(&list, 1, listParams.queueType);
        }
    }
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    env.device->runGarbageCollection();

    const std::vector<float> d = mapAll<float>(densities);
    const std::vector<float> dApp = mapAll<float>(densitiesApp);
    const std::vector<int32_t> toInternal = mapAll<int32_t>(apiToInternal);
    const std::vector<uint32_t> toApi = mapAll<uint32_t>(internalToApi);
    const std::vector<int32_t> lists = mapAll<int32_t>(neighbors);
    const std::vector<int32_t> listCounts = mapAll<int32_t>(neighborCounts);
    const std::vector<float> contactPlanes = mapAll<float>(planes);
    const std::vector<float> contactVelocities = mapAll<float>(velocities);
    const std::vector<int32_t> contactIndices = mapAll<int32_t>(indices);
    const std::vector<int32_t> contactCounts = mapAll<int32_t>(counts);
    const std::vector<float> searchPositions = mapAll<float>(capture->searchPositions);
    const std::vector<float> sumPositions = mapAll<float>(capture->sumPositions);
    const std::vector<int32_t> phases = mapAll<int32_t>(capture->phases);
    const std::vector<uint32_t> sortedToOriginal = mapAll<uint32_t>(capture->sortedToOriginal);
    const std::vector<int32_t> originalToSorted = mapAll<int32_t>(capture->originalToSorted);

    // The maps: what the stage callback was given, a permutation of the active particles, and one that
    // really permutes (sorting by grid cell moved the particles).
    uint32_t moved = 0;
    bool mapsAgree = true, permutation = true;
    for (uint32_t i = 0; i < n; ++i) {
        mapsAgree = mapsAgree && toApi[i] == sortedToOriginal[i] && toInternal[i] == originalToSorted[i];
        permutation = permutation && toApi[i] < n && toInternal[toApi[i]] == int32_t(i);
        if (toApi[i] != i) ++moved;
    }
    CHECK(mapsAgree);
    CHECK(permutation);
    CHECK(moved > n / 2);
    std::printf("  internal order: %u of %u particles are not at their API index\n", moved, n);
    CHECK(contactIndices == toInternal);  // GetContacts' indices are the API-to-internal map

    // Neighbours: FleX's loop over GetNeighbors (NvFlex.h), in API indices, against a brute-force search over
    // the positions the solver searched, in internal indices mapped to API ones.
    const float searchRadius = params.radius + params.particleCollisionMargin;
    const float searchRadiusSq = searchRadius * searchRadius;
    auto at = [](const std::vector<float>& p, uint32_t i) { return &p[size_t(i) * 4]; };
    uint32_t listMismatches = 0, capped = 0;
    size_t totalNeighbors = 0;
    for (uint32_t j = 0; j < n; ++j) {
        const uint32_t i = uint32_t(toInternal[j]);
        std::set<uint32_t> gpu;
        for (int32_t c = 0; c < listCounts[i]; ++c) gpu.insert(toApi[uint32_t(lists[size_t(c) * S + i])]);
        totalNeighbors += gpu.size();
        std::set<uint32_t> sure, maybe;
        const float* p = at(searchPositions, i);
        for (uint32_t k = 0; k < n; ++k) {
            if (k == i) continue;
            const float* o = at(searchPositions, k);
            const float dx = p[0] - o[0], dy = p[1] - o[1], dz = p[2] - o[2];
            const float d2 = dx * dx + dy * dy + dz * dz;
            if (nearBoundary(d2, searchRadiusSq))
                maybe.insert(sortedToOriginal[k]);
            else if (d2 < searchRadiusSq)
                sure.insert(sortedToOriginal[k]);
        }
        if (sure.size() + maybe.size() > N) {
            ++capped;
            continue;
        }
        bool ok = std::includes(gpu.begin(), gpu.end(), sure.begin(), sure.end());
        for (uint32_t k : gpu) ok = ok && (sure.count(k) || maybe.count(k));
        if (!ok) ++listMismatches;
    }
    std::printf("  neighbours: %zu over %u particles; %u lists differ from the brute-force search, %u at the cap\n",
                totalNeighbors, n, listMismatches, capped);
    CHECK(listMismatches == 0);
    CHECK(capped == 0);
    CHECK(totalNeighbors > n * 4);

    // Densities (API order): SolveVelocities' sum over the neighbour list at the positions of SubstepEnd,
    // fluid neighbours within the radius, over the rest density (Finalize).
    const float h = params.radius;
    const float spiky1 = 15.0f / (3.141592654f * h * h * h);
    const float rest = restDensity(params.fluidRestDistance, h);
    float worstDensity = 0.0f, meanDensity = 0.0f;
    for (uint32_t j = 0; j < n; ++j) {
        const uint32_t i = uint32_t(toInternal[j]);
        const float* p = at(sumPositions, i);
        float potential = 0.0f;
        for (int32_t c = 0; c < listCounts[i]; ++c) {
            const uint32_t k = uint32_t(lists[size_t(c) * S + i]);
            const float* o = at(sumPositions, k);
            const float dx = p[0] - o[0], dy = p[1] - o[1], dz = p[2] - o[2];
            const float d2 = dx * dx + dy * dy + dz * dz;
            const bool fluid = (phases[i] & ephys::PhaseFluid) && (phases[k] & ephys::PhaseFluid);
            if (d2 <= h * h && d2 > 0.0f && fluid) {
                const float q1 = 1.0f - std::sqrt(d2) / h;
                potential += spiky1 * q1 * q1;
            }
        }
        const float expected = potential / rest;
        worstDensity = std::max(worstDensity, std::fabs(d[j] - expected) / std::max(expected, 1.0e-3f));
        meanDensity += d[j];
    }
    meanDensity /= float(n);
    std::printf("  densities: mean %.3f of the rest density, worst relative error %.2e\n", double(meanDensity),
                double(worstDensity));
    CHECK(worstDensity < 1.0e-4f);
    CHECK(meanDensity > 0.3f && meanDensity < 1.5f);
    CHECK(std::equal(d.begin(), d.begin() + n, dApp.begin()));  // the application's command list

    // Contacts (internal order): the planes of the scene within the collision distance plus the margin of the
    // positions CollideShapes saw, in the order of the planes (the scene has no shapes).
    uint32_t contactMismatches = 0, withContacts = 0, edge = 0;
    for (uint32_t i = 0; i < n; ++i) {
        const float* p = at(searchPositions, i);
        std::vector<int> expected;
        bool atEdge = false;
        for (int k = 0; k < params.numPlanes; ++k) {
            const float* plane = params.planes[k];
            const float dist = p[0] * plane[0] + p[1] * plane[1] + p[2] * plane[2] + plane[3] - params.collisionDistance;
            if (std::fabs(dist - params.shapeCollisionMargin) < 1.0e-6f) atEdge = true;
            if (dist <= params.shapeCollisionMargin && expected.size() < C) expected.push_back(k);
        }
        if (atEdge) {
            ++edge;
            continue;
        }
        if (!expected.empty()) ++withContacts;
        bool ok = contactCounts[i] == int32_t(expected.size());
        for (size_t c = 0; ok && c < expected.size(); ++c) {
            const float* got = &contactPlanes[(size_t(i) * C + c) * 4];
            const float* gotVelocity = &contactVelocities[(size_t(i) * C + c) * 4];
            ok = std::memcmp(got, params.planes[expected[c]], 16) == 0 && gotVelocity[0] == 0.0f &&
                 gotVelocity[1] == 0.0f && gotVelocity[2] == 0.0f && gotVelocity[3] == -1.0f;
        }
        if (!ok) ++contactMismatches;
    }
    std::printf("  contacts: %u particles touch a plane, %u differ from the planes of the scene (%u at the edge)\n",
                withContacts, contactMismatches, edge);
    CHECK(contactMismatches == 0);
    CHECK(withContacts > 100);

    // The API-order outputs that have no cheaper reference than the solver's own buffers: the copies are
    // the buffers, bit for bit. The anisotropy axes of a fluid particle are unit vectors.
    for (int axis = 0; axis < 3; ++axis) {
        const SolverOutput output = SolverOutput(uint8_t(SolverOutput::Anisotropy1) + axis);
        const std::vector<float> raw = env.fetch<float>(rig.solver->getOutput(output), 0, P);
        const std::vector<float> copied = mapAll<float>(q[axis]);
        CHECK(raw == copied);
        float worstLength = 0.0f;
        for (uint32_t j = 0; j < n; ++j) {
            const float* a = &copied[size_t(j) * 4];
            worstLength = std::max(worstLength, std::fabs(std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) - 1.0f));
        }
        CHECK(worstLength < 1.0e-3f);
    }
    {
        const std::vector<float> raw = env.fetch<float>(rig.solver->getOutput(SolverOutput::SmoothedPositions), 0, P);
        CHECK(raw == mapAll<float>(smooth));
        // Smoothing pulls a particle towards its neighbours: near, not at, its position.
        const std::vector<float> positions = readPositions(env, rig, n);
        float worst = 0.0f;
        for (uint32_t j = 0; j < n; ++j)
            for (int k = 0; k < 3; ++k) worst = std::max(worst, std::fabs(raw[size_t(j) * 4 + k] - positions[size_t(j) * 4 + k]));
        CHECK(worst > 0.0f && worst < params.radius);
    }

    // Bounds: the extent of the predicted positions of the last substep (those of SubstepBegin, which the
    // reorder only permuted), grown by the radius (CalculateBounds).
    {
        const std::vector<float> lo = mapAll<float>(lower), hi = mapAll<float>(upper);
        float expectedLo[3] = {INFINITY, INFINITY, INFINITY}, expectedHi[3] = {-INFINITY, -INFINITY, -INFINITY};
        for (uint32_t i = 0; i < n; ++i)
            for (int k = 0; k < 3; ++k) {
                expectedLo[k] = std::min(expectedLo[k], searchPositions[size_t(i) * 4 + k]);
                expectedHi[k] = std::max(expectedHi[k], searchPositions[size_t(i) * 4 + k]);
            }
        for (int k = 0; k < 3; ++k) {
            expectedLo[k] -= params.radius;
            expectedHi[k] += params.radius;
        }
        const std::vector<float> raw = env.fetch<float>(rig.domain->getParticles()->getBounds(), 0, 2);
        bool same = true;
        for (int k = 0; k < 3; ++k) {
            same = same && lo[k] == raw[k] && hi[k] == raw[3 + k];
            CHECK(lo[k] == expectedLo[k]);
            CHECK(hi[k] == expectedHi[k]);
        }
        CHECK(same);
        std::printf("  bounds: (%.3f %.3f %.3f) to (%.3f %.3f %.3f)\n", double(lo[0]), double(lo[1]), double(lo[2]),
                    double(hi[0]), double(hi[1]), double(hi[2]));
    }

    // Partial regions: the elements of the region at the destination offset, the sentinel everywhere else.
    {
        const std::vector<float> part = mapAll<float>(densitiesPart);
        bool ok = true;
        for (uint32_t e = 0; e < 120; ++e) {
            if (e >= region.dstOffset && e < region.dstOffset + region.elementCount)
                ok = ok && std::memcmp(&part[e], &d[e - region.dstOffset + region.srcOffset], 4) == 0;
            else
                ok = ok && untouched(&part[e], 4);
        }
        CHECK(ok);

        const std::vector<float> q2 = mapAll<float>(q[1]);
        const std::vector<float> q2p = mapAll<float>(q2Part);
        ok = true;
        for (uint32_t e = 0; e < 120; ++e) {
            if (e >= region.dstOffset && e < region.dstOffset + region.elementCount)
                ok = ok && std::memcmp(&q2p[size_t(e) * 4], &q2[size_t(e - region.dstOffset + region.srcOffset) * 4], 16) == 0;
            else
                ok = ok && untouched(&q2p[size_t(e) * 4], 16);
        }
        CHECK(ok);

        // C contact slots per particle.
        const std::vector<float> planesP = mapAll<float>(planesPart);
        const std::vector<int32_t> countsP = mapAll<int32_t>(countsPart);
        const std::vector<int32_t> indicesP = mapAll<int32_t>(indicesPart);
        ok = true;
        for (uint32_t e = 0; e < 110; ++e) {
            const bool inside = e >= region.dstOffset && e < region.dstOffset + region.elementCount;
            const uint32_t src = e - region.dstOffset + region.srcOffset;
            if (inside) {
                ok = ok && countsP[e] == contactCounts[src] && indicesP[e] == contactIndices[src];
                ok = ok && std::memcmp(&planesP[size_t(e) * C * 4], &contactPlanes[size_t(src) * C * 4], 16 * C) == 0;
            } else {
                ok = ok && untouched(&countsP[e], 4) && untouched(&indicesP[e], 4) && untouched(&planesP[size_t(e) * C * 4], 16 * C);
            }
        }
        CHECK(ok);

        // The column-wise neighbour lists: each column k keeps its place k * S.
        const std::vector<int32_t> listsP = mapAll<int32_t>(neighborsPart);
        ok = true;
        for (size_t e = 0; e < listsP.size(); ++e) {
            const uint32_t k = uint32_t(e / S), i = uint32_t(e % S);
            const bool inside = k < N && i >= columns.dstOffset && i < columns.dstOffset + columns.elementCount;
            if (inside)
                ok = ok && listsP[e] == lists[size_t(k) * S + (i - columns.dstOffset + columns.srcOffset)];
            else
                ok = ok && untouched(&listsP[e], 4);
        }
        CHECK(ok);
    }
    CHECK(d3d.problems() == 0);
}

// ---- Diffuse particles --------------------------------------------------------------------------------------------

void testDiffuse(const Env& env, D3DMessages& d3d) {
    std::printf("diffuse particles\n");
    const scenes::Scene scene = scenes::fluidDiffuse();
    Rig rig = loadScene(env, scene);
    CHECK(bool(rig));
    if (!rig) return;
    for (uint32_t i = 0; i < 15; ++i) stepScene(env, scene, rig);

    ephys::PhysReadbackHandle readback;
    CHECK(env.phys->createReadback(rig.solver, &readback) == nvrhi::FS_OK);
    if (!readback) return;
    const uint32_t capacity = scene.maxDiffuse;
    PhysBufferHandle positions = env.create(BufferUse::Readback, capacity, 16);
    PhysBufferHandle velocities = env.create(BufferUse::Readback, capacity, 16);
    PhysBufferHandle count = env.create(BufferUse::Readback, 1, 4);
    CHECK(readback->readDiffuseParticles(nullptr, positions, velocities, count, nullptr) == nvrhi::FS_OK);
    // Only the count.
    PhysBufferHandle countOnly = sentinelReadback(env, 3, 4);
    CHECK(readback->readDiffuseParticles(nullptr, nullptr, nullptr, countOnly, nullptr) == nvrhi::FS_OK);
    // A region of the positions.
    CopyRegion region;
    region.srcOffset = 3;
    region.dstOffset = 2;
    region.elementCount = 60;
    PhysBufferHandle part = sentinelReadback(env, 70, 16);
    CHECK(readback->readDiffuseParticles(nullptr, part, nullptr, nullptr, &region) == nvrhi::FS_OK);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);

    ephys::IDiffuseParticleState* diffuse = rig.domain->getDiffuseParticles();
    const int32_t n = mapAll<int32_t>(count)[0];
    const int32_t raw = env.fetch<int32_t>(diffuse->getCountBuffer(), 0, 1)[0];
    std::printf("  %d diffuse particles of %u\n", n, capacity);
    CHECK(n == raw);
    CHECK(n > 0 && uint32_t(n) <= capacity);
    const std::vector<int32_t> counts = mapAll<int32_t>(countOnly);
    CHECK(counts[0] == n && untouched(&counts[1], 8));
    const std::vector<float> p = mapAll<float>(positions);
    CHECK(p == env.fetch<float>(diffuse->getPositions(), 0, capacity));
    CHECK(mapAll<float>(velocities) == env.fetch<float>(diffuse->getVelocities(), 0, capacity));
    // The alive particles carry their remaining lifetime in w, in (0, diffuseLifetime].
    bool alive = true;
    for (int32_t i = 0; i < n; ++i)
        alive = alive && p[size_t(i) * 4 + 3] > 0.0f && p[size_t(i) * 4 + 3] <= scene.params.diffuseLifetime;
    CHECK(alive);
    const std::vector<float> pp = mapAll<float>(part);
    bool ok = true;
    for (uint32_t e = 0; e < 70; ++e) {
        if (e >= region.dstOffset && e < region.dstOffset + region.elementCount)
            ok = ok && std::memcmp(&pp[size_t(e) * 4], &p[size_t(e - region.dstOffset + region.srcOffset) * 4], 16) == 0;
        else
            ok = ok && untouched(&pp[size_t(e) * 4], 16);
    }
    CHECK(ok);
    CHECK(d3d.problems() == 0);
}

// ---- Timers ---------------------------------------------------------------------------------------------------------

// Every name a stage of a step can have (the StageScope names of the pipeline, FleX's section names).
const char* const c_StageNames[] = {
    "FlattenTriangleMeshes", "Rebuild shapes", "Predicate", "ComputeBounds", "CalculateParticleHash",
    "SortParticleHash", "CreateGrid", "RecordParticles", "CollideParticles", "CollideTriangles", "CollideShapes",
    "ContinuousShockPropagation", "CalculateDensity", "SolveDensity", "SolveSprings", "SolveInflatables",
    "SolveShapes", "ApplyDeltas", "SolveContact", "UpdateVelocities", "CalculateVorticity", "SolveVelocities",
    "UpdateTriangles", "UpdateNormals", "NormalizeNormals", "Finalize", "CreateDiffuseParticles",
    "UpdateDiffuseParticles", "ClampDiffuseParticleCount", "CompactDiffuseParticles", "SmoothPositions",
    "CalculateAnisotropy", "LazyClearGrid"};

std::vector<ephys::DetailTimer> detailTimers(ephys::IPhysTimers* timers) {
    std::vector<ephys::DetailTimer> details(timers->getDetailTimerCount());
    for (uint32_t i = 0; i < details.size(); ++i) timers->getDetailTimer(details[i], i);
    return details;
}

float detailTime(const std::vector<ephys::DetailTimer>& details, const char* name) {
    for (const ephys::DetailTimer& d : details)
        if (std::strcmp(d.name, name) == 0) return d.time;
    return 0.0f;
}

bool hasDetail(const std::vector<ephys::DetailTimer>& details, const char* name) {
    for (const ephys::DetailTimer& d : details)
        if (std::strcmp(d.name, name) == 0) return true;
    return false;
}

// The values of a completed step: FleX's semantics of NvFlexGetTimers and NvFlexGetDetailTimers.
void checkTimerValues(ephys::IPhysTimers* timers, const std::vector<const char*>& expectedStages, bool print) {
    const std::vector<ephys::DetailTimer> details = detailTimers(timers);
    CHECK(details.size() >= 2);
    if (details.empty()) return;

    // The names: stage names, unique, in strcmp order, then "Total" with the device latency.
    bool known = true, ordered = true, nonNegative = true;
    for (size_t i = 0; i + 1 < details.size(); ++i) {
        bool isStage = false;
        for (const char* name : c_StageNames) isStage = isStage || std::strcmp(details[i].name, name) == 0;
        known = known && isStage;
        if (i > 0) ordered = ordered && std::strcmp(details[i - 1].name, details[i].name) < 0;
        nonNegative = nonNegative && details[i].time >= 0.0f && std::isfinite(details[i].time);
    }
    CHECK(known);
    CHECK(ordered);
    CHECK(nonNegative);
    CHECK(std::strcmp(details.back().name, "Total") == 0);
    CHECK(details.back().time == timers->getDeviceLatency());
    for (const char* name : expectedStages) {
        if (!hasDetail(details, name)) std::printf("  missing stage %s\n", name);
        CHECK(hasDetail(details, name));
    }
    ephys::DetailTimer past;
    past.name = "x";
    timers->getDetailTimer(past, uint32_t(details.size()));
    CHECK(past.name == nullptr && past.time == 0.0f);

    // NvFlexTimers: each member the sum of its stages, total the sum of the members.
    ephys::PhysTimers t;
    timers->getTimers(t);
    const float* members = &t.predict;
    float sum = 0.0f;
    bool membersNonNegative = true;
    for (int i = 0; i < 23; ++i) {
        sum += members[i];
        membersNonNegative = membersNonNegative && members[i] >= 0.0f;
    }
    CHECK(membersNonNegative);
    CHECK(t.total >= sum * (1.0f - 1.0e-5f));
    CHECK(t.total <= sum * (1.0f + 1.0e-5f) + 1.0e-12f);
    CHECK(t.collideFields == 0.0f);
    auto agrees = [](float a, float b) { return std::fabs(a - b) <= 1.0e-5f * std::max(std::fabs(a), std::fabs(b)) + 1.0e-12f; };
    CHECK(agrees(t.predict, detailTime(details, "Predicate")));
    CHECK(agrees(t.updateBounds, detailTime(details, "ComputeBounds")));
    CHECK(agrees(t.reorder, detailTime(details, "RecordParticles")));
    CHECK(agrees(t.solveVelocities, detailTime(details, "UpdateVelocities") + detailTime(details, "CalculateVorticity") +
                                      detailTime(details, "SolveVelocities")));
    CHECK(agrees(t.calculateAnisotropy, detailTime(details, "SmoothPositions") + detailTime(details, "CalculateAnisotropy")));
    CHECK(agrees(t.solveContacts, detailTime(details, "SolveContact")));

    // The stages lie inside the step, one after the other on one queue: together no longer than the step
    // (the bracket of the step also holds the stages FleX's members leave out, and the gaps).
    const float latency = timers->getDeviceLatency();
    CHECK(latency > 0.0f);
    CHECK(t.total <= latency * 1.02f + 2.0e-6f);
    if (print)
        std::printf("  step: %.1f us on the GPU, the members %.1f us (predict %.1f, solveDensities %.1f, solveContacts %.1f), "
                    "%zu detail timers\n",
                    double(latency) * 1e6, double(t.total) * 1e6, double(t.predict) * 1e6, double(t.solveDensities) * 1e6,
                    double(t.solveContacts) * 1e6, details.size());
}

void testTimers(const Env& env, D3DMessages& d3d) {
    std::printf("timers\n");
    const scenes::Scene scene = scenes::fluid();
    Rig rig = loadScene(env, scene);
    CHECK(bool(rig));
    if (!rig) return;
    ephys::PhysTimersHandle timers;
    CHECK(env.phys->createTimers(rig.solver, &timers) == nvrhi::FS_OK);
    if (!timers) return;
    const nvrhi::CommandQueue queue = env.phys->getInfo().executionQueue;
    const int errors = env.messages->errors.load();

    // Nothing yet.
    ephys::PhysTimerStatus status;
    timers->getStatus(status);
    CHECK(status.recordedSteps == 0 && status.completedStep == 0 && status.pendingSteps == 0);
    CHECK(status.queriesAllocated == 0);
    CHECK(status.supported == timers->isQueueSupported(queue));
    CHECK(!status.complete);
    CHECK(timers->getDetailTimerCount() == 0);
    CHECK(timers->getDeviceLatency() == 0.0f);
    {
        ephys::PhysTimers t;
        t.predict = 1.0f;
        timers->getTimers(t);
        CHECK(t.predict == 0.0f && t.total == 0.0f);
    }

    // Off: no query at all.
    for (int i = 0; i < 3; ++i) {
        stepScene(env, scene, rig);
        CHECK(timers->poll() == 0);
    }
    timers->getStatus(status);
    CHECK(status.recordedSteps == 0 && status.queriesAllocated == 0 && status.queriesLastStep == 0);

    rig.solver->setTimersEnabled(true);
    if (!timers->isQueueSupported(queue)) {
        // Markers only, and the step succeeds.
        std::printf("  this queue has no timestamp queries: the steps record markers only\n");
        for (int i = 0; i < 3; ++i) stepScene(env, scene, rig);
        timers->getStatus(status);
        CHECK(!status.supported && status.recordedSteps == 0 && status.queriesAllocated == 0);
        CHECK(env.messages->errors.load() == errors);
        return;
    }

    // Reading never waits: a step that was recorded and not submitted cannot have finished, and poll
    // returns at once (D3D11 issues its queries as they are recorded, so it may have finished there).
    CHECK(rig.solver->step(nullptr, scene.dt, scene.substeps) == nvrhi::FS_OK);
    ++rig.stepsTaken;
    const uint64_t polled = timers->poll();
    if (env.api != nvrhi::GraphicsAPI::D3D11) CHECK(polled == 0);
    timers->getStatus(status);
    CHECK(status.recordedSteps == 1);
    CHECK(status.supported);
    CHECK(status.queriesLastStep > 20);
    if (env.api != nvrhi::GraphicsAPI::D3D11) CHECK(status.pendingSteps == 1 && status.completedStep == 0);
    env.phys->flush();
    env.device->runGarbageCollection();

    // Results arrive within a few frames, without a wait.
    int firstFrame = -1;
    for (int frame = 0; frame < 30; ++frame) {
        stepScene(env, scene, rig);
        if (timers->poll() != 0 && firstFrame < 0) firstFrame = frame;
    }
    timers->getStatus(status);
    std::printf("  first results after %d frame(s); of 31 steps %llu measured (%u skipped while the GPU was behind), "
                "%llu read, %u pending; %u queries, %u per step\n",
                firstFrame + 1, static_cast<unsigned long long>(status.recordedSteps), status.skippedSteps,
                static_cast<unsigned long long>(status.completedStep), status.pendingSteps, status.queriesAllocated,
                status.queriesLastStep);
    CHECK(firstFrame >= 0 && firstFrame < 8);
    CHECK(status.recordedSteps + status.skippedSteps == 31);
    CHECK(status.recordedSteps >= 2);
    CHECK(status.queriesAllocated <= timers->getQueryBudget());

    // Everything measured is read once the GPU is idle, every step in full.
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    const uint64_t measured = status.recordedSteps;
    CHECK(timers->poll() == measured);
    timers->getStatus(status);
    CHECK(status.completedStep == measured && status.pendingSteps == 0 && status.complete);
    const std::vector<const char*> fluidStages = {
        "FlattenTriangleMeshes", "Predicate", "ComputeBounds", "CalculateParticleHash", "SortParticleHash", "CreateGrid",
        "RecordParticles", "CollideParticles", "CollideTriangles", "CollideShapes", "CalculateDensity", "SolveDensity",
        "ApplyDeltas", "SolveContact", "UpdateVelocities", "CalculateVorticity", "SolveVelocities", "Finalize",
        "SmoothPositions", "CalculateAnisotropy", "LazyClearGrid"};
    checkTimerValues(timers, fluidStages, true);
    CHECK(detailTimers(timers).size() == fluidStages.size() + 1);  // exactly the stages that ran, and Total

    // A second timer object of the solver reports the same.
    {
        ephys::PhysTimersHandle other;
        CHECK(env.phys->createTimers(rig.solver, &other) == nvrhi::FS_OK);
        ephys::PhysTimerStatus otherStatus;
        if (other) other->getStatus(otherStatus);
        CHECK(otherStatus.completedStep == status.completedStep && otherStatus.recordedSteps == status.recordedSteps);
        CHECK(other && other->getDeviceLatency() == timers->getDeviceLatency());
    }

    // A budget too small for a step: measured in part, without a failure or a message.
    timers->setQueryBudget(8);
    for (int i = 0; i < 3; ++i) stepScene(env, scene, rig);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    timers->poll();
    timers->getStatus(status);
    CHECK(status.completedStep == status.recordedSteps);
    CHECK(!status.complete);
    CHECK(status.queriesAllocated <= 8 && status.queriesLastStep == 8);
    CHECK(timers->getDetailTimerCount() >= 2 && timers->getDetailTimerCount() <= 8);
    timers->setQueryBudget(128);
    stepScene(env, scene, rig);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    timers->poll();
    timers->getStatus(status);
    CHECK(status.complete);

    // Off again: the queries go back to the device once read; a step records none.
    rig.solver->setTimersEnabled(false);
    const uint64_t recorded = status.recordedSteps;
    stepScene(env, scene, rig);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    timers->poll();
    timers->getStatus(status);
    CHECK(status.recordedSteps == recorded && status.queriesAllocated == 0 && status.pendingSteps == 0);
    CHECK(env.messages->errors.load() == errors);
    CHECK(d3d.problems() == 0);
}

// Steps on a command list of the application: on Vulkan a step there is read once a later step shows it was
// submitted; on D3D as soon as it finished.
void testTimersApplicationList(const Env& env, D3DMessages& d3d) {
    std::printf("timers, steps on the application's command list\n");
    const scenes::Scene scene = scenes::granular();
    Rig rig = loadScene(env, scene);
    CHECK(bool(rig));
    if (!rig) return;
    ephys::PhysTimersHandle timers;
    CHECK(env.phys->createTimers(rig.solver, &timers) == nvrhi::FS_OK);
    if (!timers) return;
    const nvrhi::CommandQueue queue = env.phys->getInfo().executionQueue;
    if (!timers->isQueueSupported(queue)) return;
    rig.solver->setTimersEnabled(true);
    env.phys->flush();  // the uploads; and on D3D11 only one command list may be open

    nvrhi::CommandListHandle commandList;
    nvrhi::CommandListParameters listParams;
    listParams.queueType = queue;
    CHECK(env.device->createCommandList(listParams, &commandList) == nvrhi::FS_OK);
    if (!commandList) return;
    for (int i = 0; i < 2; ++i) {
        commandList->open();
        CHECK(rig.solver->step(commandList, scene.dt, scene.substeps) == nvrhi::FS_OK);
        commandList->close();
        nvrhi::ICommandList* list = commandList;
        env.device->executeCommandLists(&list, 1, queue);
        CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);  // the second step finds the queries free
        env.device->runGarbageCollection();
    }
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    timers->poll();
    ephys::PhysTimerStatus status;
    timers->getStatus(status);
    CHECK(status.recordedSteps == 2);
    CHECK(status.completedStep == (env.api == nvrhi::GraphicsAPI::VULKAN ? 1u : 2u));

    // A step on the device's own list after them: everything is read once it ran.
    CHECK(rig.solver->step(nullptr, scene.dt, scene.substeps) == nvrhi::FS_OK);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    env.device->runGarbageCollection();
    CHECK(timers->poll() == 3);
    checkTimerValues(timers, {"Predicate", "CollideParticles", "SolveDensity", "SolveContact", "Finalize"}, false);
    CHECK(d3d.problems() == 0);
}

// The XPBD solver has the same timers. Its chain runs 50 iterations of 2 substeps, more stages than the
// default budget: it is measured in part.
void testTimersXPBD(const Env& env, D3DMessages& d3d) {
    std::printf("timers, the XPBD solver\n");
    const scenes::Scene scene = scenes::xpbdChain();
    Rig rig = loadScene(env, scene);
    CHECK(bool(rig));
    if (!rig) return;
    ephys::PhysTimersHandle timers;
    CHECK(env.phys->createTimers(rig.solver, &timers) == nvrhi::FS_OK);
    if (!timers || !timers->isQueueSupported(env.phys->getInfo().executionQueue)) return;
    rig.solver->setTimersEnabled(true);
    for (int i = 0; i < 2; ++i) {
        // Waiting in between: the second step gets the queries of the first back.
        stepScene(env, scene, rig);
        CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    }
    CHECK(timers->poll() == 2);
    ephys::PhysTimerStatus status;
    timers->getStatus(status);
    CHECK(!status.complete);
    CHECK(status.queriesLastStep == timers->getQueryBudget());
    const std::vector<ephys::DetailTimer> details = detailTimers(timers);
    CHECK(hasDetail(details, "SolveSprings"));
    CHECK(std::strcmp(details.back().name, "Total") == 0 && details.back().time > 0.0f);
    CHECK(d3d.problems() == 0);
}

// Steps that run no particle stage (a domain without an active particle: the step only brings the
// geometry library up to date, FlattenTriangleMeshes) are measured as well: the step's bracket and that one
// stage.
void testTimersMaintenance(const Env& env, D3DMessages& d3d) {
    std::printf("timers, steps without active particles\n");
    ephys::DomainDesc domainDesc;
    domainDesc.maxParticles = 64;
    ephys::PhysDomainHandle domain;
    CHECK(env.phys->createDomain(domainDesc, &domain) == nvrhi::FS_OK);
    if (!domain) return;
    ephys::PhysSolverHandle solver;
    CHECK(env.phys->createSolver(domain, ephys::SolverDesc(), &solver) == nvrhi::FS_OK);
    if (!solver) return;
    ephys::PhysTimersHandle timers;
    CHECK(env.phys->createTimers(solver, &timers) == nvrhi::FS_OK);
    if (!timers) return;
    const nvrhi::CommandQueue queue = env.phys->getInfo().executionQueue;
    solver->setTimersEnabled(true);
    for (int i = 0; i < 3; ++i) {
        CHECK(solver->step(nullptr, 1.0f / 60.0f, 2) == nvrhi::FS_OK);
        CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
        env.device->runGarbageCollection();
    }
    timers->poll();
    ephys::PhysTimerStatus status;
    timers->getStatus(status);
    if (!timers->isQueueSupported(queue)) {
        CHECK(!status.supported && status.recordedSteps == 0 && status.skippedSteps == 3);
        return;
    }
    CHECK(status.supported && status.recordedSteps == 3 && status.completedStep == 3 && status.complete);
    CHECK(status.queriesLastStep == 2);
    const std::vector<ephys::DetailTimer> details = detailTimers(timers);
    CHECK(details.size() == 2);
    if (details.size() == 2) {
        CHECK(std::strcmp(details[0].name, "FlattenTriangleMeshes") == 0 && details[0].time >= 0.0f);
        CHECK(std::strcmp(details[1].name, "Total") == 0 && details[1].time >= details[0].time);
    }
    ephys::PhysTimers t;
    timers->getTimers(t);
    CHECK(t.total == 0.0f);  // FlattenTriangleMeshes counts in no member of NvFlexTimers
    std::printf("  step bracket %.1f us, FlattenTriangleMeshes %.1f us\n", double(timers->getDeviceLatency()) * 1e6,
                details.empty() ? 0.0 : double(details[0].time) * 1e6);
    CHECK(d3d.problems() == 0);
}

}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);  // nothing is lost when the process dies
    Options options;
    parse(argc, argv, options);
    if (!options.rest.empty()) {
        std::printf("usage: %s <d3d11|d3d12|vulkan> [compute] [--adapter <n>] [--shaders <dir>]\n", argv[0]);
        return 1;
    }
    return runWithDevice(options, [&](Env& env, D3DMessages& d3d) {
        std::printf("engine queue: %s\n",
                    env.phys->getInfo().executionQueue == nvrhi::CommandQueue::Compute ? "compute" : "graphics");
        testObjects(env, d3d);
        testTimersMaintenance(env, d3d);
        testFluid(env, d3d);
        testDiffuse(env, d3d);
        testTimers(env, d3d);
        testTimersApplicationList(env, d3d);
        testTimersXPBD(env, d3d);
    });
}
