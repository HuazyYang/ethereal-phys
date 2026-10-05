// ethereal_phys_xpbd_test: the XPBD solver (IPhysSolverXPBD) on a real nvrhi device, headless, without a
// reference to compare with. The comparison with FleX's XPBD build is parity_test.cpp --xpbd.
//
//     ethereal_phys_xpbd_test <d3d11|d3d12|vulkan> [--adapter <n>] [--shaders <dir>]
//
// The device is created as in the other device tests: debug runtime, nvrhi validation layer, and every
// message of either, or of the engine, is a failure unless the test provoked it.
//
// What it checks:
//   - the interface: an XPBD solver answers IPhysSolverXPBD and IPhysSolver, a PBD solver only
//     IPhysSolver; the compliance block round-trips; the multiplier buffers are null while their
//     constraint set is empty and exist after a step otherwise; resetMultipliers zeroes them;
//   - the multipliers are cleared at the start of every substep: a stage callback copies them at the first
//     IterationStart and the last IterationEnd of each substep;
//   - the four reference scenes of FleX's XPBD branch (solver_scenes.h: chain, beam, cloth, balloon) step
//     without a NaN or a validation message and do what the reference reports of them;
//   - substep convergence, the decisive property of XPBD: the 64 x 64 cloth at rest after the same
//     simulated time with 8, 16 and 32 substeps has the same shape within 0.1%; the PBD solver on the same
//     data does not (it stiffens with every substep);
//   - compliance: a stiffer coefficient or a smaller compliance stretches a hanging chain less, by the
//     amount alpha * m * g predicts; a balloon with volumeCompliance 0 keeps its rest volume and loses more
//     of it as the compliance grows;
//   - determinism: two runs give the same bits where no kernel adds with floating-point atomics, and agree
//     closely where one does.
//
// Exit code: 0 passed, 1 failed, 77 the graphics API has no usable device on this machine (skipped).

#include <ethereal/phys/device.h>
#include <ethereal/phys/solver.h>

#include "solver_support.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

using namespace support;
using ephys::SolverStage;

bool allFinite(const std::vector<float>& values) {
    for (float v : values)
        if (!std::isfinite(v)) return false;
    return true;
}

float maxAbsDifference(const std::vector<float>& a, const std::vector<float>& b) {
    if (a.size() != b.size()) return INFINITY;
    float worst = 0.0f;
    for (size_t i = 0; i < a.size(); ++i) {
        const float d = std::fabs(a[i] - b[i]);
        if (!(d <= worst)) worst = d;  // a NaN becomes the result
    }
    return worst;
}

float maxSpeed(const std::vector<float>& velocities) {
    float worst = 0.0f;
    for (size_t i = 0; i + 2 < velocities.size(); i += 3)
        worst = std::max(worst, std::sqrt(velocities[i] * velocities[i] + velocities[i + 1] * velocities[i + 1] +
                                          velocities[i + 2] * velocities[i + 2]));
    return worst;
}

scenes::Vec3 particle(const std::vector<float>& positions, size_t i) {
    return {positions[i * 4], positions[i * 4 + 1], positions[i * 4 + 2]};
}

ephys::PhysSolverXPBDHandle xpbdOf(ephys::IPhysSolver* solver) {
    ephys::PhysSolverXPBDHandle xpbd;
    if (solver) solver->QueryInterface(NVRHI_IID_PPV_ARGS(&xpbd));
    return xpbd;
}

// `count` floats of an nvrhi buffer of the solver, after everything recorded so far has run.
std::vector<float> readFloats(const Env& env, nvrhi::IBuffer* buffer, uint32_t count) {
    CHECK(buffer != nullptr);
    if (!buffer || count == 0) return {};
    ephys::PhysBufferDesc desc;
    desc.elementCount = count;
    desc.elementStride = 4;
    desc.use = ephys::BufferUse::Structured;
    desc.debugName = "test multiplier copy";
    PhysBufferHandle copy;
    CHECK(env.phys->createBuffer(desc, &copy) == nvrhi::FS_OK);
    if (!copy) return {};
    env.submitEngineWork();  // the step first; and on D3D11 only one command list may be open
    nvrhi::CommandListHandle commandList;
    CHECK(env.device->createCommandList(nvrhi::CommandListParameters(), &commandList) == nvrhi::FS_OK);
    if (!commandList) return {};
    commandList->open();
    commandList->copyBuffer(copy->getRHIBuffer(), 0, buffer, 0, uint64_t(count) * 4);
    commandList->close();
    nvrhi::ICommandList* list = commandList;
    env.executeOnGraphics(list);
    return env.fetch<float>(copy, 0, count);
}

struct SceneState {
    std::vector<float> positions;
    std::vector<float> velocities;
};

SceneState readState(const Env& env, const scenes::Scene& scene, const Rig& rig) {
    SceneState state;
    state.positions = readPositions(env, rig, scene.numParticles());
    state.velocities = readVelocities(env, rig, scene.numParticles());
    return state;
}

// Steps a scene and checks what holds for every scene: no NaN, the substep count, pinned particles where
// they were. Returns the state after `steps` steps (and the rig in *keep).
SceneState runScene(const Env& env, D3DMessages& d3d, const scenes::Scene& scene, uint32_t steps,
                    const RigOptions& options = RigOptions(), Rig* keep = nullptr) {
    SceneState state;
    Rig rig = loadScene(env, scene, options);
    CHECK(bool(rig));
    if (!rig) return state;
    for (uint32_t i = 0; i < steps; ++i) stepScene(env, scene, rig);
    CHECK(rig.solver->getSubstepCount() == uint64_t(steps) * scene.substeps);
    state = readState(env, scene, rig);
    CHECK(d3d.problems() == 0);
    CHECK(state.positions.size() == size_t(scene.numParticles()) * 4);
    CHECK(allFinite(state.positions));
    CHECK(allFinite(state.velocities));
    bool pinned = true;
    for (size_t i = 0; i < state.positions.size(); i += 4)
        if (scene.positions[i + 3] == 0.0f) pinned = pinned && std::memcmp(&state.positions[i], &scene.positions[i], 16) == 0;
    CHECK(pinned);
    if (keep) *keep = rig;
    return state;
}

// A chain of `links` unit-mass particles hanging straight down from a fixed one, one coefficient on every
// link. At rest link j (counted from the bottom, 1-based) carries j * m * g, so with the local relaxation
// (which divides by at most one here: a particle of a chain has two constraints and kSOR = 1/2) the chain
// stretches by alpha * m * g * links * (links + 1) / 2.
scenes::Scene hangingChain(float coefficient, int links, uint32_t substeps) {
    scenes::Scene s = scenes::xpbdScene("hanging_chain");
    const int32_t phase = ephys::makePhase(0, 0);
    for (int i = 0; i <= links; ++i) scenes::addParticle(s, {0.0f, 2.5f - 0.1f * float(i), 0.0f}, i == 0 ? 0.0f : 1.0f, {0, 0, 0}, phase);
    for (int i = 0; i < links; ++i) scenes::addSpring(s, i, i + 1, coefficient);
    s.params.radius = 0.05f;
    s.params.damping = 5.0f;
    s.params.maxAcceleration = FLT_MAX;
    s.params.numIterations = 50;
    s.substeps = substeps;
    scenes::finishDemoScene(s);
    return s;
}

// ---- The interface ----------------------------------------------------------------------------------------------

void testInterface(const Env& env, D3DMessages& d3d) {
    std::printf("interface\n");
    ephys::DomainDesc domainDesc;
    domainDesc.maxParticles = 256;
    ephys::PhysDomainHandle domain;
    CHECK(env.phys->createDomain(domainDesc, &domain) == nvrhi::FS_OK);
    if (!domain) return;

    ephys::SolverDesc desc;
    ephys::PhysSolverHandle pbd;
    CHECK(env.phys->createSolver(domain, desc, &pbd) == nvrhi::FS_OK);
    desc.mode = ephys::SolverMode::XPBD;
    ephys::PhysSolverHandle solver;
    CHECK(env.phys->createSolver(domain, desc, &solver) == nvrhi::FS_OK);
    if (!pbd || !solver) return;
    const nvrhi::FLONG refs = refCount(solver.Get());

    // Who answers what.
    {
        ephys::PhysSolverXPBDHandle none;
        CHECK(pbd->QueryInterface(NVRHI_IID_PPV_ARGS(&none)) == nvrhi::FE_NOINTERFACE && none == nullptr);
        ephys::PhysSolverHandle asSolver;
        CHECK(pbd->QueryInterface(NVRHI_IID_PPV_ARGS(&asSolver)) == nvrhi::FS_OK && asSolver == pbd);
    }
    ephys::PhysSolverXPBDHandle xpbd;
    CHECK(solver->QueryInterface(NVRHI_IID_PPV_ARGS(&xpbd)) == nvrhi::FS_OK && xpbd != nullptr);
    if (!xpbd) return;
    CHECK(refCount(solver.Get()) == refs + 1);
    {
        ephys::PhysSolverHandle back;
        CHECK(xpbd->QueryInterface(NVRHI_IID_PPV_ARGS(&back)) == nvrhi::FS_OK && back == solver);
        nvrhi::AutoPtr<ephys::IPhysDomain> notADomain;
        CHECK(xpbd->QueryInterface(NVRHI_IID_PPV_ARGS(&notADomain)) == nvrhi::FE_NOINTERFACE);
    }
    CHECK(static_cast<ephys::IPhysSolver*>(xpbd.Get()) == solver.Get());
    CHECK(solver->getMode() == ephys::SolverMode::XPBD && pbd->getMode() == ephys::SolverMode::PBD);
    CHECK(solver->getDesc().mode == ephys::SolverMode::XPBD);

    // The compliance block: the defaults of params.h, and what is set comes back unchanged (no clamping:
    // that happens when a step reads it).
    {
        const ephys::XpbdComplianceParams defaults;
        CHECK(std::memcmp(&xpbd->getCompliance(), &defaults, sizeof(defaults)) == 0);
        CHECK(xpbd->getCompliance().stiffnessMin == 1.0e3f && xpbd->getCompliance().stiffnessMax == 1.0e9f &&
              xpbd->getCompliance().springDamping == 0.0f && xpbd->getCompliance().volumeCompliance == 0.0f);
        ephys::XpbdComplianceParams odd;
        odd.stiffnessMin = -5.0f;
        odd.stiffnessMax = 2.0f;
        odd.springDamping = 0.25f;
        odd.volumeCompliance = 1.0e-4f;
        xpbd->setCompliance(odd);
        CHECK(std::memcmp(&xpbd->getCompliance(), &odd, sizeof(odd)) == 0);
        xpbd->setCompliance(defaults);
        CHECK(std::memcmp(&xpbd->getCompliance(), &defaults, sizeof(defaults)) == 0);
    }

    // No springs and no inflatables: no multipliers, before and after a step, and nothing to reset.
    CHECK(xpbd->getSpringMultipliers() == nullptr && xpbd->getInflatableMultipliers() == nullptr);
    {
        scenes::Scene grid;
        grid.params = scenes::defaultParams();
        scenes::Jitter random(3);
        scenes::addParticleGrid(grid, {0, 1, 0}, 4, 4, 4, 0.2f, {0, 0, 0}, 1.0f, ephys::makePhase(0, ephys::PhaseSelfCollide),
                                0.0f, random);
        PhysBufferHandle source = env.upload(grid.positions, 16);
        CHECK(domain->getParticles()->upload(nullptr, ParticleArray::Position, source, nullptr) == nvrhi::FS_OK);
        CHECK(domain->getParticles()->setActiveCount(grid.numParticles()) == nvrhi::FS_OK);
        domain->setParams(grid.params);
    }
    CHECK(solver->step(nullptr, 1.0f / 60.0f, 2) == nvrhi::FS_OK);
    CHECK(xpbd->getSpringMultipliers() == nullptr && xpbd->getInflatableMultipliers() == nullptr);
    CHECK(xpbd->resetMultipliers(nullptr) == nvrhi::FS_OK);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    CHECK(d3d.problems() == 0);

    xpbd = nullptr;
    CHECK(refCount(solver.Get()) == refs);
}

// After a step: a multiplier buffer for each non-empty constraint set, with values in it; resetMultipliers
// clears them; the two halves of a spring between two particles with mass hold the same bits.
void testMultipliers(const Env& env, D3DMessages& d3d) {
    std::printf("multipliers\n");
    {
        const scenes::Scene scene = hangingChain(0.3f, 10, 2);
        Rig rig = loadScene(env, scene);
        CHECK(bool(rig));
        if (!rig) return;
        ephys::PhysSolverXPBDHandle xpbd = xpbdOf(rig.solver);
        CHECK(xpbd != nullptr);
        if (!xpbd) return;
        CHECK(xpbd->getSpringMultipliers() == nullptr);  // sized by the first step
        stepScene(env, scene, rig);
        nvrhi::IBuffer* springs = xpbd->getSpringMultipliers();
        CHECK(springs != nullptr && xpbd->getInflatableMultipliers() == nullptr);
        if (!springs) return;
        CHECK(springs->getDesc().structStride == 4 && springs->getDesc().byteSize >= uint64_t(2) * scene.numSprings() * 4);
        const uint32_t n = 2 * scene.numSprings();
        std::vector<float> lambdas = readFloats(env, springs, n);
        CHECK(allFinite(lambdas));
        // The half-springs of a chain sorted by particle are in spring order, so halves 2s and 2s+1 are one
        // spring. Spring 0 hangs from the pinned particle, whose half is never solved (it stays 0).
        bool pairs = lambdas.size() == n;
        uint32_t nonZero = 0;
        for (uint32_t s = 1; pairs && s < scene.numSprings(); ++s) {
            pairs = std::memcmp(&lambdas[2 * s], &lambdas[2 * s + 1], 4) == 0;
            nonZero += lambdas[2 * s] != 0.0f ? 1 : 0;
        }
        CHECK(pairs);
        CHECK(lambdas.size() == n && lambdas[0] == 0.0f && lambdas[1] != 0.0f);
        CHECK(nonZero == scene.numSprings() - 1);
        // A chain hanging under gravity: every multiplier pulls (negative in this sign convention).
        bool pulling = true;
        for (uint32_t i = 1; i < lambdas.size(); ++i) pulling = pulling && lambdas[i] < 0.0f;
        CHECK(pulling);
        std::printf("  hanging chain after one step: multipliers %.3e (top link) .. %.3e (bottom link)\n",
                    double(lambdas[1]), double(lambdas[n - 1]));

        CHECK(xpbd->resetMultipliers(nullptr) == nvrhi::FS_OK);
        lambdas = readFloats(env, springs, n);
        CHECK(lambdas == std::vector<float>(n, 0.0f));
        CHECK(d3d.problems() == 0);
    }
    {
        // Springs and inflatables.
        const scenes::Scene scene = scenes::xpbdBalloon();
        Rig rig = loadScene(env, scene);
        CHECK(bool(rig));
        if (!rig) return;
        ephys::PhysSolverXPBDHandle xpbd = xpbdOf(rig.solver);
        if (!xpbd) return;
        stepScene(env, scene, rig);
        nvrhi::IBuffer* springs = xpbd->getSpringMultipliers();
        nvrhi::IBuffer* volumes = xpbd->getInflatableMultipliers();
        CHECK(springs != nullptr && volumes != nullptr);
        if (!springs || !volumes) return;
        const uint32_t numInflatables = uint32_t(scene.inflatables.size());
        std::vector<float> lambdas = readFloats(env, volumes, numInflatables);
        CHECK(allFinite(lambdas));
        uint32_t nonZero = 0;
        for (float v : lambdas) nonZero += v != 0.0f ? 1 : 0;
        CHECK(nonZero == numInflatables);
        std::printf("  balloons after one step: volume multipliers %.3e %.3e %.3e %.3e\n", double(lambdas[0]),
                    double(lambdas[1]), double(lambdas[2]), double(lambdas[3]));
        CHECK(xpbd->resetMultipliers(nullptr) == nvrhi::FS_OK);
        CHECK(readFloats(env, volumes, numInflatables) == std::vector<float>(numInflatables, 0.0f));
        const std::vector<float> springLambdas = readFloats(env, springs, 2 * scene.numSprings());
        CHECK(springLambdas == std::vector<float>(size_t(2) * scene.numSprings(), 0.0f));
        CHECK(d3d.problems() == 0);
    }
}

// ---- Multipliers start at zero in every substep ----------------------------------------------------------------

// Copies the multipliers at the first IterationStart and at the last IterationEnd of every substep into a
// buffer of the test, one slot each.
class MultiplierProbe final : public nvrhi::ObjectImpl<ephys::IPhysSolverStageCallback> {
 public:
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(MultiplierProbe)
    NVRHI_IMPLEMENTS_INTERFACE(ephys::IPhysSolverStageCallback)
    NVRHI_END_INTERFACE_TABLE()

    void onSolverStage(const ephys::SolverStageContext& context) noexcept override {
        const bool first = context.stage == SolverStage::IterationStart && context.iteration == 0;
        const bool last = context.stage == SolverStage::IterationEnd && int(context.iteration) == iterations - 1;
        if (!first && !last) return;
        // Found as an application finds it: QueryInterface on the solver of the context.
        ephys::PhysSolverXPBDHandle xpbd = xpbdOf(context.solver);
        if (!xpbd) {
            ++failures;
            return;
        }
        const uint32_t slot = (step * context.numSubsteps + context.substep) * 2 + (last ? 1 : 0);
        const uint64_t bytes = uint64_t(count) * 4;
        if (nvrhi::IBuffer* springs = xpbd->getSpringMultipliers())
            context.commandList->copyBuffer(snapshots, uint64_t(slot) * bytes, springs, 0, bytes);
        else
            ++failures;
        if (nvrhi::IBuffer* volumes = xpbd->getInflatableMultipliers())
            context.commandList->copyBuffer(volumeSnapshots, uint64_t(slot) * 4 * volumeCount, volumes, 0, uint64_t(volumeCount) * 4);
        else
            ++failures;
        ++calls;
    }

    nvrhi::IBuffer* snapshots = nullptr;
    nvrhi::IBuffer* volumeSnapshots = nullptr;
    uint32_t count = 0;
    uint32_t volumeCount = 0;
    int iterations = 0;
    uint32_t step = 0;
    int calls = 0;
    int failures = 0;
};

void testClearedEverySubstep(const Env& env, D3DMessages& d3d) {
    std::printf("multipliers cleared at the start of every substep\n");
    // A small balloon (springs and an inflatable), three substeps, three iterations.
    scenes::Scene scene = scenes::xpbdBalloon();
    scene.params.numIterations = 3;
    scene.substeps = 3;
    Rig rig = loadScene(env, scene);
    CHECK(bool(rig));
    if (!rig) return;
    stepScene(env, scene, rig);  // sizes the multiplier buffers
    ephys::PhysSolverXPBDHandle xpbd = xpbdOf(rig.solver);
    if (!xpbd || !xpbd->getSpringMultipliers()) return;

    const uint32_t steps = 2;
    const uint32_t slots = steps * scene.substeps * 2;
    const uint32_t count = 2 * scene.numSprings();
    const uint32_t volumeCount = uint32_t(scene.inflatables.size());
    PhysBufferHandle snapshots = env.create(ephys::BufferUse::Structured, slots * count, 4);
    PhysBufferHandle volumeSnapshots = env.create(ephys::BufferUse::Structured, slots * volumeCount, 4);
    if (!snapshots || !volumeSnapshots) return;

    auto probe = MAKE_RC_OBJ_PTR(MultiplierProbe);
    probe->snapshots = snapshots->getRHIBuffer();
    probe->volumeSnapshots = volumeSnapshots->getRHIBuffer();
    probe->count = count;
    probe->volumeCount = volumeCount;
    probe->iterations = scene.params.numIterations;
    CHECK(rig.solver->setStageCallback(SolverStage::IterationStart, probe, nullptr) == nvrhi::FS_OK);
    CHECK(rig.solver->setStageCallback(SolverStage::IterationEnd, probe, nullptr) == nvrhi::FS_OK);
    for (uint32_t s = 0; s < steps; ++s) {
        probe->step = s;
        stepScene(env, scene, rig);
    }
    CHECK(probe->failures == 0 && probe->calls == int(slots));
    CHECK(rig.solver->setStageCallback(SolverStage::IterationStart, nullptr, nullptr) == nvrhi::FS_OK);
    CHECK(rig.solver->setStageCallback(SolverStage::IterationEnd, nullptr, nullptr) == nvrhi::FS_OK);

    const std::vector<float> springs = env.fetch<float>(snapshots, 0, slots * count);
    const std::vector<float> volumes = env.fetch<float>(volumeSnapshots, 0, slots * volumeCount);
    CHECK(springs.size() == size_t(slots) * count && volumes.size() == size_t(slots) * volumeCount);
    if (springs.size() != size_t(slots) * count || volumes.size() != size_t(slots) * volumeCount) return;
    bool clearedAtStart = true, filledAtEnd = true;
    for (uint32_t slot = 0; slot < slots; ++slot) {
        const bool end = (slot & 1) != 0;
        uint32_t nonZero = 0, volumeNonZero = 0;
        for (uint32_t i = 0; i < count; ++i) nonZero += springs[size_t(slot) * count + i] != 0.0f ? 1 : 0;
        for (uint32_t i = 0; i < volumeCount; ++i) volumeNonZero += volumes[size_t(slot) * volumeCount + i] != 0.0f ? 1 : 0;
        if (end) {
            filledAtEnd = filledAtEnd && nonZero > count / 2 && volumeNonZero == volumeCount;
        } else {
            clearedAtStart = clearedAtStart && nonZero == 0 && volumeNonZero == 0;
        }
    }
    std::printf("  %u substeps: all multipliers zero at every first IterationStart: %s; non-zero at every last "
                "IterationEnd: %s\n",
                steps * scene.substeps, clearedAtStart ? "yes" : "NO", filledAtEnd ? "yes" : "NO");
    CHECK(clearedAtStart);
    CHECK(filledAtEnd);
    CHECK(d3d.problems() == 0);
}

// ---- The reference scenes ----------------------------------------------------------------------------------------

// The largest relative error of a link of the chain, and its lowest point.
void chainShape(const scenes::Scene& scene, const std::vector<float>& positions, float* linkError, float* lowest) {
    *linkError = 0.0f;
    *lowest = INFINITY;
    for (uint32_t s = 0; s < scene.numSprings(); ++s) {
        const scenes::Vec3 a = particle(positions, size_t(scene.springIndices[2 * s]));
        const scenes::Vec3 b = particle(positions, size_t(scene.springIndices[2 * s + 1]));
        *linkError = std::max(*linkError, std::fabs(scenes::length(a - b) - scene.springLengths[s]) / scene.springLengths[s]);
    }
    for (size_t i = 0; i < positions.size(); i += 4) *lowest = std::min(*lowest, positions[i + 1]);
}

// Released horizontally, the chain swings down and whips taut at the bottom of the swing. The largest link
// error and the lowest point over the first three seconds, after every step.
void swingChain(const Env& env, D3DMessages& d3d, const scenes::Scene& scene, float* worstLink, float* overshoot) {
    *worstLink = 0.0f;
    *overshoot = -INFINITY;
    Rig rig = loadScene(env, scene);
    CHECK(bool(rig));
    if (!rig) return;
    const float taut = 2.5f - 1.9f;  // 19 links of 0.1 below the support
    for (uint32_t step = 0; step < 180; ++step) {
        stepScene(env, scene, rig);
        const std::vector<float> positions = readPositions(env, rig, scene.numParticles());
        CHECK(allFinite(positions));
        float link = 0.0f, low = 0.0f;
        chainShape(scene, positions, &link, &low);
        *worstLink = std::max(*worstLink, link);
        *overshoot = std::max(*overshoot, taut - low);
    }
    CHECK(d3d.problems() == 0);
}

void testChain(const Env& env, D3DMessages& d3d) {
    scenes::Scene scene = scenes::xpbdChain();
    float link50 = 0.0f, low50 = 0.0f, link10 = 0.0f, low10 = 0.0f;
    swingChain(env, d3d, scene, &link50, &low50);
    scene.params.numIterations = 10;
    swingChain(env, d3d, scene, &link10, &low10);
    std::printf("  %-16s 3 s: largest link error %.2f%% at 50 iterations, %.2f%% at 10; lowest point %.4f m and %.4f m "
                "below the taut length\n",
                scene.name.c_str(), double(link50) * 100.0, double(link10) * 100.0, double(low50), double(low10));
    // The reference reports 1.3 cm below the taut length at 50 iterations and 8 cm at 10, and a largest link
    // error of 0.8% "mid-swing" (5.2% at 10): sampled differently (this takes the worst step, the whip at the
    // bottom of the swing); parity_test --xpbd compares the trajectories themselves. What must hold: an
    // unconverged solve is softer (the artificial compliance the paper describes), the converged one holds
    // the links to a few percent.
    CHECK(link50 < 0.03f && low50 > 0.0f && low50 < 0.03f);
    CHECK(link10 > 2.0f * link50 && low10 > 2.0f * low50);
}

void testBeam(const Env& env, D3DMessages& d3d) {
    const scenes::Scene scene = scenes::xpbdBeam();
    const SceneState state = runScene(env, d3d, scene, 240);
    if (state.positions.empty()) return;
    // The tip: the free end of the beam's middle row, against where it started.
    const size_t tip = size_t(3 * 40 + 39);
    const float deflection = scene.positions[tip * 4 + 1] - state.positions[tip * 4 + 1];
    std::printf("  %-16s 4 s: tip deflection %.4f m, max speed %.4f\n", scene.name.c_str(), double(deflection),
                double(maxSpeed(state.velocities)));
    // The reference measured 0.350 m at rest (PBD: 0.111 m at 20 iterations).
    CHECK(deflection > 0.25f && deflection < 0.45f);
}

void testCloth(const Env& env, D3DMessages& d3d) {
    const scenes::Scene scene = scenes::xpbdCloth();
    const SceneState state = runScene(env, d3d, scene, 120);
    if (state.positions.empty()) return;
    float lowest = INFINITY;
    for (size_t i = 1; i < state.positions.size(); i += 4) lowest = std::min(lowest, state.positions[i]);
    const float drop = scenes::XpbdClothConstants::kPinHeight - lowest;
    std::printf("  %-16s 2 s in the wind: drop %.4f m, max speed %.4f\n", scene.name.c_str(), double(drop),
                double(maxSpeed(state.velocities)));
    // Rest height 3.15 m, the pins 4.9 m above the ground. In the wind the sheet swings in depth and its hem
    // rises and falls (the reference reports a drop of about 3.69 m averaged over its gusts, which
    // parity_test --xpbd compares step by step); it stays stretched and off the ground.
    CHECK(drop > 2.5f && drop < 4.85f);
}

float meshVolume(const std::vector<int32_t>& triangles, size_t first, size_t count, const std::vector<float>& positions) {
    float volume = 0.0f;
    for (size_t t = first; t < first + count; ++t) {
        scenes::Vec3 v[3];
        for (size_t k = 0; k < 3; ++k) v[k] = particle(positions, size_t(triangles[t * 3 + k]));
        volume += scenes::dot(v[0], scenes::cross(v[1] - v[0], v[2] - v[0]));
    }
    return volume;
}

// The volume of each balloon relative to its rest volume, and its height.
void balloonShapes(const scenes::Scene& scene, const std::vector<float>& positions, std::vector<float>& volumes,
                   std::vector<float>& heights) {
    volumes.clear();
    heights.clear();
    for (const ephys::InflatableDesc& b : scene.inflatables) {
        volumes.push_back(meshVolume(scene.triangleIndices, size_t(b.startTri), size_t(b.numTris), positions) / b.restVolume);
        float low = INFINITY, high = -INFINITY;
        for (size_t t = size_t(b.startTri); t < size_t(b.startTri + b.numTris); ++t)
            for (size_t k = 0; k < 3; ++k) {
                const float y = positions[size_t(scene.triangleIndices[t * 3 + k]) * 4 + 1];
                low = std::min(low, y);
                high = std::max(high, y);
            }
        heights.push_back(high - low);
    }
}

void testBalloon(const Env& env, D3DMessages& d3d) {
    const scenes::Scene scene = scenes::xpbdBalloon();
    const SceneState state = runScene(env, d3d, scene, 240);
    if (state.positions.empty()) return;
    std::vector<float> volumes, heights;
    balloonShapes(scene, state.positions, volumes, heights);
    std::printf("  %-16s 4 s: heights %.3f %.3f %.3f %.3f, volume / rest %.4f %.4f %.4f %.4f\n", scene.name.c_str(),
                double(heights[0]), double(heights[1]), double(heights[2]), double(heights[3]), double(volumes[0]),
                double(volumes[1]), double(volumes[2]), double(volumes[3]));
    // The reference: heights 0.43 0.52 0.70 0.87 at rest, the volume 1.000 x rest throughout. The softest
    // balloon flattens most.
    for (size_t i = 0; i < volumes.size(); ++i) CHECK(std::fabs(volumes[i] - 1.0f) < 0.01f);
    for (size_t i = 1; i < heights.size(); ++i) CHECK(heights[i] > heights[i - 1]);
    CHECK(heights[0] < 0.8f && heights[3] > 0.75f && heights[3] <= 1.01f);
}

// ---- Substep convergence ----------------------------------------------------------------------------------------
//
// XPBD's alpha~ = alpha / dt^2 makes the stiffness of a converged solve independent of the substep, so a
// sheet at rest under gravity takes the same shape at every substep count. PBD's per-iteration multiplier
// does not: each substep corrects once more, and the sheet gets stiffer with every substep. The sheet is
// the 64 x 64 cloth of xpbdCloth in still air (no wind, drag or lift) with viscous damping 4 so that it comes
// to rest, after the same simulated time at 8, 16 and 32 substeps of 20 iterations.
//
// Two hangings:
//
// - The curtain: the whole top row pinned, the out-of-plane ripple removed. Nothing in it is under lateral
//   compression, so its rest shape is unique, and that is where the 0.1% of the reference is checked.
// - The reference's own hanging, by the two top corners, with the ripple (the scene's comment on
//   kXpbdSubsteps reports 8/16/32 substeps "within 0.1%" there, measured in the wind). Below the top row the
//   sheet is laterally compressed and buckles into folds, and which folds it settles into is a bifurcation,
//   not a property of the time integration: measured on one GPU at 10, 14 s and with the sheet shifted or
//   flattened, the 32-substep rest shape moved by up to 0.5% from run to run while the 8- and 16-substep ones
//   held to 0.03%. It is checked at 0.5% (PBD spreads by 5% and more on it).

// The shape of the hanging cloth, in the quantities of the reference (xpbdcloth.h), with D the pin
// separation (the rest width, 3.15 m) and depths measured down from the top-left pin.
struct ClothShape {
    float topRow = 0.0f;     // the length of the top row
    float drop = 0.0f;       // the depth of the lowest particle: the sag of the sheet
    float meanDepth = 0.0f;  // the depth of the centre of mass
    float hemDepth = 0.0f;   // the mean depth of the bottom row
    float dip = 0.0f;        // the depth of the middle of the top row, over D
    float speed = 0.0f;      // the fastest particle: how far from rest the sheet is
};

ClothShape clothShape(const std::vector<float>& positions, const std::vector<float>& velocities) {
    using C = scenes::XpbdClothConstants;
    const int dim = C::kDim;
    const float d = float(dim - 1) * C::kSpacing;
    const float pin = positions[1];
    ClothShape shape;
    float lowest = INFINITY;
    double sum = 0.0, hem = 0.0;
    for (size_t i = 1; i < positions.size(); i += 4) {
        lowest = std::min(lowest, positions[i]);
        sum += positions[i];
    }
    for (int col = 0; col < dim; ++col) hem += positions[size_t((dim - 1) * dim + col) * 4 + 1];
    for (int col = 0; col + 1 < dim; ++col)
        shape.topRow += scenes::length(particle(positions, size_t(col + 1)) - particle(positions, size_t(col)));
    shape.drop = pin - lowest;
    shape.meanDepth = pin - float(sum / double(positions.size() / 4));
    shape.hemDepth = pin - float(hem / double(dim));
    const float middle = 0.5f * (positions[size_t(dim / 2 - 1) * 4 + 1] + positions[size_t(dim / 2) * 4 + 1]);
    shape.dip = (pin - middle) / d;
    shape.speed = maxSpeed(velocities);
    return shape;
}

// (max - min) / mean of three values.
float relativeSpread(float a, float b, float c) {
    const float low = std::min({a, b, c}), high = std::max({a, b, c});
    return (high - low) / std::fabs((a + b + c) / 3.0f);
}

// The cloth at rest after `seconds` at 8, 16 and 32 substeps, with the XPBD solver ([0]) and the PBD solver
// on the same data ([1]).
void settleCloth(const Env& env, D3DMessages& d3d, const scenes::Scene& base, float seconds, ClothShape shapes[2][3]) {
    static const uint32_t c_Substeps[3] = {8, 16, 32};
    const uint32_t steps = uint32_t(seconds / base.dt + 0.5f);
    for (int formulation = 0; formulation < 2; ++formulation) {
        RigOptions options;
        options.mode = formulation == 0 ? RigOptions::ForceXPBD : RigOptions::ForcePBD;
        for (int k = 0; k < 3; ++k) {
            scenes::Scene scene = base;
            scene.substeps = c_Substeps[k];
            const auto start = std::chrono::steady_clock::now();
            const SceneState state = runScene(env, d3d, scene, steps, options);
            if (state.positions.empty()) return;
            const ClothShape s = clothShape(state.positions, state.velocities);
            shapes[formulation][k] = s;
            std::printf("    %s %2u substeps: drop %.4f m, mean depth %.4f m, hem depth %.4f m, top row %.4f m, dip/D %.4f "
                        "(max speed %.1e m/s; %.1f s)\n",
                        formulation == 0 ? "XPBD" : "PBD ", c_Substeps[k], double(s.drop), double(s.meanDepth),
                        double(s.hemDepth), double(s.topRow), double(s.dip), double(s.speed),
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
        }
    }
}

void testSubstepConvergence(const Env& env, D3DMessages& d3d) {
    std::printf("substep convergence: the 64 x 64 cloth at rest after the same time at 8, 16 and 32 substeps\n");
    using C = scenes::XpbdClothConstants;
    const float restHeight = float(C::kDim - 1) * C::kSpacing;

    {
        std::printf("  the curtain (top row pinned, flat, still air), 8 s\n");
        scenes::Scene curtain = scenes::xpbdCloth(false, 4.0f);
        curtain.name = "xpbd_curtain";
        for (int col = 0; col < C::kDim; ++col) curtain.positions[size_t(col) * 4 + 3] = 0.0f;
        for (size_t i = 2; i < curtain.positions.size(); i += 4) curtain.positions[i] = 0.0f;
        scenes::finishDemoScene(curtain);  // the rest positions and normals of the flat sheet
        ClothShape shapes[2][3];
        settleCloth(env, d3d, curtain, 8.0f, shapes);
        float spread[2][3];
        for (int f = 0; f < 2; ++f) {
            const ClothShape* s = shapes[f];
            spread[f][0] = relativeSpread(s[0].drop, s[1].drop, s[2].drop);
            spread[f][1] = relativeSpread(s[0].meanDepth, s[1].meanDepth, s[2].meanDepth);
            spread[f][2] = relativeSpread(s[0].hemDepth, s[1].hemDepth, s[2].hemDepth);
            std::printf("    %s spread over 8/16/32 substeps: drop %.3f%%, mean depth %.3f%%, hem depth %.3f%%; stretch "
                        "of the drop %.4f / %.4f / %.4f m\n",
                        f == 0 ? "XPBD" : "PBD ", double(spread[f][0]) * 100.0, double(spread[f][1]) * 100.0,
                        double(spread[f][2]) * 100.0, double(s[0].drop - restHeight), double(s[1].drop - restHeight),
                        double(s[2].drop - restHeight));
        }
        // XPBD: the same sheet at every substep count, within 0.1%, at rest.
        for (int m = 0; m < 3; ++m) CHECK(spread[0][m] < 1.0e-3f);
        for (int k = 0; k < 3; ++k) CHECK(shapes[0][k].speed < 0.05f);
        // PBD: stiffer with every substep, by far more than XPBD spreads: the stretch of the sheet shrinks.
        const ClothShape* p = shapes[1];
        CHECK(p[0].drop > p[1].drop && p[1].drop > p[2].drop);
        CHECK(p[0].meanDepth > p[1].meanDepth && p[1].meanDepth > p[2].meanDepth);
        CHECK(spread[1][0] > 10.0f * spread[0][0] && spread[1][0] > 0.01f);
        CHECK(p[0].drop - restHeight > 4.0f * (p[2].drop - restHeight));
    }
    {
        std::printf("  the reference's hanging (two corners pinned, rippled, still air), 10 s\n");
        ClothShape shapes[2][3];
        settleCloth(env, d3d, scenes::xpbdCloth(false, 4.0f), 10.0f, shapes);
        float spread[2][2];
        for (int f = 0; f < 2; ++f) {
            const ClothShape* s = shapes[f];
            spread[f][0] = relativeSpread(s[0].topRow, s[1].topRow, s[2].topRow);
            spread[f][1] = relativeSpread(s[0].drop, s[1].drop, s[2].drop);
            std::printf("    %s spread over 8/16/32 substeps: top row %.3f%%, drop %.3f%%, dip/D %.3f%%, mean depth %.3f%%\n",
                        f == 0 ? "XPBD" : "PBD ", double(spread[f][0]) * 100.0, double(spread[f][1]) * 100.0,
                        double(relativeSpread(s[0].dip, s[1].dip, s[2].dip)) * 100.0,
                        double(relativeSpread(s[0].meanDepth, s[1].meanDepth, s[2].meanDepth)) * 100.0);
        }
        CHECK(spread[0][0] < 5.0e-3f && spread[0][1] < 5.0e-3f);
        const ClothShape* p = shapes[1];
        CHECK(p[0].topRow > p[1].topRow && p[1].topRow > p[2].topRow);
        CHECK(p[0].drop > p[1].drop && p[1].drop > p[2].drop);
        CHECK(spread[1][0] > 0.03f && spread[1][1] > 0.03f);
    }
}

// ---- Compliance -------------------------------------------------------------------------------------------------

void testCompliance(const Env& env, D3DMessages& d3d) {
    std::printf("compliance\n");
    const int links = 10;
    const float expectedSum = 9.8f * float(links * (links + 1) / 2);  // m g sum_j j, unit masses
    struct Case {
        float coefficient;
        float stiffnessMin;
    };
    // Rising stiffness: by the coefficient on the default range, then by the range at a fixed coefficient.
    // At 8 substeps the stiffest (4.4e5 N/m) has k dt^2 / m = 1.9, where 50 Jacobi iterations converge on
    // ten links; at 2 substeps the stiff cases are not converged and come out softer (the documented limit
    // of the formulation on this solver: XPBD is independent of the counts only once converged).
    const Case cases[] = {{0.1f, 1.0e3f}, {0.2f, 1.0e3f}, {0.3f, 1.0e3f}, {0.4f, 1.0e3f}, {0.3f, 4.0e3f}, {0.3f, 1.6e4f}};
    float stretches[6] = {};
    for (int c = 0; c < 6; ++c) {
        scenes::Scene scene = hangingChain(cases[c].coefficient, links, 8);
        scene.compliance.stiffnessMin = cases[c].stiffnessMin;
        const SceneState state = runScene(env, d3d, scene, 180);
        if (state.positions.empty()) return;
        const float length = 2.5f - state.positions[size_t(links) * 4 + 1];
        stretches[c] = length - 0.1f * float(links);
        const float stiffness = cases[c].stiffnessMin * std::pow(1.0e9f / cases[c].stiffnessMin, cases[c].coefficient);
        const float expected = expectedSum / stiffness;
        std::printf("  k %.1f on [%.0e, 1e9]: %.4e N/m, stretch %.5f m, alpha m g sum predicts %.5f m (%+.2f%%)\n",
                    double(cases[c].coefficient), double(cases[c].stiffnessMin), double(stiffness), double(stretches[c]),
                    double(expected), double((stretches[c] / expected - 1.0f) * 100.0f));
        CHECK(std::fabs(stretches[c] / expected - 1.0f) < 0.02f);
    }
    for (int c = 1; c < 4; ++c) CHECK(stretches[c] < stretches[c - 1]);
    CHECK(stretches[4] < stretches[2] && stretches[5] < stretches[4]);

    // The volume: 0 keeps the rest volume; a growing compliance gives way more and more.
    const float volumeCompliances[3] = {0.0f, 1.0e-4f, 1.0e-3f};
    float worstLoss[3] = {};
    for (int c = 0; c < 3; ++c) {
        scenes::Scene scene = scenes::xpbdBalloon();
        scene.compliance.volumeCompliance = volumeCompliances[c];
        const SceneState state = runScene(env, d3d, scene, 120);
        if (state.positions.empty()) return;
        std::vector<float> volumes, heights;
        balloonShapes(scene, state.positions, volumes, heights);
        for (float v : volumes) worstLoss[c] = std::max(worstLoss[c], 1.0f - v);
        std::printf("  volume compliance %.0e: volume / rest %.4f %.4f %.4f %.4f\n", double(volumeCompliances[c]),
                    double(volumes[0]), double(volumes[1]), double(volumes[2]), double(volumes[3]));
    }
    CHECK(std::fabs(worstLoss[0]) < 0.01f);
    CHECK(worstLoss[1] > worstLoss[0] && worstLoss[2] > worstLoss[1]);
}

// ---- Determinism ------------------------------------------------------------------------------------------------

void testDeterminism(const Env& env, D3DMessages& d3d) {
    std::printf("determinism\n");
    for (const scenes::Scene& scene : scenes::xpbdAll()) {
        RigOptions options;
        options.baselineReductions = true;  // the summation order of FleX
        const SceneState a = runScene(env, d3d, scene, 10, options);
        const SceneState b = runScene(env, d3d, scene, 10, options);
        const float dp = maxAbsDifference(a.positions, b.positions);
        const float dv = maxAbsDifference(a.velocities, b.velocities);
        const bool identical = !a.positions.empty() && a.positions.size() == b.positions.size() &&
                               std::memcmp(a.positions.data(), b.positions.data(), a.positions.size() * 4) == 0 &&
                               std::memcmp(a.velocities.data(), b.velocities.data(), a.velocities.size() * 4) == 0;
        std::printf("  %-16s two runs of 10 steps: %s (max difference %g in positions, %g in velocities)\n",
                    scene.name.c_str(), identical ? "bit-identical" : "NOT bit-identical", double(dp), double(dv));
        // The kernels that add with a compare-exchange loop (UpdateTriangles, UpdateVertexNormals, the
        // inflatable volume) add in the order the threads arrive: the scenes with triangles repeat only up
        // to rounding. The chain has none.
        if (scene.numTriangles() == 0 && scene.inflatables.empty() && scene.numRigids() == 0) {
            CHECK(identical);
        } else {
            CHECK(dp < 1.0e-3f);
            CHECK(dv < 5.0e-2f);
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);  // nothing is lost when the process dies
    Options options;
    parse(argc, argv, options);
    if (!options.rest.empty()) {
        std::printf("usage: %s <d3d11|d3d12|vulkan> [--adapter <n>] [--shaders <dir>]\n", argv[0]);
        return 1;
    }
    return runWithDevice(options, [&](Env& env, D3DMessages& d3d) {
        testInterface(env, d3d);
        testMultipliers(env, d3d);
        testClearedEverySubstep(env, d3d);
        std::printf("the reference scenes\n");
        testChain(env, d3d);
        testBeam(env, d3d);
        testCloth(env, d3d);
        testBalloon(env, d3d);
        testCompliance(env, d3d);
        testDeterminism(env, d3d);
        testSubstepConvergence(env, d3d);
    });
}
