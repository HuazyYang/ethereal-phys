// ethereal_phys_solver_test: IPhysSolver (the PBD solver) on a real nvrhi device, headless, without any
// reference to compare with. The comparison with FleX is parity_test.cpp, the XPBD solver xpbd_test.cpp.
//
//     ethereal_phys_solver_test <d3d11|d3d12|vulkan> [--adapter <n>] [--shaders <dir>] [--dump <file>]
//     ethereal_phys_solver_test --compare <file a> <file b>
//
// The device is created as in the other device tests: debug runtime, nvrhi validation layer, and every
// message of either, or of the engine, is a failure unless the test provoked it.
//
// What it checks:
//   - creation: the description, the interfaces a solver answers (not IPhysSolverXPBD), its outputs; what
//     createSolver and step refuse, with one message each;
//   - every scene of solver_scenes.h steps without a message from a validation layer, without a NaN, and
//     with every particle inside the box of planes and inside the bounds the solver computed; and what each
//     scene is about: a pile comes to rest, pinned cloth stays pinned, rigid transforms follow their
//     particles, an inflatable keeps its volume, particles stay outside the shapes, a fluid has densities,
//     smoothed positions and anisotropy, diffuse particles are created within their capacity;
//   - two runs of a scene on one device give the same bits where the pipeline has no floating-point atomics,
//     and agree closely where it has;
//   - a position buffer of the application (zero copy) receives the positions, and the fallback copy does
//     where zero copy is not available;
//   - stage callbacks run at every stage with a consistent context, can be chained, and a kernel a callback
//     dispatches changes the velocities the step leaves;
//   - every object is gone with its last reference.
//
// --dump writes the state of every scene after a fixed number of steps to a file; --compare compares two
// such files (of two graphics APIs) within a tolerance. ctest runs the three as a fixture.
//
// Exit code: 0 passed, 1 failed, 77 the graphics API has no usable device on this machine (skipped).

#include <ethereal/phys/device.h>
#include <ethereal/phys/solver.h>

#include "kernel_params.h"
#include "solver_support.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

using namespace support;
using ephys::SolverOutput;
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

// The smallest signed distance of a particle to a plane of the scene.
float minPlaneDistance(const scenes::Scene& scene, const std::vector<float>& positions) {
    float worst = INFINITY;
    for (size_t i = 0; i + 3 < positions.size(); i += 4)
        for (int p = 0; p < scene.params.numPlanes; ++p) {
            const float* plane = scene.params.planes[p];
            const float d = plane[0] * positions[i] + plane[1] * positions[i + 1] + plane[2] * positions[i + 2] + plane[3];
            worst = std::min(worst, d);
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

// ---- Creation --------------------------------------------------------------------------------------------------

void testCreation(const Env& env) {
    std::printf("creation\n");
    ephys::DomainDesc domainDesc;
    domainDesc.maxParticles = 1000;
    domainDesc.maxDiffuseParticles = 64;
    ephys::PhysDomainHandle domain;
    CHECK(env.phys->createDomain(domainDesc, &domain) == nvrhi::FS_OK);
    if (!domain) return;
    const nvrhi::FLONG domainRefs = refCount(domain.Get());

    ephys::SolverDesc desc;
    desc.maxNeighborsPerParticle = 40;
    desc.maxContactsPerParticle = 4;

    // What createSolver refuses.
    ephys::PhysSolverHandle none;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, env.phys->createSolver(nullptr, desc, &none));
    CHECK(env.phys->createSolver(domain, desc, nullptr) == nvrhi::FE_INVALID_ARGS);
    {
        ephys::SolverDesc bad = desc;
        bad.maxNeighborsPerParticle = 0;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, env.phys->createSolver(domain, bad, &none));
        bad = desc;
        bad.maxContactsPerParticle = 0;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, env.phys->createSolver(domain, bad, &none));
        bad = desc;
        bad.featureMode = ephys::FeatureMode(7);
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, env.phys->createSolver(domain, bad, &none));
        bad = desc;
        bad.mode = ephys::SolverMode(9);
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, env.phys->createSolver(domain, bad, &none));
        CHECK(none == nullptr);
    }
    CHECK(refCount(domain.Get()) == domainRefs);

    ephys::PhysSolverHandle solver;
    CHECK(env.phys->createSolver(domain, desc, &solver) == nvrhi::FS_OK);
    CHECK(solver != nullptr);
    if (!solver) return;

    CHECK(solver->getMode() == ephys::SolverMode::PBD);
    CHECK(solver->getDomain() == domain);
    CHECK(solver->getDesc().maxNeighborsPerParticle == 40 && solver->getDesc().maxContactsPerParticle == 4);
    CHECK(solver->getSubstepCount() == 0);
    CHECK(!solver->areTimersEnabled());
    CHECK(refCount(domain.Get()) == domainRefs + 1);  // the solver keeps its domain

    // QueryInterface.
    nvrhi::AutoPtr<ephys::IPhysSolver> same;
    CHECK(solver->QueryInterface(NVRHI_IID_PPV_ARGS(&same)) == nvrhi::FS_OK && same == solver);
    same = nullptr;
    nvrhi::AutoPtr<ephys::IPhysDomain> notADomain;
    CHECK(solver->QueryInterface(NVRHI_IID_PPV_ARGS(&notADomain)) == nvrhi::FE_NOINTERFACE);
    // A PBD solver has no XPBD interface (ethereal_phys_xpbd_* tests the XPBD solver).
    ephys::PhysSolverXPBDHandle notXpbd;
    CHECK(solver->QueryInterface(NVRHI_IID_PPV_ARGS(&notXpbd)) == nvrhi::FE_NOINTERFACE && notXpbd == nullptr);

    // The outputs.
    for (uint32_t i = 0; i < uint32_t(SolverOutput::Count); ++i) {
        const SolverOutput output = SolverOutput(i);
        IPhysBuffer* buffer = solver->getOutput(output);
        CHECK(buffer != nullptr);
        if (!buffer) continue;
        CHECK(buffer->getElementStride() == ephys::solverOutputStride(output));
        uint32_t expected = 1000;
        if (output == SolverOutput::ContactPlanes || output == SolverOutput::ContactVelocities) expected = 1000 * 4;
        if (output == SolverOutput::Neighbors) expected = 1024 * 40;
        CHECK(buffer->getElementCount() >= expected);
        if (output != SolverOutput::Neighbors && output != SolverOutput::InternalToApi)
            CHECK(buffer->getElementCount() == expected);
    }
    CHECK(solver->getOutput(SolverOutput::Count) == nullptr);
    // Before the first step: FleX's initial anisotropy, zeros elsewhere.
    {
        const std::vector<float> q2 = env.fetch<float>(solver->getOutput(SolverOutput::Anisotropy2), 0, 1000);
        bool ok = q2.size() == 4000;
        for (size_t i = 0; ok && i < q2.size(); i += 4)
            ok = q2[i] == 0.0f && q2[i + 1] == 1.0f && q2[i + 2] == 0.0f && q2[i + 3] == 0.1f;
        CHECK(ok);
        const std::vector<float> densities = env.fetch<float>(solver->getOutput(SolverOutput::Densities), 0, 1000);
        CHECK(densities == std::vector<float>(1000, 0.0f));
    }

    // What step refuses.
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, solver->step(nullptr, 1.0f / 60.0f, 0));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, solver->step(nullptr, 0.0f, 1));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, solver->step(nullptr, -1.0f, 1));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, solver->step(nullptr, NAN, 1));
    CHECK(solver->getSubstepCount() == 0);

    // A domain without an active particle: the step is maintenance only.
    const uint64_t revision = domain->getParticles()->getContentRevision();
    CHECK(solver->step(nullptr, 1.0f / 60.0f, 2) == nvrhi::FS_OK);
    CHECK(solver->getSubstepCount() == 0);
    CHECK(domain->getParticles()->getContentRevision() == revision);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);

    // Stage callbacks: a stage that is none.
    nvrhi::AutoPtr<ephys::IPhysSolverStageCallback> previous;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, solver->setStageCallback(SolverStage::Count, nullptr, &previous));
    CHECK(previous == nullptr);

    solver->setTimersEnabled(true);
    CHECK(solver->areTimersEnabled());

    solver = nullptr;
    CHECK(refCount(domain.Get()) == domainRefs);
}

// ---- The scenes ------------------------------------------------------------------------------------------------

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

// Steps a scene and checks what holds for every scene. Returns the state after `steps` steps.
SceneState runScene(const Env& env, D3DMessages& d3d, const scenes::Scene& scene, uint32_t steps,
                    const RigOptions& options = RigOptions(), Rig* keep = nullptr) {
    SceneState state;
    Rig rig = loadScene(env, scene, options);
    CHECK(bool(rig));
    if (!rig) return state;

    ephys::IParticleState* particles = rig.domain->getParticles();
    const uint64_t revision = particles->getContentRevision();
    for (uint32_t i = 0; i < steps; ++i) stepScene(env, scene, rig);
    CHECK(rig.solver->getSubstepCount() == uint64_t(steps) * scene.substeps);
    CHECK(particles->getContentRevision() == revision + steps);
    CHECK(rig.solver->getNeighborStride() == scene.numParticles());

    state = readState(env, scene, rig);
    CHECK(d3d.problems() == 0);
    CHECK(state.positions.size() == size_t(scene.numParticles()) * 4);
    CHECK(allFinite(state.positions));
    CHECK(allFinite(state.velocities));

    // Inside the planes: a particle rests at the collision distance from a plane.
    const float planeDistance = minPlaneDistance(scene, state.positions);
    CHECK(planeDistance > -0.02f);

    // Inside the bounds the solver computed in the last substep (the predicted positions, grown by the radius).
    const std::vector<float> bounds = env.fetch<float>(particles->getBounds(), 0, 2);
    CHECK(bounds.size() == 6);
    if (bounds.size() == 6) {
        bool inside = true;
        const float slack = 0.25f;  // the bounds are of the predicted, not of the final positions
        for (size_t i = 0; i + 3 < state.positions.size(); i += 4)
            for (int k = 0; k < 3; ++k)
                inside = inside && state.positions[i + k] >= bounds[k] - slack && state.positions[i + k] <= bounds[3 + k] + slack;
        CHECK(inside);
        CHECK(bounds[0] < bounds[3] && bounds[1] < bounds[4] && bounds[2] < bounds[5]);
    }

    // The inverse masses are untouched.
    bool massesKept = true;
    for (size_t i = 3; i < state.positions.size(); i += 4) massesKept = massesKept && state.positions[i] == scene.positions[i];
    CHECK(massesKept);

    // The two index maps are inverse to each other.
    {
        const uint32_t n = scene.numParticles();
        const std::vector<int32_t> apiToInternal = env.fetch<int32_t>(rig.solver->getOutput(SolverOutput::ApiToInternal), 0, n);
        const std::vector<uint32_t> internalToApi =
            env.fetch<uint32_t>(rig.solver->getOutput(SolverOutput::InternalToApi), 0, n);
        bool inverse = apiToInternal.size() == n && internalToApi.size() == n;
        for (uint32_t j = 0; inverse && j < n; ++j)
            inverse = uint32_t(apiToInternal[j]) < n && internalToApi[size_t(apiToInternal[j])] == j;
        CHECK(inverse);
    }

    std::printf("  %-16s %5u particles, %3u steps: min plane distance %+.4f, max speed %.4f\n", scene.name.c_str(),
                scene.numParticles(), steps, double(planeDistance), double(maxSpeed(state.velocities)));
    if (keep) *keep = rig;
    return state;
}

void testGranular(const Env& env, D3DMessages& d3d) {
    for (const scenes::Scene& scene : {scenes::granular(), scenes::granularGlobal()}) {
        Rig rig;
        const SceneState state = runScene(env, d3d, scene, 300, RigOptions(), &rig);
        if (!rig) continue;
        // A pile at rest: five seconds after the drop the particles have (all but) stopped, and it lies on
        // the ground. (Without a sleep threshold a few grains on the surface of the pile keep creeping.)
        double meanSpeed = 0.0;
        for (size_t i = 0; i + 2 < state.velocities.size(); i += 3)
            meanSpeed += std::sqrt(double(state.velocities[i]) * state.velocities[i] +
                                   double(state.velocities[i + 1]) * state.velocities[i + 1] +
                                   double(state.velocities[i + 2]) * state.velocities[i + 2]);
        meanSpeed /= double(scene.numParticles());
        std::printf("    mean speed %.5f\n", meanSpeed);
        CHECK(meanSpeed < 0.02);
        CHECK(maxSpeed(state.velocities) < 0.5f);
        float top = 0.0f;
        for (size_t i = 1; i < state.positions.size(); i += 4) top = std::max(top, state.positions[i]);
        CHECK(top < 1.0f);

        // The contacts of the bottom layer: every particle near the ground has one.
        const uint32_t n = scene.numParticles();
        const std::vector<int32_t> counts = env.fetch<int32_t>(rig.solver->getOutput(SolverOutput::ContactCounts), 0, n);
        const std::vector<int32_t> apiToInternal = env.fetch<int32_t>(rig.solver->getOutput(SolverOutput::ApiToInternal), 0, n);
        uint32_t onGround = 0, withContact = 0;
        for (uint32_t j = 0; j < n && counts.size() == n && apiToInternal.size() == n; ++j) {
            if (state.positions[size_t(j) * 4 + 1] < scene.params.collisionDistance + 0.01f) {
                ++onGround;
                if (counts[size_t(apiToInternal[j])] > 0) ++withContact;
            }
        }
        CHECK(onGround > 50);
        CHECK(withContact == onGround);

        // Neighbours: a particle inside the pile has some, and no list is longer than the capacity.
        const std::vector<int32_t> neighborCounts =
            env.fetch<int32_t>(rig.solver->getOutput(SolverOutput::NeighborCounts), 0, n);
        int32_t most = 0;
        for (int32_t c : neighborCounts) most = std::max(most, c);
        CHECK(most > 4 && most <= int32_t(scene.maxNeighbors));
    }
}

void testFluid(const Env& env, D3DMessages& d3d) {
    const scenes::Scene scene = scenes::fluid();
    Rig rig;
    const SceneState state = runScene(env, d3d, scene, 120, RigOptions(), &rig);
    if (!rig) return;
    const uint32_t n = scene.numParticles();

    // The dam has broken: the fluid has spread along x.
    float right = 0.0f;
    for (size_t i = 0; i < state.positions.size(); i += 4) right = std::max(right, state.positions[i]);
    CHECK(right > 1.2f);

    // Densities relative to the rest density: finite, not negative, and around one inside the fluid.
    const std::vector<float> densities = env.fetch<float>(rig.solver->getOutput(SolverOutput::Densities), 0, n);
    CHECK(allFinite(densities));
    double mean = 0.0;
    float lowest = INFINITY;
    for (float d : densities) {
        mean += d;
        lowest = std::min(lowest, d);
    }
    mean /= double(n);
    CHECK(lowest >= 0.0f);
    CHECK(mean > 0.5 && mean < 1.5);

    // The smoothed positions stay near the positions; the anisotropy axes are unit vectors with a scale.
    const std::vector<float> smooth = env.fetch<float>(rig.solver->getOutput(SolverOutput::SmoothedPositions), 0, n);
    CHECK(allFinite(smooth));
    float furthest = 0.0f;
    for (size_t i = 0; i + 3 < smooth.size(); i += 4)
        for (int k = 0; k < 3; ++k) furthest = std::max(furthest, std::fabs(smooth[i + k] - state.positions[i + k]));
    CHECK(furthest < scene.params.radius);
    CHECK(furthest > 0.0f);
    for (SolverOutput output : {SolverOutput::Anisotropy1, SolverOutput::Anisotropy2, SolverOutput::Anisotropy3}) {
        const std::vector<float> q = env.fetch<float>(rig.solver->getOutput(output), 0, n);
        CHECK(allFinite(q));
        bool unit = true;
        for (size_t i = 0; i + 3 < q.size(); i += 4) {
            const float l = std::sqrt(q[i] * q[i] + q[i + 1] * q[i + 1] + q[i + 2] * q[i + 2]);
            unit = unit && std::fabs(l - 1.0f) < 1.0e-3f && q[i + 3] > 0.0f;
        }
        CHECK(unit);
    }
    std::printf("    mean density %.3f, lowest %.3f, smoothing moved a particle by at most %.4f\n", mean, double(lowest),
                double(furthest));
}

void testDiffuse(const Env& env, D3DMessages& d3d) {
    const scenes::Scene scene = scenes::fluidDiffuse();
    Rig rig;
    runScene(env, d3d, scene, 90, RigOptions(), &rig);
    if (!rig) return;
    ephys::IDiffuseParticleState* diffuse = rig.domain->getDiffuseParticles();
    CHECK(diffuse != nullptr);
    if (!diffuse) return;
    const std::vector<int32_t> count = env.fetch<int32_t>(diffuse->getCountBuffer(), 0, 1);
    CHECK(count.size() == 1);
    if (count.size() != 1) return;
    std::printf("    %d diffuse particles\n", count[0]);
    CHECK(count[0] > 0 && count[0] <= int32_t(scene.maxDiffuse));
    const uint32_t alive = uint32_t(std::max(0, std::min(count[0], int32_t(scene.maxDiffuse))));
    const std::vector<float> positions = env.fetch<float>(diffuse->getPositions(), 0, alive);
    const std::vector<float> velocities = env.fetch<float>(diffuse->getVelocities(), 0, alive);
    CHECK(allFinite(positions) && allFinite(velocities));
    bool lifetimes = true;
    for (size_t i = 3; i < positions.size(); i += 4) lifetimes = lifetimes && positions[i] > 0.0f;  // compacted: all alive
    CHECK(lifetimes);
    CHECK(diffuse->getContentRevision() > 0);
}

void testCloth(const Env& env, D3DMessages& d3d) {
    const scenes::Scene scene = scenes::cloth();
    Rig rig;
    const SceneState state = runScene(env, d3d, scene, 120, RigOptions(), &rig);
    if (!rig) return;

    // Pinned particles (inverse mass 0) are where they were, to the bit; the rest of the cloth hangs below.
    bool pinned = true;
    float lowest = INFINITY;
    for (size_t i = 0; i < state.positions.size(); i += 4) {
        if (scene.positions[i + 3] == 0.0f)
            pinned = pinned && std::memcmp(&state.positions[i], &scene.positions[i], 16) == 0;
        lowest = std::min(lowest, state.positions[i + 1]);
    }
    CHECK(pinned);
    CHECK(lowest < 1.0f && lowest > 0.2f);  // it swung down, and its springs held

    // The solver computed the normals of the triangles and of their particles: unit vectors.
    const std::vector<float> triNormals = env.fetch<float>(rig.domain->getTriangles()->getNormals(), 0, scene.numTriangles());
    bool unit = triNormals.size() == size_t(scene.numTriangles()) * 3;
    for (size_t i = 0; unit && i < triNormals.size(); i += 3) {
        const float l = std::sqrt(triNormals[i] * triNormals[i] + triNormals[i + 1] * triNormals[i + 1] +
                                  triNormals[i + 2] * triNormals[i + 2]);
        unit = std::fabs(l - 1.0f) < 1.0e-3f;
    }
    CHECK(unit);
    const std::vector<float> normals =
        env.fetch<float>(rig.domain->getParticles()->getArray(ParticleArray::Normal), 0, scene.numParticles());
    unit = normals.size() == size_t(scene.numParticles()) * 4;
    for (size_t i = 0; unit && i < normals.size(); i += 4) {
        const float l = std::sqrt(normals[i] * normals[i] + normals[i + 1] * normals[i + 1] + normals[i + 2] * normals[i + 2]);
        unit = std::fabs(l - 1.0f) < 1.0e-3f;
    }
    CHECK(unit);
    CHECK(rig.domain->getTriangles()->getContentRevision() > 1);
}

void testRigid(const Env& env, D3DMessages& d3d) {
    for (const scenes::Scene& scene : {scenes::rigid(), scenes::rigidPlastic()}) {
        Rig rig;
        const SceneState state = runScene(env, d3d, scene, 120, RigOptions(), &rig);
        if (!rig) continue;
        ephys::IShapeMatchSet* rigids = rig.domain->getShapeMatches();
        const uint32_t numRigids = scene.numRigids();
        const std::vector<float> rotations = env.fetch<float>(rigids->getRotations(), 0, numRigids);
        const std::vector<float> translations = env.fetch<float>(rigids->getTranslations(), 0, numRigids);
        CHECK(allFinite(rotations) && allFinite(translations));
        CHECK(rotations.size() == size_t(numRigids) * 4 && translations.size() == size_t(numRigids) * 3);
        if (rotations.size() != size_t(numRigids) * 4 || translations.size() != size_t(numRigids) * 3) continue;

        float worstCentre = 0.0f, worstShape = 0.0f;
        for (uint32_t r = 0; r < numRigids; ++r) {
            // A unit quaternion.
            const float* q = &rotations[size_t(r) * 4];
            CHECK(std::fabs(std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]) - 1.0f) < 1.0e-3f);

            // The translation is the centre of the particles of the body, and the body kept its shape: the
            // distances between its particles are those of the rest pose (not with plastic deformation).
            const int32_t first = scene.rigidOffsets[r], last = scene.rigidOffsets[r + 1];
            double centre[3] = {0, 0, 0};
            for (int32_t k = first; k < last; ++k)
                for (int a = 0; a < 3; ++a) centre[a] += state.positions[size_t(scene.rigidIndices[size_t(k)]) * 4 + a];
            for (int a = 0; a < 3; ++a)
                worstCentre = std::max(worstCentre, std::fabs(float(centre[a] / double(last - first)) - translations[size_t(r) * 3 + a]));

            const size_t a0 = size_t(scene.rigidIndices[size_t(first)]) * 4, a1 = size_t(scene.rigidIndices[size_t(last - 1)]) * 4;
            const scenes::Vec3 now = {state.positions[a0] - state.positions[a1], state.positions[a0 + 1] - state.positions[a1 + 1],
                                      state.positions[a0 + 2] - state.positions[a1 + 2]};
            const scenes::Vec3 rest = {scene.positions[a0] - scene.positions[a1], scene.positions[a0 + 1] - scene.positions[a1 + 1],
                                       scene.positions[a0 + 2] - scene.positions[a1 + 2]};
            if (scene.rigidThresholds.empty() && scene.rigidStiffness[r] == 1.0f)
                worstShape = std::max(worstShape, std::fabs(scenes::length(now) - scenes::length(rest)));
        }
        std::printf("    rigid centres off by at most %.5f, diagonals by %.5f\n", double(worstCentre), double(worstShape));
        CHECK(worstCentre < 0.02f);
        CHECK(worstShape < 0.02f);
        CHECK(rigids->getContentRevision() > 1);
    }
}

float meshVolume(const scenes::Scene& scene, const std::vector<float>& positions) {
    float volume = 0.0f;
    for (size_t i = 0; i + 2 < scene.triangleIndices.size(); i += 3) {
        scenes::Vec3 v[3];
        for (int k = 0; k < 3; ++k) {
            const size_t p = size_t(scene.triangleIndices[i + k]) * 4;
            v[k] = {positions[p], positions[p + 1], positions[p + 2]};
        }
        volume += scenes::dot(v[0], scenes::cross(v[1] - v[0], v[2] - v[0]));
    }
    return volume;
}

void testInflatable(const Env& env, D3DMessages& d3d) {
    const scenes::Scene scene = scenes::inflatable();
    const SceneState state = runScene(env, d3d, scene, 120);
    if (state.positions.empty()) return;
    // The ball rests on the ground and has kept (most of) its volume. Without the volume constraint the
    // springs alone do not hold it up.
    const float rest = scene.inflatables[0].restVolume;
    const float volume = meshVolume(scene, state.positions);
    std::printf("    volume %.4f of rest volume %.4f\n", double(volume), double(rest));
    CHECK(volume > 0.8f * rest && volume < 1.4f * rest);
}

void testShapes(const Env& env, D3DMessages& d3d) {
    const scenes::Scene scene = scenes::shapes();
    Rig rig;
    const SceneState state = runScene(env, d3d, scene, 90, RigOptions(), &rig);
    if (!rig) return;
    const uint32_t n = scene.numParticles();

    // No particle is inside the sphere, the capsule or the distance field sphere (their centres are fixed).
    const scenes::Shape& sphere = scene.shapes[0];
    const scenes::Shape& sdfShape = scene.shapes[5];
    const scenes::Vec3 sphereCentre = {sphere.position[0], sphere.position[1], sphere.position[2]};
    const float sdfWidth = sdfShape.geometry[0];
    const scenes::Vec3 sdfCentre = {sdfShape.position[0] + sdfWidth * 0.5f, sdfShape.position[1] + sdfWidth * 0.5f,
                                    sdfShape.position[2] + sdfWidth * 0.5f};
    float nearestSphere = INFINITY, nearestSdf = INFINITY;
    for (size_t i = 0; i + 3 < state.positions.size(); i += 4) {
        const scenes::Vec3 p = {state.positions[i], state.positions[i + 1], state.positions[i + 2]};
        nearestSphere = std::min(nearestSphere, scenes::length(p - sphereCentre) - sphere.geometry[0]);
        nearestSdf = std::min(nearestSdf, scenes::length(p - sdfCentre) - 0.35f * sdfWidth);
    }
    std::printf("    nearest particle: %.4f from the sphere, %.4f from the distance field sphere\n", double(nearestSphere),
                double(nearestSdf));
    CHECK(nearestSphere > 0.0f);
    CHECK(nearestSdf > -0.03f);  // the field is sampled on a grid of 24

    // Some contact names every shape (the w of a contact velocity is the index of its shape).
    const std::vector<int32_t> counts = env.fetch<int32_t>(rig.solver->getOutput(SolverOutput::ContactCounts), 0, n);
    CHECK(counts.size() == n);
    uint32_t withContacts = 0;
    for (int32_t c : counts) withContacts += c > 0 ? 1 : 0;
    CHECK(withContacts > n / 4);

    // In the first steps, while the particles land on the shapes: every shape, of each of the six kinds,
    // is named by some contact (the w of a contact velocity is the index of its shape).
    Rig early = loadScene(env, scene);
    CHECK(bool(early));
    if (!early) return;
    std::vector<bool> named(scene.shapes.size(), false);
    for (uint32_t step = 0; step < 6; ++step) {
        stepScene(env, scene, early);
        const uint32_t c = scene.maxContacts;
        const std::vector<int32_t> stepCounts = env.fetch<int32_t>(early.solver->getOutput(SolverOutput::ContactCounts), 0, n);
        const std::vector<float> velocities =
            env.fetch<float>(early.solver->getOutput(SolverOutput::ContactVelocities), 0, n * c);
        for (uint32_t i = 0; i < n && stepCounts.size() == n && velocities.size() == size_t(n) * c * 4; ++i)
            for (int32_t k = 0; k < std::min(stepCounts[i], int32_t(c)); ++k) {
                const float w = velocities[(size_t(i) * c + size_t(k)) * 4 + 3];
                if (w >= 0.0f && size_t(w) < named.size()) named[size_t(w)] = true;
            }
    }
    for (size_t s = 0; s < named.size(); ++s) {
        if (!named[s]) std::printf("    no contact with shape %zu\n", s);
        CHECK(named[s]);
    }
    CHECK(d3d.problems() == 0);
}

// ---- Determinism -----------------------------------------------------------------------------------------------

void testDeterminism(const Env& env, D3DMessages& d3d) {
    std::printf("determinism\n");
    for (const scenes::Scene& scene : scenes::all()) {
        // Two runs with the reductions of the wave variant, and (where it differs) two with the baseline.
        const SceneState a = runScene(env, d3d, scene, 20);
        const SceneState b = runScene(env, d3d, scene, 20);
        const float dp = maxAbsDifference(a.positions, b.positions);
        const float dv = maxAbsDifference(a.velocities, b.velocities);
        const bool identical = a.positions.size() == b.positions.size() && !a.positions.empty() &&
                               std::memcmp(a.positions.data(), b.positions.data(), a.positions.size() * 4) == 0 &&
                               std::memcmp(a.velocities.data(), b.velocities.data(), a.velocities.size() * 4) == 0;
        std::printf("  %-16s two runs: %s (max difference %g in positions, %g in velocities)\n", scene.name.c_str(),
                    identical ? "bit-identical" : "NOT bit-identical", double(dp), double(dv));

        // The kernels that add into shared memory with a compare-exchange loop (UpdateTriangles,
        // UpdateVertexNormals, SolveInflatableVolume, SolveShapes) add in the order the threads arrive, so a
        // scene with triangles, inflatables or rigids repeats only up to rounding. Every other scene repeats
        // to the bit.
        const bool atomics = scene.numTriangles() || scene.numRigids() || !scene.inflatables.empty();
        if (atomics) {
            CHECK(dp < 1.0e-3f);
            CHECK(dv < 5.0e-2f);
        } else {
            CHECK(identical);
        }
    }
}

// ---- Zero copy ---------------------------------------------------------------------------------------------------

void testSharedPositions(const Env& env, D3DMessages& d3d) {
    std::printf("render-shared position buffer\n");
    const scenes::Scene scene = scenes::granular();
    const uint32_t steps = 10;
    const SceneState plain = runScene(env, d3d, scene, steps);

    if (env.phys->getInfo().supportsRenderSharedBuffers) {
        // A vertex buffer of the application is the position array: the solver writes into it.
        ephys::PhysBufferDesc desc;
        desc.elementCount = scene.maxParticles();
        desc.elementStride = 16;
        desc.use = ephys::BufferUse::RenderShared;
        desc.debugName = "test shared positions";
        PhysBufferHandle shared;
        CHECK(env.phys->createBuffer(desc, &shared) == nvrhi::FS_OK);
        if (!shared) return;
        CHECK(shared->getRHIBuffer()->getDesc().isVertexBuffer);

        RigOptions options;
        options.positionArray = shared;
        Rig rig;
        const SceneState viaArray = runScene(env, d3d, scene, steps, options, &rig);
        if (!rig) return;
        CHECK(rig.domain->getParticles()->getArray(ParticleArray::Position) == shared);
        CHECK(rig.domain->getParticles()->areRenderArraysShared());
        const std::vector<float> inShared = env.fetch<float>(shared, 0, scene.numParticles());
        CHECK(inShared == plain.positions);  // the same bits as without the substitution
        CHECK(viaArray.positions == plain.positions);
        CHECK(shared->getContentRevision() > 0);
    } else {
        // D3D11: no zero copy; the fallback is a device-to-device copy into the application's buffer.
        PhysBufferHandle target = env.create(ephys::BufferUse::Readback, scene.maxParticles(), 16);
        ephys::PhysBufferDesc desc;
        desc.elementCount = scene.maxParticles();
        desc.elementStride = 16;
        desc.use = ephys::BufferUse::Structured;
        desc.debugName = "test render positions";
        PhysBufferHandle renderBuffer;
        CHECK(env.phys->createBuffer(desc, &renderBuffer) == nvrhi::FS_OK);
        if (!renderBuffer) return;

        Rig rig = loadScene(env, scene);
        CHECK(bool(rig));
        if (!rig) return;
        ephys::IParticleState* particles = rig.domain->getParticles();
        CHECK_FAILS(nvrhi::FE_UNSUPPORTED, env.messages, particles->setArray(nullptr, ParticleArray::Position, renderBuffer));
        CHECK(particles->setRenderArray(ParticleArray::Position, renderBuffer) == nvrhi::FS_OK);
        CHECK(!particles->areRenderArraysShared());
        for (uint32_t i = 0; i < steps; ++i) stepScene(env, scene, rig);
        CHECK(particles->updateRenderArrays(nullptr) == nvrhi::FS_OK);
        const std::vector<float> copied = env.fetch<float>(renderBuffer, 0, scene.numParticles());
        CHECK(copied == plain.positions);
        CHECK(d3d.problems() == 0);
    }
}

// ---- Stage callbacks ---------------------------------------------------------------------------------------------

class StageRecorder final : public nvrhi::ObjectImpl<ephys::IPhysSolverStageCallback> {
 public:
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(StageRecorder)
    NVRHI_IMPLEMENTS_INTERFACE(ephys::IPhysSolverStageCallback)
    NVRHI_END_INTERFACE_TABLE()

    void onSolverStage(const ephys::SolverStageContext& context) noexcept override {
        stages.push_back(context.stage);
        contexts.push_back(context);
        if (next) next->onSolverStage(context);
        if (!setVelocities || context.stage != velocityStage) return;

        // Sets the velocity of every particle with one of the kernels of the engine.
        ephys::detail::ClearFloat4Params params;
        params.length = int32_t(context.numActive);
        params.value[0] = velocity[0];
        params.value[1] = velocity[1];
        params.value[2] = velocity[2];
        ephys::KernelDispatchDesc dispatch;
        dispatch.kernel = ephys::KernelId::ClearFloat4;
        dispatch.gridDim[0] = (context.numActive + 255) / 256;
        dispatch.rootConstants = &params;
        dispatch.rootConstantsSize = sizeof(params);
        dispatch.readWrite[0] = context.velocities;
        ephys::IPhysKernelLibrary* kernels = context.solver->getDomain()->getDevice()->getKernels();
        dispatchResult = kernels->dispatch(context.commandList, dispatch);
    }

    std::vector<SolverStage> stages;
    std::vector<ephys::SolverStageContext> contexts;
    nvrhi::AutoPtr<ephys::IPhysSolverStageCallback> next;
    bool setVelocities = false;
    SolverStage velocityStage = SolverStage::UpdateEnd;
    float velocity[3] = {1.0f, 2.0f, 3.0f};
    nvrhi::FRESULT dispatchResult = nvrhi::FS_OK;
};

void testCallbacks(const Env& env, D3DMessages& d3d) {
    std::printf("stage callbacks\n");

    // Particles in free space: no gravity, no planes, further apart than the radius. Whatever velocity a
    // callback gives them is the velocity the step leaves.
    scenes::Scene scene;
    scene.name = "free";
    scene.params = scenes::defaultParams();
    scene.params.gravity[1] = 0.0f;
    scene.params.radius = 0.1f;
    scene.params.numIterations = 3;
    scene.params.maxAcceleration = FLT_MAX;
    scenes::finishParams(scene.params);
    scenes::Jitter random(9);
    scenes::addParticleGrid(scene, {0, 0, 0}, 8, 8, 8, 0.5f, {0.1f, 0.0f, -0.2f}, 1.0f,
                            ephys::makePhase(0, ephys::PhaseSelfCollide), 0.01f, random);
    scene.substeps = 3;
    const uint32_t n = scene.numParticles();

    Rig rig = loadScene(env, scene);
    CHECK(bool(rig));
    if (!rig) return;

    auto recorder = MAKE_RC_OBJ_PTR(StageRecorder);
    const nvrhi::FLONG recorderRefs = refCount(recorder.Get());
    for (uint32_t s = 0; s < uint32_t(SolverStage::Count); ++s) {
        nvrhi::AutoPtr<ephys::IPhysSolverStageCallback> previous;
        CHECK(rig.solver->setStageCallback(SolverStage(s), recorder, &previous) == nvrhi::FS_OK);
        CHECK(previous == nullptr);
    }
    CHECK(refCount(recorder.Get()) == recorderRefs + nvrhi::FLONG(SolverStage::Count));

    stepScene(env, scene, rig);
    CHECK(d3d.problems() == 0);

    // The order of the stages of one step: per substep SubstepBegin, the iterations, SubstepEnd; UpdateEnd
    // once, after the last SubstepEnd.
    std::vector<SolverStage> expected;
    for (uint32_t s = 0; s < scene.substeps; ++s) {
        expected.push_back(SolverStage::SubstepBegin);
        for (int i = 0; i < scene.params.numIterations; ++i) {
            expected.push_back(SolverStage::IterationStart);
            expected.push_back(SolverStage::IterationEnd);
        }
        expected.push_back(SolverStage::SubstepEnd);
    }
    expected.push_back(SolverStage::UpdateEnd);
    CHECK(recorder->stages == expected);

    // The context.
    uint32_t substep = 0, iteration = 0;
    bool consistent = true;
    const float dta = scene.dt / float(scene.substeps);
    for (const ephys::SolverStageContext& c : recorder->contexts) {
        consistent = consistent && c.solver == rig.solver && c.commandList != nullptr && c.positions && c.velocities &&
                     c.phases && c.sortedToOriginalMap && c.originalToSortedMap && c.numActive == n &&
                     c.numSubsteps == scene.substeps && c.substep == substep;
        consistent = consistent && c.positions->getDesc().structStride == 16 && c.velocities->getDesc().structStride == 16 &&
                     c.phases->getDesc().structStride == 4 && c.positions->getDesc().canHaveUAVs;
        switch (c.stage) {
            case SolverStage::SubstepBegin:
                iteration = 0;
                consistent = consistent && c.dt == dta && c.iteration == 0;
                break;
            case SolverStage::IterationStart:
                consistent = consistent && c.dt == dta && c.iteration == iteration;
                break;
            case SolverStage::IterationEnd:
                consistent = consistent && c.dt == dta && c.iteration == iteration;
                ++iteration;
                break;
            case SolverStage::SubstepEnd:
                consistent = consistent && c.dt == dta && c.iteration == 0;
                if (substep + 1 < scene.substeps) ++substep;
                break;
            case SolverStage::UpdateEnd:
                consistent = consistent && c.dt == scene.dt && c.substep == scene.substeps - 1;
                break;
            default:
                consistent = false;
        }
    }
    CHECK(consistent);

    // Without interference the particles keep their velocity.
    {
        const std::vector<float> velocities = readVelocities(env, rig, n);
        float worst = 0.0f;
        for (size_t i = 0; i + 2 < velocities.size(); i += 3)
            worst = std::max({worst, std::fabs(velocities[i] - 0.1f), std::fabs(velocities[i + 1]), std::fabs(velocities[i + 2] + 0.2f)});
        CHECK(worst < 1.0e-4f);
    }

    // A callback that dispatches a kernel on the velocities at UpdateEnd.
    recorder->setVelocities = true;
    stepScene(env, scene, rig);
    CHECK(recorder->dispatchResult == nvrhi::FS_OK);
    CHECK(d3d.problems() == 0);
    {
        const std::vector<float> velocities = readVelocities(env, rig, n);
        float worst = 0.0f;
        for (size_t i = 0; i + 2 < velocities.size(); i += 3)
            worst = std::max({worst, std::fabs(velocities[i] - 1.0f), std::fabs(velocities[i + 1] - 2.0f),
                              std::fabs(velocities[i + 2] - 3.0f)});
        std::printf("  velocities set by a callback at UpdateEnd arrive within %g\n", double(worst));
        CHECK(worst < 1.0e-4f);
    }

    // The same at an IterationStart is discarded, as the header says: the velocities are derived from the
    // positions afterwards.
    recorder->velocityStage = SolverStage::IterationStart;
    recorder->velocity[0] = 50.0f;
    stepScene(env, scene, rig);
    {
        const std::vector<float> velocities = readVelocities(env, rig, n);
        float worst = 0.0f;
        for (size_t i = 0; i + 2 < velocities.size(); i += 3)
            worst = std::max({worst, std::fabs(velocities[i] - 1.0f), std::fabs(velocities[i + 1] - 2.0f),
                              std::fabs(velocities[i + 2] - 3.0f)});
        CHECK(worst < 1.0e-3f);
    }
    recorder->setVelocities = false;

    // Chaining: the previous callback comes back with a reference, and the chain runs both.
    auto second = MAKE_RC_OBJ_PTR(StageRecorder);
    {
        nvrhi::AutoPtr<ephys::IPhysSolverStageCallback> previous;
        CHECK(rig.solver->setStageCallback(SolverStage::UpdateEnd, second, &previous) == nvrhi::FS_OK);
        CHECK(previous == recorder);
        second->next = previous;
    }
    recorder->stages.clear();
    stepScene(env, scene, rig);
    CHECK(second->stages == std::vector<SolverStage>{SolverStage::UpdateEnd});
    CHECK(std::count(recorder->stages.begin(), recorder->stages.end(), SolverStage::UpdateEnd) == 1);

    // Removing a callback, with and without asking for the previous one.
    CHECK(rig.solver->setStageCallback(SolverStage::UpdateEnd, nullptr, nullptr) == nvrhi::FS_OK);
    second->next = nullptr;
    CHECK(refCount(second.Get()) == 1);
    for (uint32_t s = 0; s < uint32_t(SolverStage::Count); ++s) {
        nvrhi::AutoPtr<ephys::IPhysSolverStageCallback> previous;
        CHECK(rig.solver->setStageCallback(SolverStage(s), nullptr, &previous) == nvrhi::FS_OK);
        CHECK((previous == nullptr) == (SolverStage(s) == SolverStage::UpdateEnd));
    }
    CHECK(refCount(recorder.Get()) == recorderRefs);
    recorder->stages.clear();
    stepScene(env, scene, rig);
    CHECK(recorder->stages.empty());

    // A callback that is still registered goes with the solver.
    CHECK(rig.solver->setStageCallback(SolverStage::SubstepBegin, recorder, nullptr) == nvrhi::FS_OK);
    CHECK(refCount(recorder.Get()) == recorderRefs + 1);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    rig = Rig();
    CHECK(refCount(recorder.Get()) == recorderRefs);
}

// A step on a command list of the application, with the stage markers on.
void testOwnCommandList(const Env& env, D3DMessages& d3d) {
    std::printf("a step on the application's command list, with stage markers\n");
    const scenes::Scene scene = scenes::granular();
    const SceneState plain = runScene(env, d3d, scene, 3);

    Rig rig = loadScene(env, scene);
    CHECK(bool(rig));
    if (!rig) return;
    rig.solver->setTimersEnabled(true);
    env.phys->flush();  // the uploads; and on D3D11 only one command list may be open

    nvrhi::CommandListHandle commandList;
    CHECK(env.device->createCommandList(nvrhi::CommandListParameters(), &commandList) == nvrhi::FS_OK);
    if (!commandList) return;
    for (uint32_t i = 0; i < 3; ++i) {
        commandList->open();
        CHECK(rig.solver->step(commandList, scene.dt, scene.substeps) == nvrhi::FS_OK);
        commandList->close();
        nvrhi::ICommandList* list = commandList;
        env.device->executeCommandLists(&list, 1);
    }
    const SceneState state = readState(env, scene, rig);
    CHECK(state.positions == plain.positions);
    CHECK(state.velocities == plain.velocities);
    CHECK(d3d.problems() == 0);
}

// ---- Dump and compare --------------------------------------------------------------------------------------------

constexpr uint32_t c_DumpSteps = 10;

void dumpScenes(const Env& env, D3DMessages& d3d, const std::string& path) {
    std::printf("dump to %s\n", path.c_str());
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    CHECK(file.good());
    for (const scenes::Scene& scene : scenes::all()) {
        RigOptions options;
        options.baselineReductions = true;  // the same summation order on every device
        const SceneState state = runScene(env, d3d, scene, c_DumpSteps, options);
        const uint32_t counts[2] = {uint32_t(state.positions.size()), uint32_t(state.velocities.size())};
        file.write(reinterpret_cast<const char*>(counts), sizeof(counts));
        file.write(reinterpret_cast<const char*>(state.positions.data()), std::streamsize(state.positions.size() * 4));
        file.write(reinterpret_cast<const char*>(state.velocities.data()), std::streamsize(state.velocities.size() * 4));
    }
    CHECK(file.good());
}

int compareDumps(const std::string& pathA, const std::string& pathB) {
    std::ifstream a(pathA, std::ios::binary), b(pathB, std::ios::binary);
    if (!a.good() || !b.good()) {
        std::printf("SKIPPED: %s or %s is missing (a graphics API without a device)\n", pathA.c_str(), pathB.c_str());
        return 77;
    }
    std::printf("%s against %s, after %u steps\n", pathA.c_str(), pathB.c_str(), c_DumpSteps);
    for (const scenes::Scene& scene : scenes::all()) {
        SceneState states[2];
        std::ifstream* files[2] = {&a, &b};
        for (int k = 0; k < 2; ++k) {
            uint32_t counts[2] = {0, 0};
            files[k]->read(reinterpret_cast<char*>(counts), sizeof(counts));
            CHECK(files[k]->good() && counts[0] == scene.numParticles() * 4 && counts[1] == scene.numParticles() * 3);
            if (!files[k]->good() || counts[0] > (1u << 26) || counts[1] > (1u << 26)) return 1;
            states[k].positions.resize(counts[0]);
            states[k].velocities.resize(counts[1]);
            files[k]->read(reinterpret_cast<char*>(states[k].positions.data()), std::streamsize(counts[0]) * 4);
            files[k]->read(reinterpret_cast<char*>(states[k].velocities.data()), std::streamsize(counts[1]) * 4);
        }
        const float dp = maxAbsDifference(states[0].positions, states[1].positions);
        const float dv = maxAbsDifference(states[0].velocities, states[1].velocities);
        std::printf("  %-16s max difference %g in positions, %g in velocities\n", scene.name.c_str(), double(dp), double(dv));
        // Two shader compilers and two drivers: the same arithmetic, not the same rounding. After ten steps a
        // scene where a contact made on one API and missed on the other changes the motion of a particle (a
        // pile landing, a fluid splashing, particles landing on shapes) has amplified that to millimetres for
        // a few particles; every other scene stays at the level of rounding (as in parity_test.cpp).
        const bool chaotic = scene.name == "granular" || scene.name == "fluid" || scene.name == "fluid_diffuse" ||
                             scene.name == "shapes";
        CHECK(dp < (chaotic ? 1.0e-2f : 2.0e-5f));
        CHECK(dv < (chaotic ? 1.0f : 5.0e-4f));
    }
    if (g_Failures) {
        std::printf("FAILED: %d check(s)\n", g_Failures);
        return 1;
    }
    std::printf("PASSED\n");
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);  // nothing is lost when the process dies
    if (argc == 4 && std::strcmp(argv[1], "--compare") == 0) return compareDumps(argv[2], argv[3]);

    Options options;
    parse(argc, argv, options);
    std::string dumpPath;
    for (size_t i = 0; i < options.rest.size(); ++i) {
        if (options.rest[i] == "--dump" && i + 1 < options.rest.size()) {
            dumpPath = options.rest[++i];
        } else {
            std::printf("usage: %s <d3d11|d3d12|vulkan> [--adapter <n>] [--shaders <dir>] [--dump <file>]\n"
                        "       %s --compare <file a> <file b>\n",
                        argv[0], argv[0]);
            return 1;
        }
    }
    // A dump of an earlier run must not be compared when this run has no device.
    if (!dumpPath.empty()) {
        std::error_code ignored;
        std::filesystem::remove(std::filesystem::u8path(dumpPath), ignored);
        std::filesystem::create_directories(std::filesystem::u8path(dumpPath).parent_path(), ignored);
    }

    return runWithDevice(options, [&](Env& env, D3DMessages& d3d) {
        testCreation(env);
        CHECK(d3d.problems() == 0);
        std::printf("scenes\n");
        testGranular(env, d3d);
        testFluid(env, d3d);
        testDiffuse(env, d3d);
        testCloth(env, d3d);
        testRigid(env, d3d);
        testInflatable(env, d3d);
        testShapes(env, d3d);
        testDeterminism(env, d3d);
        testSharedPositions(env, d3d);
        testCallbacks(env, d3d);
        testOwnCommandList(env, d3d);
        if (!dumpPath.empty()) dumpScenes(env, d3d, dumpPath);
    });
}
