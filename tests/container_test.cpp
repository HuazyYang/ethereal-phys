// ethereal_phys_container_test: the container, instances, soft joints, force fields and moving frame of the
// extension layer (ext/container.h, ext/force_field.h, ext/moving_frame.h) on a real, headless device.
//
//     ethereal_phys_container_test <d3d11|d3d12|vulkan> [--shaders <dir>] [--adapter <n>]
//
// Without a reference (container_parity_test.cpp compares with FleX's extension library when one is configured):
//
// - the allocator under thousands of random allocations and frees hands out exactly what a model of FleX's free
//   list (a stack) predicts, its active list is the sorted complement, and bad frees are refused;
// - instances spawn at their transform with their velocity, phase and inverse mass scale, in the mirrors and on
//   the device, with their constraints compacted and remapped into the domain; destroying one frees its
//   particles and constraints; notifyAssetChanged recompacts; the refusals;
// - a soft joint over the touching faces of two cubes, moved up, drags both cubes after it;
// - a moving frame's state and apply against a hand computation;
// - each force mode, with and without falloff, changes the velocities of one step by the analytic amount; the
//   set's stage callback chains with the application's;
// - reference counts balance.
//
// Exit code: 0 passed, 1 failed, 77 no device for this graphics API.

#include <ethereal/phys/device.h>
#include <ethereal/phys/ext/asset_builder.h>
#include <ethereal/phys/ext/container.h>
#include <ethereal/phys/ext/force_field.h>
#include <ethereal/phys/ext/moving_frame.h>

#include "solver_support.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <set>
#include <vector>

namespace {

using namespace support;
using ephys::ContainerParticleData;
using ephys::ForceField;
using ephys::ForceMode;
using ephys::IPhysAsset;
using ephys::IPhysContainer;

// ---- Fixtures ----------------------------------------------------------------------------------------------

// Parameters without gravity, planes or clamps: particles float where they are put.
ephys::PhysParams floatingParams(float radius) {
    ephys::PhysParams p;
    p.radius = radius;
    p.numIterations = 4;
    p.relaxationFactor = 1.0f;
    scenes::finishParams(p);
    return p;
}

struct Fixture {
    ephys::PhysDomainHandle domain;
    ephys::PhysSolverHandle solver;
    ephys::PhysContainerHandle container;
    explicit operator bool() const { return domain && solver && container; }
};

Fixture makeFixture(Env& env, uint32_t maxParticles, const ephys::PhysParams& params, uint32_t containerMax = 0) {
    Fixture f;
    ephys::DomainDesc desc;
    desc.maxParticles = maxParticles;
    CHECK(env.phys->createDomain(desc, &f.domain) == nvrhi::FS_OK);
    if (!f.domain) return f;
    f.domain->setParams(params);
    ephys::SolverDesc solverDesc;
    CHECK(env.phys->createSolver(f.domain, solverDesc, &f.solver) == nvrhi::FS_OK);
    if (!f.solver) return f;
    CHECK(env.phys->createContainer(f.solver, containerMax ? containerMax : maxParticles, &f.container) ==
          nvrhi::FS_OK);
    env.phys->flush();
    return f;
}

void tick(Env& env, Fixture& f, float dt, uint32_t substeps) {
    CHECK(f.container->tickContainer(nullptr, dt, substeps) == nvrhi::FS_OK);
    env.phys->flush();
    env.device->runGarbageCollection();
}

// A closed box mesh: 8 vertices, 12 triangles, outward winding.
struct MeshData {
    std::vector<float> vertices;
    std::vector<int32_t> indices;
};

MeshData boxMesh(float h) {
    MeshData m;
    for (int i = 0; i < 8; ++i) m.vertices.insert(m.vertices.end(), {(i & 1) ? h : -h, (i & 2) ? h : -h, (i & 4) ? h : -h});
    const int32_t faces[12][3] = {{0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}, {0, 1, 4}, {1, 5, 4},
                                  {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6}, {1, 3, 5}, {3, 7, 5}};
    for (const auto& f : faces) m.indices.insert(m.indices.end(), {f[0], f[1], f[2]});
    return m;
}

// An n x n cloth in the x z plane centred at the origin, every particle of inverse mass 1.
void clothGrid(int n, float spacing, std::vector<float>& particles, std::vector<int32_t>& indices) {
    const float o = -0.5f * spacing * float(n - 1);
    for (int z = 0; z < n; ++z)
        for (int x = 0; x < n; ++x) particles.insert(particles.end(), {o + spacing * float(x), 0.0f, o + spacing * float(z), 1.0f});
    auto at = [n](int x, int z) { return int32_t(z * n + x); };
    for (int z = 0; z + 1 < n; ++z)
        for (int x = 0; x + 1 < n; ++x) {
            indices.insert(indices.end(), {at(x, z), at(x, z + 1), at(x + 1, z + 1)});
            indices.insert(indices.end(), {at(x, z), at(x + 1, z + 1), at(x + 1, z)});
        }
}

ephys::PhysAssetHandle rigidCube(ephys::IPhysAssetBuilder* builder, float half, float spacing) {
    const MeshData box = boxMesh(half);
    ephys::PhysAssetHandle asset;
    CHECK(builder->createRigidFromMesh(box.vertices.data(), 8, box.indices.data(), 36, spacing, 0.0f, &asset) ==
          nvrhi::FS_OK);
    return asset;
}

ephys::PhysAssetHandle clothSheet(ephys::IPhysAssetBuilder* builder, int n, float spacing) {
    std::vector<float> particles;
    std::vector<int32_t> indices;
    clothGrid(n, spacing, particles, indices);
    ephys::ClothAssetParams params;
    params.stretchStiffness = 0.8f;
    params.bendStiffness = 0.5f;
    ephys::PhysAssetHandle asset;
    CHECK(builder->createClothFromMesh(particles.data(), uint32_t(n * n), indices.data(), uint32_t(indices.size() / 3),
                                       params, &asset) == nvrhi::FS_OK);
    return asset;
}

// A rotation about y by `angle` followed by a translation, as FleX's column-major matrix for column vectors.
void rotationY(float angle, const float t[3], float m[16]) {
    const float c = std::cos(angle), s = std::sin(angle);
    const float matrix[16] = {c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0, t[0], t[1], t[2], 1};
    std::memcpy(m, matrix, sizeof(matrix));
}

void transformPoint(const float m[16], const float p[3], double out[3]) {
    for (int r = 0; r < 3; ++r)
        out[r] = double(m[r]) * p[0] + double(m[4 + r]) * p[1] + double(m[8 + r]) * p[2] + double(m[12 + r]);
}

// FleX's quaternion rotation (x y z imaginary, w real), in double.
void rotate(const float q[4], const double v[3], double out[3]) {
    const double u[3] = {q[0], q[1], q[2]}, w = q[3];
    const double c1[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
    const double t[3] = {c1[0] + w * v[0], c1[1] + w * v[1], c1[2] + w * v[2]};
    const double c2[3] = {u[1] * t[2] - u[2] * t[1], u[2] * t[0] - u[0] * t[2], u[0] * t[1] - u[1] * t[0]};
    for (int k = 0; k < 3; ++k) out[k] = v[k] + 2.0 * c2[k];
}

// ---- The allocator ---------------------------------------------------------------------------------------------

void testAllocator(Env& env) {
    std::printf("allocator\n");
    const uint32_t max = 500;
    Fixture f = makeFixture(env, max, floatingParams(0.1f));
    if (!f) return;
    IPhysContainer* c = f.container;

    // A model of FleX's free list: a stack that starts as 0 .. max - 1.
    std::vector<int32_t> model(max);
    for (uint32_t i = 0; i < max; ++i) model[i] = int32_t(i);
    std::vector<int32_t> allocated;
    uint32_t state = 12345u;
    auto rand = [&state]() { return (state = state * 1664525u + 1013904223u) >> 8; };

    uint32_t mismatches = 0, activeChecks = 0, activeMismatches = 0, allocations = 0, frees = 0;
    std::vector<int32_t> out(max), active(max);
    for (int op = 0; op < 4000; ++op) {
        if (rand() % 3 != 0) {
            const uint32_t n = rand() % 48;
            const uint32_t got = c->allocParticles(n, out.data());
            const uint32_t expected = std::min(n, uint32_t(model.size()));
            mismatches += got != expected;
            for (uint32_t i = 0; i < std::min(got, expected); ++i)
                mismatches += out[i] != model[model.size() - expected + i];
            model.resize(model.size() - expected);
            allocated.insert(allocated.end(), out.begin(), out.begin() + got);
            ++allocations;
        } else if (!allocated.empty()) {
            const uint32_t n = 1 + rand() % std::min<uint32_t>(12, uint32_t(allocated.size()));
            std::vector<int32_t> release;
            for (uint32_t i = 0; i < n; ++i) {
                const size_t k = rand() % allocated.size();
                release.push_back(allocated[k]);
                allocated.erase(allocated.begin() + std::ptrdiff_t(k));
            }
            CHECK(c->freeParticles(n, release.data()) == nvrhi::FS_OK);
            model.insert(model.end(), release.begin(), release.end());
            ++frees;
        }
        if (op % 97 == 0) {
            ++activeChecks;
            std::vector<int32_t> expected = allocated;
            std::sort(expected.begin(), expected.end());
            const uint32_t count = c->getActiveList(active.data(), max);
            activeMismatches += count != expected.size() || c->getActiveCount() != count ||
                                !std::equal(expected.begin(), expected.end(), active.begin());
        }
    }
    std::printf("  %u allocations, %u frees: %u index mismatches; %u active-list checks, %u mismatches\n", allocations,
                frees, mismatches, activeChecks, activeMismatches);
    CHECK(mismatches == 0 && activeMismatches == 0);

    // Refusals: nothing changes.
    const uint32_t before = c->getActiveCount();
    const int32_t outOfRange[] = {int32_t(max)};
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, c->freeParticles(1, outOfRange));
    if (!model.empty()) {
        const int32_t alreadyFree[] = {model.back()};
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, c->freeParticles(1, alreadyFree));
    }
    if (allocated.size() >= 2) {
        const int32_t twice[] = {allocated[0], allocated[1], allocated[0]};
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, c->freeParticles(3, twice));
    }
    CHECK(c->getActiveCount() == before);
    if (!allocated.empty()) {
        // still allocated: a valid free goes through after the refusals
        CHECK(c->freeParticles(1, &allocated[0]) == nvrhi::FS_OK);
        CHECK(c->getActiveCount() == before - 1);
    }
    // getActiveList with less room than the count: the count, and as many as fit.
    CHECK(c->getActiveList(nullptr, 0) == c->getActiveCount());

    // The first push makes the active list the domain's.
    CHECK(c->pushToDevice(nullptr) == nvrhi::FS_OK);
    env.phys->flush();
    CHECK(f.domain->getParticles()->getActiveCount() == c->getActiveCount());
    const uint32_t n = c->getActiveList(active.data(), max);
    const std::vector<int32_t> onDevice =
        env.fetch<int32_t>(f.domain->getParticles()->getArray(ParticleArray::ActiveIndex), 0, n);
    CHECK(n == 0 || std::equal(onDevice.begin(), onDevice.end(), active.begin()));
}

// ---- Instances -------------------------------------------------------------------------------------------------

void testInstances(Env& env, ephys::IPhysAssetBuilder* builder) {
    std::printf("instances\n");
    ephys::PhysAssetHandle rigid = rigidCube(builder, 0.2f, 0.1f);
    ephys::PhysAssetHandle cloth = clothSheet(builder, 6, 0.1f);
    if (!rigid || !cloth) return;
    const uint32_t rigidN = rigid->getParticleCount(), clothN = cloth->getParticleCount();
    std::printf("  rigid: %u particles, %u shape(s); cloth: %u particles, %u springs, %u triangles\n", rigidN,
                rigid->getShapeCount(), clothN, cloth->getSpringCount(), cloth->getTriangleCount());

    Fixture f = makeFixture(env, 1024, floatingParams(0.1f));
    if (!f) return;
    IPhysContainer* c = f.container;

    float rigidXform[16], clothXform[16];
    const float t1[3] = {1.0f, 2.0f, 3.0f}, t2[3] = {-1.0f, 1.5f, 0.5f};
    rotationY(0.5236f, t1, rigidXform);
    rotationY(-0.3f, t2, clothXform);
    const float rigidVelocity[3] = {0.5f, 0.0f, 0.0f}, clothVelocity[3] = {0.0f, 0.0f, -1.0f};
    const int32_t rigidPhase = ephys::makePhase(1, 0), clothPhase = ephys::makePhase(2, ephys::PhaseSelfCollide);

    // Without the mapped data, and with it (FleX's way).
    ephys::PhysInstanceHandle a, b;
    CHECK(c->createInstance(nullptr, rigid, rigidXform, rigidVelocity, rigidPhase, 0.5f, &a) == nvrhi::FS_OK);
    ContainerParticleData data;
    c->mapParticleData(data);
    CHECK(c->createInstance(&data, cloth, clothXform, clothVelocity, clothPhase, 1.0f, &b) == nvrhi::FS_OK);
    // A view that is not the container's mapped data.
    ContainerParticleData other = data;
    other.particles = nullptr;
    ephys::PhysInstanceHandle refused;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages,
                c->createInstance(&other, cloth, clothXform, clothVelocity, clothPhase, 1.0f, &refused));
    if (!a || !b) return;
    CHECK(a->getParticleCount() == rigidN && b->getParticleCount() == clothN);
    CHECK(c->getInstanceCount() == 2 && c->getActiveCount() == rigidN + clothN);
    CHECK(a->getAsset() == rigid.Get() && a->getContainer() == c);

    // The mirrors hold the spawned particles.
    auto checkSpawn = [&](ephys::IPhysInstance* inst, IPhysAsset* asset, const float* xform, const float* v,
                          int32_t phase, float scale) {
        double worst = 0.0;
        uint32_t wrong = 0;
        const float* p = asset->getParticles();
        for (uint32_t i = 0; i < inst->getParticleCount(); ++i) {
            const size_t k = size_t(inst->getParticleIndices()[i]);
            double expected[3];
            transformPoint(xform, &p[i * 4], expected);
            for (int j = 0; j < 3; ++j) worst = std::max(worst, std::fabs(expected[j] - data.particles[k * 4 + j]));
            wrong += data.particles[k * 4 + 3] != p[i * 4 + 3] * scale;
            wrong += std::memcmp(&data.restParticles[k * 4], &p[i * 4], 16) != 0;
            wrong += std::memcmp(&data.velocities[k * 3], v, 12) != 0;
            wrong += data.phases[k] != phase;
            for (int j = 0; j < 4; ++j) wrong += data.normals[k * 4 + j] != 0.0f;
        }
        return std::make_pair(worst, wrong);
    };
    const auto ra = checkSpawn(a, rigid, rigidXform, rigidVelocity, rigidPhase, 0.5f);
    const auto rb = checkSpawn(b, cloth, clothXform, clothVelocity, clothPhase, 1.0f);
    std::printf("  spawn: positions within %.2e / %.2e of the transform, %u / %u other values wrong\n", ra.first,
                rb.first, ra.second, rb.second);
    CHECK(ra.first < 1e-5 && rb.first < 1e-5 && ra.second == 0 && rb.second == 0);
    // The shape of the rigid: its centre through the transform, the rotation of the transform.
    {
        double centre[3], image[3];
        transformPoint(rigidXform, rigid->getShapeCenters(), centre);
        bool ok = true;
        for (int j = 0; j < 3; ++j) ok &= std::fabs(centre[j] - a->getShapeTranslations()[j]) < 1e-5;
        const double x[3] = {1, 0, 0};
        rotate(a->getShapeRotations(), x, image);
        for (int j = 0; j < 3; ++j) ok &= std::fabs(image[j] - rigidXform[j]) < 1e-5;
        CHECK(ok);
    }
    c->unmapParticleData();

    // Push: the device holds what the mirrors do; the constraints are compacted and remapped.
    CHECK(c->pushToDevice(nullptr) == nvrhi::FS_OK);
    env.phys->flush();
    CHECK(a->getShapeIndex() == 0 && b->getShapeIndex() == -1);
    CHECK(a->getTriangleIndex() == 0 && b->getTriangleIndex() == 0);
    ephys::IParticleState* particles = f.domain->getParticles();
    CHECK(particles->getActiveCount() == rigidN + clothN);
    CHECK(f.domain->getSprings()->getNumSprings() == cloth->getSpringCount());
    CHECK(f.domain->getShapeMatches()->getNumRigids() == rigid->getShapeCount());
    CHECK(f.domain->getShapeMatches()->getNumIndices() == rigid->getShapeIndexCount());
    CHECK(f.domain->getTriangles()->getNumTriangles() == cloth->getTriangleCount());
    {
        const uint32_t ns = cloth->getSpringCount();
        const std::vector<int32_t> springs = env.fetch<int32_t>(f.domain->getSprings()->getIndices(), 0, 2 * ns);
        uint32_t wrong = 0;
        for (uint32_t i = 0; i < 2 * ns && i < springs.size(); ++i)
            wrong += springs[i] != b->getParticleIndices()[cloth->getSpringIndices()[i]];
        const std::vector<int32_t> tris =
            env.fetch<int32_t>(f.domain->getTriangles()->getIndices(), 0, 3 * cloth->getTriangleCount());
        for (uint32_t i = 0; i < tris.size(); ++i) wrong += tris[i] != b->getParticleIndices()[cloth->getTriangleIndices()[i]];
        const std::vector<int32_t> shapeIndices =
            env.fetch<int32_t>(f.domain->getShapeMatches()->getIndices(), 0, rigid->getShapeIndexCount());
        for (uint32_t i = 0; i < shapeIndices.size(); ++i)
            wrong += shapeIndices[i] != a->getParticleIndices()[rigid->getShapeIndices()[i]];
        std::printf("  compaction: %u remapped indices wrong\n", wrong);
        CHECK(wrong == 0);
    }
    {
        ContainerParticleData view;
        c->mapParticleData(view);
        const std::vector<float> device = env.fetch<float>(particles->getArray(ParticleArray::Position), 0, 1024);
        uint32_t differ = 0;
        for (const ephys::IPhysInstance* inst : {a.Get(), b.Get()})
            for (uint32_t i = 0; i < inst->getParticleCount(); ++i) {
                const size_t k = size_t(inst->getParticleIndices()[i]);
                differ += std::memcmp(&device[k * 4], &view.particles[k * 4], 16) != 0;
            }
        CHECK(differ == 0);
        c->unmapParticleData();
    }

    // One tick without gravity: every particle moves by its velocity (the bodies are at rest in themselves).
    std::vector<float> before(size_t(1024) * 4);
    {
        ContainerParticleData view;
        c->mapParticleData(view);
        std::memcpy(before.data(), view.particles, before.size() * sizeof(float));
        c->unmapParticleData();
    }
    const float dt = 1.0f / 60.0f;
    tick(env, f, dt, 2);
    {
        ContainerParticleData view;
        c->mapParticleData(view);
        double worst = 0.0;
        for (const auto& [inst, v] : {std::make_pair(a.Get(), rigidVelocity), std::make_pair(b.Get(), clothVelocity)})
            for (uint32_t i = 0; i < inst->getParticleCount(); ++i) {
                const size_t k = size_t(inst->getParticleIndices()[i]);
                for (int j = 0; j < 3; ++j)
                    worst = std::max(worst, std::fabs(double(view.particles[k * 4 + j]) - (before[k * 4 + j] + v[j] * dt)));
            }
        std::printf("  one tick: particles within %.2e of their velocity times dt\n", worst);
        CHECK(worst < 1e-4);
        // The pulled bounds contain them.
        CHECK(view.lower[1] < 1.5f && view.upper[1] > 2.0f);
        c->unmapParticleData();
        // Shape data and triangle data are the compacted arrays.
        ephys::ContainerShapeData shapes;
        c->mapShapeData(shapes);
        CHECK(shapes.n == 1 && shapes.rotations && shapes.positions);
        if (shapes.positions) CHECK(std::fabs(shapes.positions[0] - (a->getShapeTranslations()[0] + rigidVelocity[0] * dt)) < 1e-4);
        c->unmapShapeData();
        ephys::ContainerTriangleData triangles;
        c->mapTriangleData(triangles);
        CHECK(triangles.numTriangles == cloth->getTriangleCount() && triangles.indices && triangles.normals);
        if (triangles.normals) {  // FleX never read them back; here they are the solver's: unit, along y
            const float* nrm = triangles.normals;
            CHECK(std::fabs(std::sqrt(nrm[0] * nrm[0] + nrm[1] * nrm[1] + nrm[2] * nrm[2]) - 1.0f) < 1e-3f);
            CHECK(std::fabs(nrm[1]) > 0.99f);
        }
        // A push that has to compact while the triangle data is mapped is refused.
        c->notifyAssetChanged(cloth);
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, c->pushToDevice(nullptr));
        c->unmapTriangleData();
        const uint64_t revision = f.domain->getTriangles()->getContentRevision();
        CHECK(c->pushToDevice(nullptr) == nvrhi::FS_OK);  // notifyAssetChanged: recompacted
        CHECK(f.domain->getTriangles()->getContentRevision() > revision);
        env.phys->flush();
    }
    // updateInstances copies the solver's transform of the rigid.
    c->updateInstances();
    CHECK(std::fabs(a->getShapeTranslations()[0] - (t1[0] + 0.0f) - rigidVelocity[0] * dt) < 0.05f);

    // Not enough room.
    {
        Fixture tiny = makeFixture(env, 64, floatingParams(0.1f), 10);
        ephys::PhysInstanceHandle none;
        if (tiny)
            CHECK_FAILS(nvrhi::FE_OUT_OF_MEMORY, env.messages,
                        tiny.container->createInstance(nullptr, rigid, rigidXform, rigidVelocity, 0, 1.0f, &none));
        CHECK(!none);
    }

    // Destroying the rigid instance frees its particles and its shape.
    std::vector<int32_t> rigidIndices(a->getParticleIndices(), a->getParticleIndices() + rigidN);
    a = nullptr;
    CHECK(c->getInstanceCount() == 1 && c->getActiveCount() == clothN);
    CHECK(c->pushToDevice(nullptr) == nvrhi::FS_OK);
    env.phys->flush();
    CHECK(f.domain->getShapeMatches()->getNumRigids() == 0);
    CHECK(f.domain->getSprings()->getNumSprings() == cloth->getSpringCount());
    CHECK(particles->getActiveCount() == clothN);
    std::vector<int32_t> active(1024);
    const uint32_t n = c->getActiveList(active.data(), 1024);
    uint32_t stillActive = 0;
    for (int32_t index : rigidIndices) stillActive += std::binary_search(active.begin(), active.begin() + n, index);
    CHECK(stillActive == 0);
    // The freed particles are handed out next (the end of the free list).
    std::vector<int32_t> again(rigidN);
    CHECK(c->allocParticles(rigidN, again.data()) == rigidN);
    std::sort(again.begin(), again.end());
    std::sort(rigidIndices.begin(), rigidIndices.end());
    CHECK(again == rigidIndices);
    CHECK(c->freeParticles(rigidN, again.data()) == nvrhi::FS_OK);

    b = nullptr;
    CHECK(c->getInstanceCount() == 0 && c->getActiveCount() == 0);
    CHECK(c->pushToDevice(nullptr) == nvrhi::FS_OK);
    CHECK(f.domain->getSprings()->getNumSprings() == 0 && f.domain->getTriangles()->getNumTriangles() == 0);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    env.device->runGarbageCollection();
    f.container = nullptr;
    f.solver = nullptr;
    CHECK(refCount(f.domain.Get()) == 1);  // the solver held it, and is gone with the container
    CHECK(refCount(rigid.Get()) == 1 && refCount(cloth.Get()) == 1);
}

// ---- Soft joints -----------------------------------------------------------------------------------------------

void testSoftJoint(Env& env, ephys::IPhysAssetBuilder* builder) {
    std::printf("soft joint\n");
    ephys::PhysAssetHandle cube = rigidCube(builder, 0.2f, 0.1f);
    if (!cube) return;
    Fixture f = makeFixture(env, 512, floatingParams(0.1f));
    if (!f) return;
    IPhysContainer* c = f.container;

    float left[16], right[16];
    const float tl[3] = {-0.22f, 1.0f, 0.0f}, tr[3] = {0.22f, 1.0f, 0.0f};
    rotationY(0.0f, tl, left);
    rotationY(0.0f, tr, right);
    const float zero[3] = {0, 0, 0};
    ephys::PhysInstanceHandle a, b;
    CHECK(c->createInstance(nullptr, cube, left, zero, ephys::makePhase(1, 0), 1.0f, &a) == nvrhi::FS_OK);
    CHECK(c->createInstance(nullptr, cube, right, zero, ephys::makePhase(2, 0), 1.0f, &b) == nvrhi::FS_OK);
    if (!a || !b) return;
    tick(env, f, 1.0f / 60.0f, 2);

    // The facing layers of the two cubes.
    ContainerParticleData data;
    c->mapParticleData(data);
    auto bodyCentre = [&](ephys::IPhysInstance* inst) {
        double y = 0;
        for (uint32_t i = 0; i < inst->getParticleCount(); ++i) y += data.particles[size_t(inst->getParticleIndices()[i]) * 4 + 1];
        return y / inst->getParticleCount();
    };
    float maxA = -FLT_MAX, minB = FLT_MAX;
    for (uint32_t i = 0; i < a->getParticleCount(); ++i) maxA = std::max(maxA, data.particles[size_t(a->getParticleIndices()[i]) * 4]);
    for (uint32_t i = 0; i < b->getParticleCount(); ++i) minB = std::min(minB, data.particles[size_t(b->getParticleIndices()[i]) * 4]);
    std::vector<int32_t> joint;
    for (uint32_t i = 0; i < a->getParticleCount(); ++i)
        if (data.particles[size_t(a->getParticleIndices()[i]) * 4] > maxA - 0.01f) joint.push_back(a->getParticleIndices()[i]);
    for (uint32_t i = 0; i < b->getParticleCount(); ++i)
        if (data.particles[size_t(b->getParticleIndices()[i]) * 4] < minB + 0.01f) joint.push_back(b->getParticleIndices()[i]);
    double centroid[3] = {0, 0, 0};
    for (int32_t k : joint)
        for (int j = 0; j < 3; ++j) centroid[j] += data.particles[size_t(k) * 4 + j] / double(joint.size());
    std::vector<float> local;
    for (int32_t k : joint)
        for (int j = 0; j < 3; ++j) local.push_back(float(data.particles[size_t(k) * 4 + j] - centroid[j]));
    const double startA = bodyCentre(a), startB = bodyCentre(b);
    c->unmapParticleData();

    ephys::PhysSoftJointHandle j;
    CHECK(c->createSoftJoint(joint.data(), local.data(), uint32_t(joint.size()), 1.0f, &j) == nvrhi::FS_OK);
    ephys::PhysSoftJointHandle bad;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, c->createSoftJoint(joint.data(), local.data(), 0, 1.0f, &bad));
    if (!j) return;
    CHECK(j->getShapeIndex() == -1 && !j->isInitialized() && c->getSoftJointCount() == 1);
    tick(env, f, 1.0f / 60.0f, 2);
    CHECK(j->getShapeIndex() == 2);  // after the shapes of the two cubes
    c->updateInstances();
    CHECK(j->isInitialized());
    const float* com = j->getShapeTranslation();
    std::printf("  joint of %u particles: centre (%.4f %.4f %.4f), expected (%.4f %.4f %.4f)\n", uint32_t(joint.size()),
                com[0], com[1], com[2], centroid[0], centroid[1], centroid[2]);
    CHECK(std::fabs(com[0] - centroid[0]) < 2e-3 && std::fabs(com[1] - centroid[1]) < 2e-3);

    // Move the joint up by 0.3: its particles move with it at once, in the mirrors.
    const float target[3] = {com[0], com[1] + 0.3f, com[2]};
    const float identity[4] = {0, 0, 0, 1};
    std::vector<float> jointBefore;
    c->mapParticleData(data);
    for (int32_t k : joint) jointBefore.push_back(data.particles[size_t(k) * 4 + 1]);
    c->unmapParticleData();
    CHECK(j->setTransform(target, identity) == nvrhi::FS_OK);
    CHECK(j->getShapeTranslation()[1] == target[1]);
    c->mapParticleData(data);
    double lift = 0;
    for (size_t i = 0; i < joint.size(); ++i) lift = std::max(lift, std::fabs(data.particles[size_t(joint[i]) * 4 + 1] - jointBefore[i] - 0.3));
    c->unmapParticleData();
    CHECK(lift < 1e-4);

    for (int frame = 0; frame < 20; ++frame) tick(env, f, 1.0f / 60.0f, 2);
    c->mapParticleData(data);
    const double endA = bodyCentre(a), endB = bodyCentre(b);
    double jointY = 0;
    for (int32_t k : joint) jointY += data.particles[size_t(k) * 4 + 1] / double(joint.size());
    c->unmapParticleData();
    std::printf("  moved up by 0.3: the cubes' centres rose by %.4f and %.4f, the joint's particles by %.4f\n",
                endA - startA, endB - startB, jointY - centroid[1]);
    CHECK(endA - startA > 0.03 && endB - startB > 0.03);  // dragged after the joint
    CHECK(jointY - centroid[1] > 0.03);

    j = nullptr;
    CHECK(c->getSoftJointCount() == 0);
    CHECK(c->pushToDevice(nullptr) == nvrhi::FS_OK);
    CHECK(f.domain->getShapeMatches()->getNumRigids() == 2);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    env.device->runGarbageCollection();
}

// ---- The moving frame --------------------------------------------------------------------------------------------

struct D3 {
    double x = 0, y = 0, z = 0;
};
D3 sub(D3 a, D3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
D3 add(D3 a, D3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
D3 mul(D3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
D3 cross(D3 a, D3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
struct DQ {
    double x = 0, y = 0, z = 0, w = 1;
};
DQ qmul(DQ a, DQ q) {  // FleX's product
    return {a.w * q.x + q.w * a.x + a.y * q.z - q.y * a.z, a.w * q.y + q.w * a.y + a.z * q.x - q.z * a.x,
            a.w * q.z + q.w * a.z + a.x * q.y - q.x * a.y, a.w * q.w - a.x * q.x - a.y * q.y - a.z * q.z};
}
DQ qy(double angle) { return {0, std::sin(angle / 2), 0, std::cos(angle / 2)}; }
D3 qrot(DQ q, D3 v) {
    const D3 u{q.x, q.y, q.z};
    return add(v, mul(cross(u, add(cross(u, v), mul(v, q.w))), 2.0));
}

void testMovingFrame(Env& env) {
    std::printf("moving frame\n");
    const D3 t0{0, 1, 0}, t1{0.1, 1, 0}, t2{0.25, 1.02, -0.01};
    const DQ q0 = qy(0.0), q1 = qy(0.05), q2 = qy(0.12);
    const double dt = 0.1;
    auto f3 = [](D3 v, float* out) { out[0] = float(v.x), out[1] = float(v.y), out[2] = float(v.z); };
    auto f4 = [](DQ q, float* out) { out[0] = float(q.x), out[1] = float(q.y), out[2] = float(q.z), out[3] = float(q.w); };
    float p[3], q[4];
    f3(t0, p), f4(q0, q);
    ephys::MovingFrameHandle frame;
    CHECK(env.phys->createMovingFrame(p, q, &frame) == nvrhi::FS_OK);
    if (!frame) return;
    CHECK(frame->getState().delta[0][0] == 1.0f && frame->getState().velocity[0] == 0.0f);
    f3(t1, p), f4(q1, q);
    CHECK(frame->update(p, q, float(dt)) == nvrhi::FS_OK);
    f3(t2, p), f4(q2, q);
    CHECK(frame->update(p, q, float(dt)) == nvrhi::FS_OK);
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, frame->update(p, q, 0.0f));

    // By hand: velocities and angular velocities of the two moves, and their differences.
    auto omega = [&](DQ to, DQ from) {
        const DQ d{to.x - from.x, to.y - from.y, to.z - from.z, to.w - from.w};
        const DQ r = qmul(d, DQ{-from.x, -from.y, -from.z, from.w});
        return D3{2 * r.x / dt, 2 * r.y / dt, 2 * r.z / dt};
    };
    const D3 v1 = mul(sub(t1, t0), 1 / dt), v2 = mul(sub(t2, t1), 1 / dt);
    const D3 w1 = omega(q1, q0), w2 = omega(q2, q1);
    const D3 a = mul(sub(v2, v1), 1 / dt), tau = mul(sub(w2, w1), 1 / dt);
    const ephys::MovingFrameState& s = frame->getState();
    double worstState = 0;
    auto cmp = [&](const float* got, D3 want) {
        const double scale = std::max(1.0, std::fabs(want.x) + std::fabs(want.y) + std::fabs(want.z));
        worstState = std::max({worstState, std::fabs(got[0] - want.x) / scale, std::fabs(got[1] - want.y) / scale,
                               std::fabs(got[2] - want.z) / scale});
    };
    cmp(s.velocity, v2), cmp(s.omega, w2), cmp(s.acceleration, a), cmp(s.tau, tau), cmp(s.position, t2);

    // apply: the particles go from the frame's previous pose to its current one; the velocities gain dt times the
    // scaled inertial accelerations at the particles' positions before the move.
    const float linearScale = 0.25f, angularScale = 0.5f;
    std::vector<float> positions = {0.3f, 1.2f, 0.1f, 1.0f, -0.2f, 0.9f, 0.4f, 0.5f, 0.05f, 1.5f, -0.3f, 2.0f};
    std::vector<float> velocities = {0.1f, 0.0f, 0.0f, 0.0f, -0.2f, 0.3f, 0.0f, 0.0f, 0.0f};
    const std::vector<float> p0 = positions, v0 = velocities;
    CHECK(frame->apply(positions.data(), velocities.data(), 3, linearScale, angularScale, float(dt)) == nvrhi::FS_OK);
    double worstApply = 0;
    for (int i = 0; i < 3; ++i) {
        const D3 x{p0[i * 4], p0[i * 4 + 1], p0[i * 4 + 2]};
        // previous pose: t1, q1; current: t2, q2
        const DQ q1inv{-q1.x, -q1.y, -q1.z, q1.w};
        const D3 moved = add(qrot(q2, qrot(q1inv, sub(x, t1))), t2);
        const D3 d = sub(x, t2);
        const D3 angular = mul(add(mul(cross(w2, cross(w2, d)), -1.0), mul(cross(tau, d), -1.0)), angularScale);
        const D3 linear = mul(a, -linearScale);
        const D3 v = add(D3{v0[i * 3], v0[i * 3 + 1], v0[i * 3 + 2]}, mul(add(linear, angular), dt));
        worstApply = std::max({worstApply, std::fabs(positions[i * 4] - moved.x), std::fabs(positions[i * 4 + 1] - moved.y),
                               std::fabs(positions[i * 4 + 2] - moved.z),
                               std::fabs(velocities[i * 3] - v.x) / std::max(1.0, std::fabs(v.x)),
                               std::fabs(velocities[i * 3 + 1] - v.y) / std::max(1.0, std::fabs(v.y)),
                               std::fabs(velocities[i * 3 + 2] - v.z) / std::max(1.0, std::fabs(v.z))});
        CHECK(positions[i * 4 + 3] == p0[i * 4 + 3]);  // the inverse mass is left alone
    }
    std::printf("  state within %.2e (relative), apply within %.2e of the hand computation\n", worstState, worstApply);
    CHECK(worstState < 1e-4 && worstApply < 1e-4);
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, frame->apply(nullptr, velocities.data(), 1, 1.0f, 1.0f, 0.1f));
}

// ---- Force fields ------------------------------------------------------------------------------------------------

// Counts its calls, so that the chaining of the force field set's callback is visible.
class CountingCallback final : public nvrhi::ObjectImpl<ephys::IPhysSolverStageCallback> {
 public:
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(CountingCallback)
    NVRHI_IMPLEMENTS_INTERFACE(ephys::IPhysSolverStageCallback)
    NVRHI_END_INTERFACE_TABLE()
    void onSolverStage(const ephys::SolverStageContext&) noexcept override { ++calls; }
    int calls = 0;
};

void testForceFields(Env& env) {
    std::printf("force fields\n");
    Fixture f = makeFixture(env, 64, floatingParams(0.05f));
    if (!f) return;
    IPhysContainer* c = f.container;

    // An application callback at UpdateEnd before the set: it stays called.
    auto counting = MAKE_RC_OBJ_PTR(CountingCallback);
    CHECK(f.solver->setStageCallback(ephys::SolverStage::UpdateEnd, counting, nullptr) == nvrhi::FS_OK);
    ephys::ForceFieldSetHandle set;
    CHECK(env.phys->createForceFieldSet(f.solver, &set) == nvrhi::FS_OK);
    if (!set) return;
    CHECK(set->getSolver() == f.solver.Get() && set->getForceFieldCount() == 0);

    // Five particles around the centre (0, 1, 0) of a field of radius 1: two inside with different masses, one
    // at the centre, one outside, one fixed inside.
    struct P {
        float pos[3];
        float invMass;
    };
    const P layout[] = {{{0.5f, 1.0f, 0.0f}, 0.5f},
                        {{0.0f, 1.25f, 0.0f}, 1.0f},
                        {{0.0f, 1.0f, 0.0f}, 1.0f},
                        {{1.5f, 1.0f, 0.0f}, 1.0f},
                        {{0.0f, 1.0f, -0.5f}, 0.0f}};
    const uint32_t n = 5;
    int32_t indices[n];
    CHECK(c->allocParticles(n, indices) == n);

    const float centre[3] = {0.0f, 1.0f, 0.0f}, radius = 1.0f, strength = 3.0f, dt = 0.05f;
    const ForceMode modes[] = {ForceMode::Force, ForceMode::Impulse, ForceMode::VelocityChange};
    const char* names[] = {"force", "impulse", "velocity change"};
    int run = 0;
    for (int m = 0; m < 3; ++m)
        for (int falloff = 0; falloff < 2; ++falloff, ++run) {
            ContainerParticleData data;
            c->mapParticleData(data);
            for (uint32_t i = 0; i < n; ++i) {
                const size_t k = size_t(indices[i]);
                std::memcpy(&data.particles[k * 4], layout[i].pos, 12);
                data.particles[k * 4 + 3] = layout[i].invMass;
                std::memset(&data.velocities[k * 3], 0, 12);
                data.phases[k] = ephys::makePhase(int32_t(i + 1), 0);
            }
            c->unmapParticleData();
            ForceField field;
            std::memcpy(field.position, centre, 12);
            field.radius = radius;
            field.strength = strength;
            field.mode = modes[m];
            field.linearFalloff = falloff != 0;
            CHECK(set->setForceFields(&field, 1) == nvrhi::FS_OK);
            tick(env, f, dt, 2);
            c->mapParticleData(data);
            double worst = 0;
            for (uint32_t i = 0; i < n; ++i) {
                const P& p = layout[i];
                double d[3] = {p.pos[0] - centre[0], p.pos[1] - centre[1], p.pos[2] - centre[2]};
                const double dist = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
                double expected[3] = {0, 0, 0};
                // A fixed particle stays at rest: the solver's last stages give it no velocity, whatever the field.
                if (dist < radius && p.invMass > 0.0f) {
                    double s = strength;
                    if (falloff) s *= 1.0 - dist / radius;
                    const double unit = modes[m] == ForceMode::Force ? dt * p.invMass
                                        : modes[m] == ForceMode::Impulse ? p.invMass
                                                                          : 1.0;
                    for (int j = 0; j < 3; ++j) expected[j] = (dist > 0 ? d[j] / dist : d[j]) * s * unit;
                }
                const float* v = &data.velocities[size_t(indices[i]) * 3];
                for (int j = 0; j < 3; ++j) worst = std::max(worst, std::fabs(v[j] - expected[j]));
                if (run == 5 && i == 4)
                    std::printf("  the fixed particle under a velocity change: v (%.3f %.3f %.3f)\n", v[0], v[1], v[2]);
            }
            std::printf("  %s%s: velocities within %.2e of the analytic change\n", names[m], falloff ? ", linear falloff" : "",
                        worst);
            CHECK(worst < 1e-5);
            c->unmapParticleData();
        }
    CHECK(counting->calls == run);  // chained: called once per step

    // No fields: no dispatch, the velocities stay.
    CHECK(set->setForceFields(nullptr, 0) == nvrhi::FS_OK);
    CHECK(set->getForceFields() == nullptr);
    ForceField broken;
    broken.mode = ForceMode(7);
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, set->setForceFields(&broken, 1));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, set->setForceFields(nullptr, 1));

    // Releasing the set gives the stage back to the application's callback.
    set = nullptr;
    ephys::PhysSolverStageCallbackHandle current;
    CHECK(f.solver->setStageCallback(ephys::SolverStage::UpdateEnd, nullptr, &current) == nvrhi::FS_OK);
    CHECK(current.Get() == counting.Get());
    current = nullptr;
    CHECK(refCount(counting.Get()) == 1);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    env.device->runGarbageCollection();
    const nvrhi::FLONG solverRefs = refCount(f.solver.Get());
    CHECK(solverRefs == 2);  // the test and the container
}

void refusals(Env& env) {
    std::printf("refusals\n");
    Fixture f = makeFixture(env, 32, floatingParams(0.1f));
    if (!f) return;
    ephys::PhysContainerHandle none;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, env.phys->createContainer(f.solver, 0, &none));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, env.phys->createContainer(f.solver, 33, &none));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, env.phys->createContainer(nullptr, 8, &none));
    ephys::ForceFieldSetHandle noSet;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, env.phys->createForceFieldSet(nullptr, &noSet));
    ephys::MovingFrameHandle noFrame;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, env.phys->createMovingFrame(nullptr, nullptr, &noFrame));
    ephys::PhysInstanceHandle noInstance;
    const float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, f.container->createInstance(nullptr, nullptr, m, m, 0, 1.0f, &noInstance));
    const int32_t outOfRange[] = {32};
    const float local[3] = {0, 0, 0};
    ephys::PhysSoftJointHandle noJoint;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, env.messages, f.container->createSoftJoint(outOfRange, local, 1, 1.0f, &noJoint));
    CHECK(!none && !noSet && !noFrame && !noInstance && !noJoint);
}

void run(Env& env, D3DMessages& d3d) {
    ephys::PhysAssetBuilderHandle builder;
    CHECK(env.phys->createAssetBuilder(&builder) == nvrhi::FS_OK);
    if (!builder) return;
    testAllocator(env);
    testInstances(env, builder);
    testSoftJoint(env, builder);
    testMovingFrame(env);
    testForceFields(env);
    refusals(env);
    CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
    env.device->runGarbageCollection();
    CHECK(d3d.problems() == 0);
}

}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    Options options;
    parse(argc, argv, options);
    if (!options.rest.empty()) {
        std::printf("usage: %s <d3d11|d3d12|vulkan> [--adapter <n>] [--shaders <dir>]\n", argv[0]);
        return 1;
    }
    return runWithDevice(options, run);
}
