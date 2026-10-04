// ethereal_phys_ext_test: the asset builders and tearing (IPhysAssetBuilder, IPhysAsset, IPhysTearingCloth) on a
// real, headless device.
//
//     ethereal_phys_ext_test <d3d11|d3d12|vulkan> --assets <dir> [--shaders <dir>] [--adapter <n>]
//
// Without a reference (ext_parity_test.cpp compares with FleX bit for bit when one is configured):
//
// - every builder, on procedural meshes and meshes of assets/, gives a well-formed asset: indices in range,
//   counts that agree with each other, spring rest lengths that are the distances of their particles (tethers:
//   to a fixed particle, with the give), shape offsets that increase to the index count, one rigid shape over all
//   particles with its centre at their mean, skinning weights that sum to 1;
// - the arguments FleX did not check are refused with FE_INVALID_ARGS and exactly one message;
// - a cloth asset and a rigid asset (and a soft body) in a domain, stepped by the solver for two seconds: the
//   particles stay finite, the cloth swings down and stays connected (no spring stretched beyond 1.5 times its
//   rest length, the fixed particles where they were), the rigid body falls onto the ground and keeps its shape
//   (every particle within half the particle spacing of where the solver's transform of the body puts it, and
//   every distance between two of its particles within half the spacing of the rest pose);
// - a tearing cloth pulled apart by moving its fixed edge: tearing produces clones and mesh edits, the
//   particles stay finite, and the springs and triangles the asset holds stay within its particle count.
//
// Exit code: 0 passed, 1 failed, 77 no device for this graphics API.

#include <ethereal/phys/device.h>
#include <ethereal/phys/ext/asset_builder.h>
#include <ethereal/phys_utils/mesh.h>

#include "solver_support.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace support;
using ephys::IPhysAsset;
using ephys::IPhysAssetBuilder;
using ephys::IPhysTearingCloth;

std::filesystem::path g_Assets;

struct MeshData {
    std::string name;
    std::vector<float> vertices;  // 3 per vertex
    std::vector<int32_t> indices;

    uint32_t numVertices() const { return uint32_t(vertices.size() / 3); }
    uint32_t numIndices() const { return uint32_t(indices.size()); }
    uint32_t numTriangles() const { return uint32_t(indices.size() / 3); }
};

MeshData loadMesh(const char* file, dm::float3 scale, dm::float3 translation = dm::float3(0.0f)) {
    MeshData data;
    data.name = file;
    std::unique_ptr<ephys::utils::Mesh> mesh = ephys::utils::ImportMesh(g_Assets / file);
    CHECK(mesh != nullptr);
    if (!mesh)
        return data;
    mesh->Normalize();
    mesh->Transform(dm::scaling(scale) * dm::translation(translation));
    for (const dm::float3& p : mesh->m_positions)
        data.vertices.insert(data.vertices.end(), {p.x, p.y, p.z});
    for (uint32_t i : mesh->m_indices)
        data.indices.push_back(int32_t(i));
    return data;
}

// nx x nz quads in the x z plane at height y, shared vertices.
MeshData gridMesh(int nx, int nz, float spacing, dm::float3 origin) {
    MeshData data;
    data.name = "grid";
    for (int z = 0; z <= nz; ++z)
        for (int x = 0; x <= nx; ++x)
            data.vertices.insert(data.vertices.end(),
                                 {origin.x + float(x) * spacing, origin.y, origin.z + float(z) * spacing});
    auto index = [&](int x, int z) { return int32_t(z * (nx + 1) + x); };
    for (int z = 0; z < nz; ++z)
        for (int x = 0; x < nx; ++x) {
            data.indices.insert(data.indices.end(), {index(x, z), index(x, z + 1), index(x + 1, z + 1)});
            data.indices.insert(data.indices.end(), {index(x, z), index(x + 1, z + 1), index(x + 1, z)});
        }
    return data;
}

MeshData weld(IPhysAssetBuilder* builder, const MeshData& mesh, float threshold) {
    const uint32_t n = mesh.numVertices();
    std::vector<int32_t> unique(n), remap(n);
    uint32_t count = 0;
    CHECK(builder->createWeldedMeshIndices(mesh.vertices.data(), n, unique.data(), remap.data(), threshold, count) ==
          nvrhi::FS_OK);
    MeshData welded;
    welded.name = mesh.name + " (welded)";
    for (uint32_t i = 0; i < count; ++i)
        welded.vertices.insert(welded.vertices.end(), &mesh.vertices[size_t(unique[i]) * 3],
                               &mesh.vertices[size_t(unique[i]) * 3] + 3);
    for (int32_t index : mesh.indices)
        welded.indices.push_back(remap[size_t(index)]);
    return welded;
}

template <typename F>
std::vector<float> particlesOf(const MeshData& mesh, F pinned) {
    std::vector<float> particles;
    for (uint32_t i = 0; i < mesh.numVertices(); ++i) {
        const float* p = &mesh.vertices[size_t(i) * 3];
        particles.insert(particles.end(), {p[0], p[1], p[2], pinned(p[0], p[1], p[2]) ? 0.0f : 1.0f});
    }
    return particles;
}

float distance3(const float* a, const float* b) {
    const float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool allFinite(const float* values, size_t n) {
    for (size_t i = 0; i < n; ++i)
        if (!std::isfinite(values[i]))
            return false;
    return true;
}

// ---- Well-formed assets -------------------------------------------------------------------------------------

// Indices in range, counts consistent, every spring's rest length the distance of its particles (a tether's,
// with the give, to a fixed particle).
void checkSprings(const char* what, IPhysAsset* asset, const float* particles, uint32_t numParticles, float give) {
    const uint32_t n = asset->getSpringCount();
    if (n == 0) {
        CHECK(asset->getSpringIndices() == nullptr && asset->getSpringRestLengths() == nullptr);
        return;
    }
    const int32_t* indices = asset->getSpringIndices();
    const float* lengths = asset->getSpringRestLengths();
    const float* coefficients = asset->getSpringCoefficients();
    CHECK(indices && lengths && coefficients);
    if (!indices || !lengths || !coefficients)
        return;
    uint32_t bad = 0;
    for (uint32_t s = 0; s < n; ++s) {
        const int32_t a = indices[s * 2], b = indices[s * 2 + 1];
        if (a < 0 || b < 0 || uint32_t(a) >= numParticles || uint32_t(b) >= numParticles || a == b) {
            ++bad;
            continue;
        }
        float expected = distance3(&particles[size_t(a) * 4], &particles[size_t(b) * 4]);
        if (coefficients[s] < 0.0f) {
            expected *= 1.0f + give;
            if (particles[size_t(b) * 4 + 3] != 0.0f)
                ++bad;  // a tether ends at a fixed particle
        }
        if (std::fabs(lengths[s] - expected) > 1e-5f * std::max(1.0f, expected))
            ++bad;
    }
    if (bad)
        std::printf("  %s: %u of %u springs are not well-formed\n", what, bad, n);
    CHECK(bad == 0);
}

void checkCloth(IPhysAssetBuilder* builder, const MeshData& mesh, const std::vector<float>& particles,
                const ephys::ClothAssetParams& params) {
    ephys::PhysAssetHandle asset;
    CHECK(builder->createClothFromMesh(particles.data(), mesh.numVertices(), mesh.indices.data(), mesh.numTriangles(),
                                       params, &asset) == nvrhi::FS_OK);
    if (!asset)
        return;
    const uint32_t n = mesh.numVertices();
    CHECK(asset->getParticleCount() == n && asset->getMaxParticles() == n);
    CHECK(std::memcmp(asset->getParticles(), particles.data(), particles.size() * 4) == 0);
    CHECK(asset->getTriangleCount() == mesh.numTriangles());
    CHECK(std::memcmp(asset->getTriangleIndices(), mesh.indices.data(), mesh.indices.size() * 4) == 0);
    CHECK(asset->getShapeCount() == 0 && asset->getShapeIndexCount() == 0 && asset->getShapeIndices() == nullptr);

    // a stretch spring per edge, a bending spring per interior edge (when bending), a tether per free particle
    std::set<std::pair<int32_t, int32_t>> edges;
    std::map<std::pair<int32_t, int32_t>, int> edgeTriangles;
    for (uint32_t t = 0; t < mesh.numTriangles(); ++t)
        for (int e = 0; e < 3; ++e) {
            const int32_t a = mesh.indices[t * 3 + e], b = mesh.indices[t * 3 + (e + 1) % 3];
            const auto key = std::make_pair(std::min(a, b), std::max(a, b));
            edges.insert(key);
            ++edgeTriangles[key];
        }
    size_t interior = 0;
    for (const auto& [edge, count] : edgeTriangles)
        interior += count == 2 ? 1 : 0;
    size_t fixed = 0;
    for (uint32_t i = 0; i < n; ++i)
        fixed += particles[size_t(i) * 4 + 3] == 0.0f ? 1 : 0;
    const size_t tethers = params.tetherStiffness > 0.0f && fixed ? n - fixed : 0;
    const size_t expected = edges.size() + (params.bendStiffness > 0.0f ? interior : 0) + tethers;
    if (asset->getSpringCount() != expected)
        std::printf("  cloth %s: %u springs, expected %zu\n", mesh.name.c_str(), asset->getSpringCount(), expected);
    CHECK(asset->getSpringCount() == expected);
    checkSprings(mesh.name.c_str(), asset, particles.data(), n, params.tetherGive);

    if (params.pressure > 0.0f) {
        CHECK(asset->isInflatable());
        CHECK(asset->getInflatablePressure() == params.pressure);
        CHECK(std::isfinite(asset->getInflatableVolume()) && asset->getInflatableVolume() != 0.0f);
        CHECK(asset->getInflatableStiffness() > 0.0f && std::isfinite(asset->getInflatableStiffness()));
    } else {
        CHECK(!asset->isInflatable());
    }
    std::printf("cloth %s: %u particles, %u springs%s\n", mesh.name.c_str(), n, asset->getSpringCount(),
                asset->isInflatable() ? ", inflatable" : "");
}

// Shapes: offsets increasing to the index count, indices in range, centres finite.
void checkShapes(const char* what, IPhysAsset* asset) {
    const uint32_t shapes = asset->getShapeCount();
    const uint32_t count = asset->getShapeIndexCount();
    const uint32_t n = asset->getParticleCount();
    CHECK(shapes > 0);
    if (!shapes)
        return;
    const int32_t* offsets = asset->getShapeOffsets();
    const int32_t* indices = asset->getShapeIndices();
    const float* centers = asset->getShapeCenters();
    const float* coefficients = asset->getShapeCoefficients();
    CHECK(offsets && indices && centers && coefficients);
    if (!offsets || !indices || !centers || !coefficients)
        return;
    int32_t previous = 0;
    uint32_t bad = 0;
    for (uint32_t s = 0; s < shapes; ++s) {
        if (offsets[s] <= previous)
            ++bad;
        previous = offsets[s];
        if (!allFinite(&centers[s * 3], 3))
            ++bad;
    }
    CHECK(uint32_t(previous) == count);
    for (uint32_t i = 0; i < count; ++i)
        if (indices[i] < 0 || uint32_t(indices[i]) >= n)
            ++bad;
    if (bad)
        std::printf("  %s: %u problems in the shapes\n", what, bad);
    CHECK(bad == 0);
}

void checkRigid(IPhysAssetBuilder* builder, const MeshData& mesh, float spacing, float expand) {
    ephys::PhysAssetHandle asset;
    CHECK(builder->createRigidFromMesh(mesh.vertices.data(), mesh.numVertices(), mesh.indices.data(),
                                       mesh.numIndices(), spacing, expand, &asset) == nvrhi::FS_OK);
    if (!asset)
        return;
    const uint32_t n = asset->getParticleCount();
    CHECK(n > 0 && asset->getMaxParticles() == n);
    CHECK(asset->getShapeCount() == 1 && asset->getShapeIndexCount() == n);
    checkShapes(mesh.name.c_str(), asset);
    CHECK(asset->getShapeOffsets()[0] == int32_t(n));
    CHECK(asset->getShapeCoefficients()[0] == 1.0f);
    CHECK(asset->getSpringCount() == 0 && asset->getTriangleCount() == 0 && !asset->isInflatable());
    CHECK(asset->getShapePlasticThresholds() == nullptr && asset->getShapePlasticCreeps() == nullptr);

    // all particles, in order; mass 1; inside the bounds of the mesh (with the expansion and half a voxel)
    float lower[3] = {FLT_MAX, FLT_MAX, FLT_MAX}, upper[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
    for (uint32_t v = 0; v < mesh.numVertices(); ++v)
        for (int k = 0; k < 3; ++k) {
            lower[k] = std::min(lower[k], mesh.vertices[size_t(v) * 3 + k]);
            upper[k] = std::max(upper[k], mesh.vertices[size_t(v) * 3 + k]);
        }
    const float* particles = asset->getParticles();
    double mean[3] = {0, 0, 0};
    uint32_t bad = 0;
    for (uint32_t i = 0; i < n; ++i) {
        if (asset->getShapeIndices()[i] != int32_t(i) || particles[i * 4 + 3] != 1.0f)
            ++bad;
        for (int k = 0; k < 3; ++k) {
            const float p = particles[i * 4 + k];
            mean[k] += p;
            if (!(p >= lower[k] - spacing - std::fabs(expand) && p <= upper[k] + spacing + std::fabs(expand)))
                ++bad;
        }
    }
    CHECK(bad == 0);
    for (int k = 0; k < 3; ++k)
        CHECK(std::fabs(float(mean[k] / n) - asset->getShapeCenters()[k]) < 1e-4f);
    std::printf("rigid %s @%g: %u particles\n", mesh.name.c_str(), double(spacing), n);
}

void checkSoft(IPhysAssetBuilder* builder, const MeshData& mesh, const ephys::SoftAssetParams& params) {
    ephys::PhysAssetHandle asset;
    CHECK(builder->createSoftFromMesh(mesh.vertices.data(), mesh.numVertices(), mesh.indices.data(),
                                      mesh.numIndices(), params, &asset) == nvrhi::FS_OK);
    if (!asset)
        return;
    const uint32_t n = asset->getParticleCount();
    CHECK(n > 0 && asset->getMaxParticles() == n);
    CHECK(allFinite(asset->getParticles(), size_t(n) * 4));
    checkShapes(mesh.name.c_str(), asset);
    checkSprings(mesh.name.c_str(), asset, asset->getParticles(), n, 0.0f);
    if (params.linkRadius > 0.0f)
        CHECK(asset->getSpringCount() > 0);
    const bool plastic = params.clusterPlasticCreep != 0.0f;
    CHECK((asset->getShapePlasticThresholds() != nullptr) == plastic);
    CHECK((asset->getShapePlasticCreeps() != nullptr) == plastic);
    if (params.globalStiffness > 0.0f) {
        // the last shape is all particles
        const uint32_t shapes = asset->getShapeCount();
        const int32_t begin = shapes > 1 ? asset->getShapeOffsets()[shapes - 2] : 0;
        CHECK(uint32_t(asset->getShapeOffsets()[shapes - 1] - begin) == n);
        CHECK(asset->getShapeCoefficients()[shapes - 1] == params.globalStiffness);
    }

    // the skinning of the mesh to the clusters: weights that sum to 1, bones in range or -1 with weight 0
    const uint32_t vertices = mesh.numVertices();
    std::vector<float> weights(size_t(vertices) * 4);
    std::vector<int32_t> bones(size_t(vertices) * 4);
    CHECK(builder->createSoftMeshSkinning(mesh.vertices.data(), vertices, asset->getShapeCenters(),
                                          asset->getShapeCount(), 2.0f, 100.0f, weights.data(),
                                          bones.data()) == nvrhi::FS_OK);
    uint32_t bad = 0;
    for (uint32_t v = 0; v < vertices; ++v) {
        float sum = 0.0f;
        for (int k = 0; k < 4; ++k) {
            const float w = weights[size_t(v) * 4 + k];
            const int32_t bone = bones[size_t(v) * 4 + k];
            sum += w;
            if (!(w >= 0.0f && w <= 1.0f) || bone < -1 || bone >= int32_t(asset->getShapeCount()) ||
                (bone == -1 && w != 0.0f))
                ++bad;
        }
        if (std::fabs(sum - 1.0f) > 1e-5f)
            ++bad;
    }
    CHECK(bad == 0);
    std::printf("soft %s: %u particles, %u clusters, %u links\n", mesh.name.c_str(), n, asset->getShapeCount(),
                asset->getSpringCount());
}

void checkWeld(IPhysAssetBuilder* builder, const MeshData& mesh, float threshold) {
    const uint32_t n = mesh.numVertices();
    std::vector<int32_t> unique(n, -7), remap(n, -7);
    uint32_t count = 0;
    CHECK(builder->createWeldedMeshIndices(mesh.vertices.data(), n, unique.data(), remap.data(), threshold, count) ==
          nvrhi::FS_OK);
    CHECK(count > 0 && count <= n);
    std::set<int32_t> distinct;
    uint32_t bad = 0;
    for (uint32_t i = 0; i < count; ++i) {
        if (unique[i] < 0 || uint32_t(unique[i]) >= n || !distinct.insert(unique[i]).second)
            ++bad;
        else if (remap[size_t(unique[i])] != int32_t(i))
            ++bad;  // a unique vertex maps to itself
    }
    for (uint32_t v = 0; v < n && !bad; ++v) {
        const int32_t u = remap[v];
        if (u < 0 || uint32_t(u) >= count)
            ++bad;
        else if (distance3(&mesh.vertices[size_t(v) * 3], &mesh.vertices[size_t(unique[size_t(u)]) * 3]) > threshold)
            ++bad;
    }
    CHECK(bad == 0);
    std::printf("weld %s: %u -> %u vertices\n", mesh.name.c_str(), n, count);
}

void wellFormed(Env& env, IPhysAssetBuilder* builder) {
    (void)env;
    checkWeld(builder, loadMesh("box_high.ply", dm::float3(1.0f)), 1e-5f);
    checkWeld(builder, loadMesh("irregular_plane.obj", dm::float3(2.0f)), 1e-5f);
    checkWeld(builder, loadMesh("bunny.ply", dm::float3(1.0f)), 0.01f);

    const MeshData grid = gridMesh(20, 16, 0.05f, dm::float3(0.0f, 1.0f, 0.0f));
    ephys::ClothAssetParams params;
    params.stretchStiffness = 0.9f;
    params.bendStiffness = 0.7f;
    params.tetherStiffness = 0.5f;
    params.tetherGive = 0.1f;
    checkCloth(builder, grid, particlesOf(grid, [](float, float, float z) { return z == 0.0f; }), params);

    const MeshData plane = weld(builder, loadMesh("irregular_plane.obj", dm::float3(2.0f)), 1e-5f);
    params = ephys::ClothAssetParams();
    checkCloth(builder, plane, particlesOf(plane, [](float x, float, float) { return x == 0.0f; }), params);

    const MeshData sphere = weld(builder, loadMesh("sphere.ply", dm::float3(1.0f)), 1e-5f);
    params = ephys::ClothAssetParams();
    params.pressure = 1.2f;
    checkCloth(builder, sphere, particlesOf(sphere, [](float, float, float) { return false; }), params);

    checkRigid(builder, loadMesh("box.ply", dm::float3(1.0f)), 0.05f, -0.025f);
    checkRigid(builder, loadMesh("bunny.ply", dm::float3(1.0f)), 0.04f, -0.02f);
    checkRigid(builder, loadMesh("torus.obj", dm::float3(1.0f)), 0.03f, 0.0f);
    checkRigid(builder, loadMesh("rockb.ply", dm::float3(0.5f)), 0.02f, 0.0f);

    ephys::SoftAssetParams soft;
    soft.clusterSpacing = 0.3f;
    soft.clusterStiffness = 0.2f;
    checkSoft(builder, loadMesh("bunny.ply", dm::float3(2.0f)), soft);
    soft = ephys::SoftAssetParams();
    soft.clusterSpacing = 0.1f;
    soft.globalStiffness = 1.0f;
    soft.clusterPlasticThreshold = 0.002f;
    soft.clusterPlasticCreep = 0.2f;
    soft.linkRadius = 0.12f;
    checkSoft(builder, loadMesh("sphere.ply", dm::float3(0.8f)), soft);
    soft = ephys::SoftAssetParams();
    soft.volumeSampling = 0.0f;
    soft.surfaceSampling = 0.2f;
    soft.clusterSpacing = 0.2f;
    soft.clusterRadius = 0.3f;
    checkSoft(builder, loadMesh("softs/octopus.obj", dm::float3(2.0f)), soft);
}

// ---- Refusals -----------------------------------------------------------------------------------------------

void refusals(Env& env, IPhysAssetBuilder* builder) {
    PhysMessages* messages = env.messages;
    const MeshData grid = gridMesh(4, 4, 0.1f, dm::float3(0.0f));
    const std::vector<float> particles = particlesOf(grid, [](float, float, float) { return false; });
    const MeshData box = loadMesh("box.ply", dm::float3(1.0f));
    ephys::PhysAssetHandle asset;
    ephys::PhysTearingClothHandle cloth;

    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages,
                builder->createClothFromMesh(nullptr, 25, grid.indices.data(), grid.numTriangles(),
                                             ephys::ClothAssetParams(), &asset));
    CHECK(asset == nullptr);
    std::vector<int32_t> outOfRange = grid.indices;
    outOfRange[5] = 25;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages,
                builder->createClothFromMesh(particles.data(), grid.numVertices(), outOfRange.data(),
                                             grid.numTriangles(), ephys::ClothAssetParams(), &asset));
    // three triangles on one edge
    const float fan[] = {0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, -1, 0, 1, 0, 0, 1, 1};
    const int32_t fanIndices[] = {0, 1, 2, 1, 0, 3, 0, 1, 4};
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages,
                builder->createClothFromMesh(fan, 5, fanIndices, 3, ephys::ClothAssetParams(), &asset));
    CHECK(asset == nullptr);
    // a repeated vertex
    const int32_t degenerate[] = {0, 1, 1};
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages,
                builder->createClothFromMesh(fan, 5, degenerate, 1, ephys::ClothAssetParams(), &asset));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages,
                builder->createTearingClothFromMesh(particles.data(), grid.numVertices(), grid.numVertices() - 1,
                                                    grid.indices.data(), grid.numTriangles(),
                                                    ephys::TearingClothAssetParams(), &cloth));
    CHECK(cloth == nullptr);
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages,
                builder->createRigidFromMesh(box.vertices.data(), box.numVertices(), box.indices.data(),
                                             box.numIndices() - 1, 0.05f, 0.0f, &asset));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages,
                builder->createRigidFromMesh(box.vertices.data(), box.numVertices(), box.indices.data(),
                                             box.numIndices(), 0.01f, 0.0f, &asset));  // > 64 voxels per side
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages,
                builder->createRigidFromMesh(box.vertices.data(), box.numVertices(), box.indices.data(),
                                             box.numIndices(), 0.0f, 0.0f, &asset));
    std::vector<float> notFinite = box.vertices;
    notFinite[7] = NAN;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages,
                builder->createRigidFromMesh(notFinite.data(), box.numVertices(), box.indices.data(),
                                             box.numIndices(), 0.05f, 0.0f, &asset));
    ephys::SoftAssetParams nothing;
    nothing.volumeSampling = 0.0f;
    nothing.surfaceSampling = 0.0f;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages,
                builder->createSoftFromMesh(box.vertices.data(), box.numVertices(), box.indices.data(),
                                            box.numIndices(), nothing, &asset));
    CHECK(asset == nullptr);
    uint32_t count = 0;
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages,
                builder->createWeldedMeshIndices(box.vertices.data(), box.numVertices(), nullptr, nullptr, 0.0f, count));
    CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages,
                builder->createSoftMeshSkinning(box.vertices.data(), box.numVertices(), nullptr, 3, 1.0f, 1.0f, nullptr,
                                                nullptr));

    // the tearing cloth checks its arguments too
    CHECK(builder->createTearingClothFromMesh(particles.data(), grid.numVertices(), grid.numVertices() + 10,
                                             grid.indices.data(), grid.numTriangles(), ephys::TearingClothAssetParams(),
                                             &cloth) == nvrhi::FS_OK);
    if (cloth) {
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, cloth->setParticles(particles.data(), 30, 6));
        uint32_t copies = 0, edits = 0;
        CHECK_FAILS(nvrhi::FE_INVALID_ARGS, messages, cloth->tearClothMesh(2.0f, 1, nullptr, 4, copies, nullptr, 0, edits));
        // unstretched: nothing tears, and null arrays without capacity are fine
        CHECK(cloth->tearClothMesh(1.5f, 4, nullptr, 0, copies, nullptr, 0, edits) == nvrhi::FS_OK);
        CHECK(copies == 0 && edits == 0 && cloth->getParticleCount() == grid.numVertices());
        // the asset answers IPhysAsset
        ephys::PhysAssetHandle base;
        CHECK(cloth->QueryInterface(NVRHI_IID_PPV_ARGS(&base)) == nvrhi::FS_OK && base);
    }
}

// ---- In a domain --------------------------------------------------------------------------------------------

scenes::Scene baseScene(const char* name) {
    scenes::Scene s;
    s.name = name;
    s.params = scenes::defaultParams();
    s.params.radius = 0.05f;
    s.params.dynamicFriction = 0.4f;
    s.params.numIterations = 6;
    s.params.relaxationFactor = 1.0f;
    scenes::setPlaneBox(s.params, {-3.0f, 0.0f, -3.0f}, {3.0f, 6.0f, 3.0f}, 1);
    s.spareParticles = 0;
    return s;
}

void addAssetParticles(scenes::Scene& s, IPhysAsset* asset, int32_t phase) {
    const float* p = asset->getParticles();
    for (uint32_t i = 0; i < asset->getParticleCount(); ++i)
        scenes::addParticle(s, {p[i * 4], p[i * 4 + 1], p[i * 4 + 2]}, p[i * 4 + 3], {0, 0, 0}, phase);
}

void addAssetSprings(scenes::Scene& s, IPhysAsset* asset) {
    const uint32_t n = asset->getSpringCount();
    s.springIndices.assign(asset->getSpringIndices(), asset->getSpringIndices() + n * 2);
    s.springLengths.assign(asset->getSpringRestLengths(), asset->getSpringRestLengths() + n);
    s.springStiffness.assign(asset->getSpringCoefficients(), asset->getSpringCoefficients() + n);
}

// The shapes of an asset as IShapeMatchSet takes them: offsets with a leading 0, rest positions relative to the
// centre of each shape (FleX's CalculateRigidLocalPositions does the same with the mean of the shape's particles).
void addAssetShapes(scenes::Scene& s, IPhysAsset* asset) {
    const float* p = asset->getParticles();
    const int32_t* offsets = asset->getShapeOffsets();
    const int32_t* indices = asset->getShapeIndices();
    s.rigidOffsets.push_back(0);
    int32_t begin = 0;
    for (uint32_t r = 0; r < asset->getShapeCount(); ++r) {
        const int32_t end = offsets[r];
        double c[3] = {0, 0, 0};
        for (int32_t k = begin; k < end; ++k)
            for (int d = 0; d < 3; ++d)
                c[d] += p[size_t(indices[k]) * 4 + d];
        for (double& v : c)
            v /= double(end - begin);
        for (int32_t k = begin; k < end; ++k) {
            const float* q = &p[size_t(indices[k]) * 4];
            s.rigidIndices.push_back(indices[k]);
            s.rigidRestPositions.insert(s.rigidRestPositions.end(),
                                        {q[0] - float(c[0]), q[1] - float(c[1]), q[2] - float(c[2])});
            s.rigidRestNormals.insert(s.rigidRestNormals.end(), {0.0f, 0.0f, 0.0f, 0.0f});
        }
        s.rigidOffsets.push_back(end);
        s.rigidStiffness.push_back(asset->getShapeCoefficients()[r]);
        s.rigidRotations.insert(s.rigidRotations.end(), {0.0f, 0.0f, 0.0f, 1.0f});
        s.rigidTranslations.insert(s.rigidTranslations.end(), {float(c[0]), float(c[1]), float(c[2])});
        begin = end;
    }
    if (asset->getShapePlasticThresholds()) {
        s.rigidThresholds.assign(asset->getShapePlasticThresholds(),
                                 asset->getShapePlasticThresholds() + asset->getShapeCount());
        s.rigidCreeps.assign(asset->getShapePlasticCreeps(), asset->getShapePlasticCreeps() + asset->getShapeCount());
    }
}

void clothInDomain(Env& env, IPhysAssetBuilder* builder) {
    // a square in the x z plane pinned along z = 0, swinging down
    const MeshData grid = gridMesh(24, 24, 0.04f, dm::float3(-0.5f, 2.0f, -0.5f));
    const std::vector<float> particles = particlesOf(grid, [](float, float, float z) { return z == -0.5f; });
    ephys::ClothAssetParams params;
    params.stretchStiffness = 1.0f;
    params.bendStiffness = 0.5f;
    params.tetherStiffness = 0.5f;
    ephys::PhysAssetHandle asset;
    CHECK(builder->createClothFromMesh(particles.data(), grid.numVertices(), grid.indices.data(), grid.numTriangles(),
                                       params, &asset) == nvrhi::FS_OK);
    if (!asset)
        return;

    scenes::Scene s = baseScene("asset cloth");
    s.params.radius = 0.04f;
    scenes::finishParams(s.params);
    addAssetParticles(s, asset, ephys::makePhase(0, ephys::PhaseSelfCollide | ephys::PhaseSelfCollideFilter));
    addAssetSprings(s, asset);
    s.triangleIndices.assign(asset->getTriangleIndices(), asset->getTriangleIndices() + asset->getTriangleCount() * 3);
    s.triangleNormals.assign(size_t(asset->getTriangleCount()) * 3, 0.0f);
    s.restPositions = s.positions;

    Rig rig = loadScene(env, s);
    CHECK(bool(rig));
    if (!rig)
        return;
    // the cloth swings like a pendulum: its lowest point over the run, read every 10 frames
    const uint32_t n = s.numParticles();
    float lowest = FLT_MAX;
    for (int frame = 0; frame < 120; ++frame) {
        stepScene(env, s, rig);
        if (frame % 10 == 9) {
            const std::vector<float> p = readPositions(env, rig, n);
            for (uint32_t i = 0; i < n; ++i)
                lowest = std::min(lowest, p[i * 4 + 1]);
        }
    }

    const std::vector<float> positions = readPositions(env, rig, n);
    CHECK(allFinite(positions.data(), positions.size()));
    uint32_t moved = 0, stretched = 0;
    float worst = 0.0f;
    for (uint32_t i = 0; i < n; ++i) {
        if (s.positions[i * 4 + 3] == 0.0f && std::memcmp(&positions[i * 4], &s.positions[i * 4], 12) != 0)
            ++moved;
    }
    for (uint32_t k = 0; k < s.numSprings(); ++k) {
        if (s.springStiffness[k] < 0.0f)
            continue;  // tethers may be slack
        const float ratio = distance3(&positions[size_t(s.springIndices[k * 2]) * 4],
                                      &positions[size_t(s.springIndices[k * 2 + 1]) * 4]) /
                            s.springLengths[k];
        worst = std::max(worst, ratio);
        if (ratio > 1.5f)
            ++stretched;
    }
    std::printf("cloth in a domain: lowest point %.3f over the run (from 2.0), worst spring stretch %.3f\n", double(lowest),
                double(worst));
    CHECK(moved == 0);       // the fixed particles stay where they are
    CHECK(stretched == 0);   // connected
    CHECK(lowest < 1.3f);    // swung down
}

void rigidInDomain(Env& env, IPhysAssetBuilder* builder) {
    const float spacing = 0.05f;
    const MeshData bunny = loadMesh("bunny.ply", dm::float3(0.6f), dm::float3(0.0f, 1.0f, 0.0f));
    ephys::PhysAssetHandle rigid;
    CHECK(builder->createRigidFromMesh(bunny.vertices.data(), bunny.numVertices(), bunny.indices.data(),
                                       bunny.numIndices(), spacing, -0.5f * spacing, &rigid) == nvrhi::FS_OK);
    ephys::SoftAssetParams softParams;
    softParams.particleSpacing = spacing;
    softParams.clusterSpacing = 0.15f;
    softParams.clusterStiffness = 0.4f;
    const MeshData sphere = loadMesh("sphere.ply", dm::float3(0.4f), dm::float3(1.5f, 1.5f, 0.0f));
    ephys::PhysAssetHandle soft;
    CHECK(builder->createSoftFromMesh(sphere.vertices.data(), sphere.numVertices(), sphere.indices.data(),
                                      sphere.numIndices(), softParams, &soft) == nvrhi::FS_OK);
    if (!rigid || !soft)
        return;

    scenes::Scene s = baseScene("asset rigid and soft");
    s.params.radius = spacing;
    scenes::finishParams(s.params);
    addAssetParticles(s, rigid, ephys::makePhase(1, 0));
    addAssetShapes(s, rigid);
    const uint32_t rigidParticles = s.numParticles();

    // the soft body after the rigid one: its indices shifted
    scenes::Scene softPart;
    addAssetParticles(softPart, soft, ephys::makePhase(2, 0));
    addAssetShapes(softPart, soft);
    s.positions.insert(s.positions.end(), softPart.positions.begin(), softPart.positions.end());
    s.velocities.insert(s.velocities.end(), softPart.velocities.begin(), softPart.velocities.end());
    s.phases.insert(s.phases.end(), softPart.phases.begin(), softPart.phases.end());
    for (size_t k = 1; k < softPart.rigidOffsets.size(); ++k)
        s.rigidOffsets.push_back(softPart.rigidOffsets[k] + int32_t(s.rigidIndices.size()));
    for (int32_t index : softPart.rigidIndices)
        s.rigidIndices.push_back(index + int32_t(rigidParticles));
    s.rigidRestPositions.insert(s.rigidRestPositions.end(), softPart.rigidRestPositions.begin(),
                                softPart.rigidRestPositions.end());
    s.rigidRestNormals.insert(s.rigidRestNormals.end(), softPart.rigidRestNormals.begin(),
                              softPart.rigidRestNormals.end());
    s.rigidStiffness.insert(s.rigidStiffness.end(), softPart.rigidStiffness.begin(), softPart.rigidStiffness.end());
    s.rigidRotations.insert(s.rigidRotations.end(), softPart.rigidRotations.begin(), softPart.rigidRotations.end());
    s.rigidTranslations.insert(s.rigidTranslations.end(), softPart.rigidTranslations.begin(),
                               softPart.rigidTranslations.end());

    Rig rig = loadScene(env, s);
    CHECK(bool(rig));
    if (!rig)
        return;
    for (int frame = 0; frame < 120; ++frame)
        stepScene(env, s, rig);

    const uint32_t n = s.numParticles();
    const std::vector<float> positions = readPositions(env, rig, n);
    CHECK(allFinite(positions.data(), positions.size()));

    // The rigid body kept its shape: every particle where the solver's transform of the body puts its rest position
    // (up to the contact corrections that follow the shape matching), and every distance between two particles as in
    // the rest pose (a sample of the pairs).
    ephys::IShapeMatchSet* shapes = rig.domain->getShapeMatches();
    const std::vector<float> q = env.fetch<float>(shapes->getRotations(), 0, 1);
    const std::vector<float> t = env.fetch<float>(shapes->getTranslations(), 0, 1);
    CHECK(q.size() == 4 && t.size() == 3);
    if (q.size() != 4 || t.size() != 3)
        return;
    CHECK(allFinite(q.data(), 4) && allFinite(t.data(), 3));
    float worstTransform = 0.0f, worst = 0.0f, lowest = FLT_MAX;
    double centre = 0.0, start = 0.0;
    for (uint32_t i = 0; i < rigidParticles; ++i) {
        lowest = std::min(lowest, positions[i * 4 + 1]);
        centre += positions[i * 4 + 1];
        start += s.positions[i * 4 + 1];
        // the rest position rotated by the quaternion (x y z w): r + 2 w (u x r) + 2 u x (u x r), then translated
        const float* r = &s.rigidRestPositions[size_t(i) * 3];
        const float w = q[3];
        const float c1[3] = {q[1] * r[2] - q[2] * r[1], q[2] * r[0] - q[0] * r[2], q[0] * r[1] - q[1] * r[0]};
        const float c2[3] = {q[1] * c1[2] - q[2] * c1[1], q[2] * c1[0] - q[0] * c1[2], q[0] * c1[1] - q[1] * c1[0]};
        const float predicted[3] = {r[0] + 2 * w * c1[0] + 2 * c2[0] + t[0], r[1] + 2 * w * c1[1] + 2 * c2[1] + t[1],
                                    r[2] + 2 * w * c1[2] + 2 * c2[2] + t[2]};
        worstTransform = std::max(worstTransform, distance3(predicted, &positions[i * 4]));
        for (uint32_t j = i + 1; j < rigidParticles; j += 7) {
            const float rest = distance3(&s.positions[i * 4], &s.positions[j * 4]);
            const float now = distance3(&positions[i * 4], &positions[j * 4]);
            worst = std::max(worst, std::fabs(now - rest));
        }
    }
    float softLowest = FLT_MAX;
    for (uint32_t i = rigidParticles; i < n; ++i)
        softLowest = std::min(softLowest, positions[i * 4 + 1]);
    std::printf("rigid in a domain: %u particles, centre %.3f (from %.3f), lowest %.3f, worst distance from the body's "
                "transform %.4f, worst distance change %.4f (spacing %.2f); soft body lowest %.3f\n",
                rigidParticles, centre / rigidParticles, start / rigidParticles, double(lowest), double(worstTransform),
                double(worst), double(spacing), double(softLowest));
    CHECK(worstTransform < 0.5f * spacing);              // rigid
    CHECK(worst < 0.5f * spacing);
    CHECK(lowest > -0.01f && lowest < 0.1f);             // on the ground, not through it
    CHECK(centre / rigidParticles < start / rigidParticles - 0.3);  // it fell
    CHECK(softLowest > -0.01f && softLowest < 0.1f);     // the soft body rests on the ground too
}

// A tearing cloth held along x = 0 and pulled along its other edge, torn every frame as the demo does.
void tearingInDomain(Env& env, IPhysAssetBuilder* builder) {
    const MeshData grid = gridMesh(20, 20, 0.05f, dm::float3(-0.5f, 1.0f, -0.5f));
    const float right = -0.5f + 20 * 0.05f;
    std::vector<float> particles =
        particlesOf(grid, [&](float x, float, float) { return x == -0.5f || std::fabs(x - right) < 1e-6f; });
    const uint32_t n0 = grid.numVertices();
    const uint32_t maxParticles = n0 + 800;
    ephys::TearingClothAssetParams params;
    params.stretchStiffness = 0.8f;
    params.bendStiffness = 0.8f;
    ephys::PhysTearingClothHandle cloth;
    CHECK(builder->createTearingClothFromMesh(particles.data(), n0, maxParticles, grid.indices.data(),
                                             grid.numTriangles(), params, &cloth) == nvrhi::FS_OK);
    if (!cloth)
        return;

    scenes::Scene s = baseScene("tearing cloth");
    s.params.radius = 0.05f;
    s.params.relaxationMode = ephys::RelaxationMode::Global;
    s.params.relaxationFactor = 0.35f;
    s.params.numIterations = 8;
    scenes::finishParams(s.params);
    const int32_t phase = ephys::makePhase(0, ephys::PhaseSelfCollide | ephys::PhaseSelfCollideFilter);
    addAssetParticles(s, cloth, phase);
    addAssetSprings(s, cloth);
    s.triangleIndices.assign(cloth->getTriangleIndices(), cloth->getTriangleIndices() + cloth->getTriangleCount() * 3);
    s.triangleNormals.assign(size_t(cloth->getTriangleCount()) * 3, 0.0f);
    s.restPositions = s.positions;
    s.spareParticles = maxParticles - n0;

    Rig rig = loadScene(env, s);
    CHECK(bool(rig));
    if (!rig)
        return;
    ephys::IParticleState* state = rig.domain->getParticles();

    // the host copies of the arrays the clones are applied to
    std::vector<float> rest(size_t(maxParticles) * 4, 0.0f);
    std::copy(s.positions.begin(), s.positions.end(), rest.begin());
    std::vector<int32_t> phases(maxParticles, phase);

    const uint32_t maxCopies = 256, maxEdits = 1024;
    std::vector<ephys::TearingParticleClone> copies(maxCopies);
    std::vector<ephys::TearingMeshEdit> edits(maxEdits);
    uint32_t totalCopies = 0, totalEdits = 0;
    bool finite = true, inRange = true;

    for (int frame = 0; frame < 90; ++frame) {
        stepScene(env, s, rig);

        uint32_t count = cloth->getParticleCount();
        std::vector<float> positions = env.fetch<float>(state->getArray(ParticleArray::Position), 0, count);
        std::vector<float> velocities = env.fetch<float>(state->getArray(ParticleArray::Velocity), 0, count);
        finite = finite && allFinite(positions.data(), positions.size()) && allFinite(velocities.data(), velocities.size());

        // the pulled edge moves away (its particles are fixed: the solver leaves them where they are put)
        for (uint32_t i = 0; i < count; ++i)
            if (positions[i * 4 + 3] == 0.0f && positions[i * 4] > 0.0f)
                positions[i * 4] += 0.02f;

        CHECK(cloth->setParticles(positions.data(), 0, count) == nvrhi::FS_OK);
        uint32_t numCopies = 0, numEdits = 0;
        CHECK(cloth->tearClothMesh(2.0f, 4, copies.data(), maxCopies, numCopies, edits.data(), maxEdits, numEdits) ==
              nvrhi::FS_OK);
        totalCopies += numCopies;
        totalEdits += numEdits;

        // the clones (demo/scenes/tearing.h): position (with mass 1), velocity, phase, rest position
        const uint32_t newCount = cloth->getParticleCount();
        positions.resize(size_t(newCount) * 4, 0.0f);
        velocities.resize(size_t(newCount) * 3, 0.0f);
        for (uint32_t k = 0; k < numCopies; ++k) {
            const int32_t src = copies[k].srcIndex, dst = copies[k].destIndex;
            if (src < 0 || dst < 0 || uint32_t(src) >= newCount || uint32_t(dst) >= newCount) {
                inRange = false;
                continue;
            }
            std::memcpy(&positions[size_t(dst) * 4], &positions[size_t(src) * 4], 12);
            positions[size_t(dst) * 4 + 3] = 1.0f;
            std::memcpy(&velocities[size_t(dst) * 3], &velocities[size_t(src) * 3], 12);
            phases[size_t(dst)] = phases[size_t(src)];
            std::memcpy(&rest[size_t(dst) * 4], &rest[size_t(src) * 4], 16);
        }
        count = newCount;

        // the domain: particles, the active count, and the topology the asset holds now
        auto upload = [&](ParticleArray array, const void* data, size_t bytes) {
            PhysBufferHandle source = env.upload(data, bytes, ephys::particleArrayStride(array));
            CHECK(state->upload(nullptr, array, source, nullptr) == nvrhi::FS_OK);
        };
        upload(ParticleArray::Position, positions.data(), size_t(count) * 16);
        upload(ParticleArray::Velocity, velocities.data(), size_t(count) * 12);
        upload(ParticleArray::Phase, phases.data(), size_t(count) * 4);
        upload(ParticleArray::RestPosition, rest.data(), size_t(count) * 16);
        CHECK(state->setActiveCount(count) == nvrhi::FS_OK);

        const uint32_t springs = cloth->getSpringCount();
        for (uint32_t k = 0; k < springs * 2; ++k)
            inRange = inRange && cloth->getSpringIndices()[k] >= 0 && uint32_t(cloth->getSpringIndices()[k]) < count;
        for (uint32_t k = 0; k < cloth->getTriangleCount() * 3; ++k)
            inRange = inRange && cloth->getTriangleIndices()[k] >= 0 && uint32_t(cloth->getTriangleIndices()[k]) < count;
        if (numCopies || numEdits) {
            PhysBufferHandle indices = env.upload(cloth->getSpringIndices(), size_t(springs) * 8, 4);
            PhysBufferHandle lengths = env.upload(cloth->getSpringRestLengths(), size_t(springs) * 4, 4);
            PhysBufferHandle stiffness = env.upload(cloth->getSpringCoefficients(), size_t(springs) * 4, 4);
            CHECK(rig.domain->getSprings()->upload(nullptr, indices, lengths, stiffness, springs) == nvrhi::FS_OK);
            PhysBufferHandle triangles = env.upload(cloth->getTriangleIndices(), size_t(cloth->getTriangleCount()) * 12, 4);
            std::vector<float> normals(size_t(cloth->getTriangleCount()) * 3, 0.0f);
            PhysBufferHandle triangleNormals = env.upload(normals, 12);
            CHECK(rig.domain->getTriangles()->upload(nullptr, triangles, triangleNormals, cloth->getTriangleCount()) ==
                  nvrhi::FS_OK);
        }
    }

    const uint32_t count = cloth->getParticleCount();
    const std::vector<float> positions = env.fetch<float>(state->getArray(ParticleArray::Position), 0, count);
    finite = finite && allFinite(positions.data(), positions.size());
    std::printf("tearing in a domain: %u clones, %u edits, %u of %u particles in use\n", totalCopies, totalEdits, count,
                maxParticles);
    CHECK(totalCopies > 0 && totalEdits > 0);
    CHECK(count == n0 + totalCopies);
    CHECK(finite);
    CHECK(inRange);
}

void run(Env& env, D3DMessages&) {
    ephys::PhysAssetBuilderHandle builder;
    CHECK(env.phys->createAssetBuilder(&builder) == nvrhi::FS_OK);
    CHECK(env.phys->createAssetBuilder(nullptr) == nvrhi::FE_INVALID_ARGS);
    if (!builder)
        return;
    std::printf("-- well-formed assets\n");
    wellFormed(env, builder);
    std::printf("-- refusals\n");
    refusals(env, builder);
    std::printf("-- cloth in a domain\n");
    clothInDomain(env, builder);
    std::printf("-- rigid and soft bodies in a domain\n");
    rigidInDomain(env, builder);
    std::printf("-- tearing in a domain\n");
    tearingInDomain(env, builder);
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    parse(argc, argv, options);
    for (size_t i = 0; i < options.rest.size(); ++i)
        if (options.rest[i] == "--assets" && i + 1 < options.rest.size())
            g_Assets = options.rest[++i];
    if (g_Assets.empty()) {
        std::printf("usage: ethereal_phys_ext_test <api> --assets <dir> [--shaders <dir>]\n");
        return 2;
    }
    return runWithDevice(options, run);
}
