// ethereal_phys_ext_parity_test: the asset builders (IPhysAssetBuilder, ext/asset_builder.h) against a reference
// build of FleX's extension library, bit for bit.
//
//     ethereal_phys_ext_parity_test <d3d11|d3d12|vulkan> --flex-ext-dll <file> --assets <dir> [--shaders <dir>]
//
// The builders are host code: a device is created only because the builder is obtained from one. The reference
// is a DLL of the FleX checkout the build was pointed at (ETHEREAL_PHYS_FLEX_REFERENCE_DIR; by default
// bin/win64/NvFlexExtReleaseD3D_x64.dll, the extension library NVIDIA shipped), loaded at run time and called
// through GetProcAddress: nothing of FleX is linked.
//
// Every builder is called with identical inputs on both sides, procedural meshes and meshes of assets/ (read with
// ethereal_phys_utils), and every output array is compared element for element as bits: the welding maps, the
// particles, springs, shapes, triangles and inflatable values of every asset, the skinning weights and indices,
// and, for tearing, the particle clones, the mesh edits and the spring and triangle arrays after each of several
// tears of a stretched cloth. The calls run in the same order on both sides, because the builder's random
// generator (FleX: the extension module's) is seeded by the soft body builder and drawn by tearing: the tearing
// cloths are torn once before any soft body is built (the generator at zero, as in a FleX process that never
// built one) and once after.
//
// Exit code: 0 passed, 1 failed, 77 skipped (no device, or the reference DLL is missing or cannot be loaded).

#include <ethereal/phys/device.h>
#include <ethereal/phys/ext/asset_builder.h>
#include <ethereal/phys_utils/mesh.h>

#include "solver_support.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <xmmintrin.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace {

using namespace support;
using ephys::IPhysAsset;
using ephys::IPhysAssetBuilder;
using ephys::IPhysTearingCloth;

// ---- The reference --------------------------------------------------------------------------------------------
// The structs of reference/flex/include/NvFlexExt.h, repeated: that header pulls in FleX's runtime loader.

struct FlexExtAsset {
    float* particles;
    int numParticles;
    int maxParticles;
    int* springIndices;
    float* springCoefficients;
    float* springRestLengths;
    int numSprings;
    int* shapeIndices;
    int numShapeIndices;
    int* shapeOffsets;
    float* shapeCoefficients;
    float* shapeCenters;
    int numShapes;
    float* shapePlasticThresholds;
    float* shapePlasticCreeps;
    int* triangleIndices;
    int numTriangles;
    bool inflatable;
    float inflatableVolume;
    float inflatablePressure;
    float inflatableStiffness;
};

struct FlexTearingParticleClone {
    int srcIndex;
    int destIndex;
};
struct FlexTearingMeshEdit {
    int triIndex;
    int newParticleIndex;
};
static_assert(sizeof(FlexTearingParticleClone) == sizeof(ephys::TearingParticleClone));
static_assert(sizeof(FlexTearingMeshEdit) == sizeof(ephys::TearingMeshEdit));

struct FlexExtApi {
    HMODULE module = nullptr;
    int (*CreateWeldedMeshIndices)(const float*, int, int*, int*, float) = nullptr;
    FlexExtAsset* (*CreateClothFromMesh)(const float*, int, const int*, int, float, float, float, float, float) = nullptr;
    FlexExtAsset* (*CreateTearingClothFromMesh)(const float*, int, int, const int*, int, float, float, float) = nullptr;
    void (*DestroyTearingCloth)(FlexExtAsset*) = nullptr;
    void (*TearClothMesh)(FlexExtAsset*, float, int, FlexTearingParticleClone*, int*, int, FlexTearingMeshEdit*, int*,
                          int) = nullptr;
    FlexExtAsset* (*CreateRigidFromMesh)(const float*, int, const int*, int, float, float) = nullptr;
    FlexExtAsset* (*CreateSoftFromMesh)(const float*, int, const int*, int, float, float, float, float, float, float,
                                        float, float, float, float, float) = nullptr;
    void (*DestroyAsset)(FlexExtAsset*) = nullptr;
    void (*CreateSoftMeshSkinning)(const float*, int, const float*, int, float, float, float*, int*) = nullptr;

    std::string load(const std::filesystem::path& file) {
        if (!std::filesystem::exists(file))
            return "the file does not exist";
        const std::filesystem::path dll = std::filesystem::absolute(file).make_preferred();
        module = LoadLibraryExW(dll.wstring().c_str(), nullptr,
                                LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
        if (!module)
            return "LoadLibrary failed with error " + std::to_string(GetLastError());
#define RESOLVE(name)                                                                        \
    name = reinterpret_cast<decltype(name)>(GetProcAddress(module, "NvFlexExt" #name)); \
    if (!name)                                                                               \
        return std::string("the DLL does not export NvFlexExt") + #name;
        RESOLVE(CreateWeldedMeshIndices)
        RESOLVE(CreateClothFromMesh)
        RESOLVE(CreateTearingClothFromMesh)
        RESOLVE(DestroyTearingCloth)
        RESOLVE(TearClothMesh)
        RESOLVE(CreateRigidFromMesh)
        RESOLVE(CreateSoftFromMesh)
        RESOLVE(DestroyAsset)
        RESOLVE(CreateSoftMeshSkinning)
#undef RESOLVE
        return std::string();
    }
};

FlexExtApi g_Flex;

// ---- Comparison -----------------------------------------------------------------------------------------------

struct Tally {
    int cases = 0;
    uint64_t arrays = 0;
    uint64_t elements = 0;
    uint64_t mismatches = 0;
};
std::map<std::string, Tally> g_Tally;  // by builder

uint32_t bitsOf(float f) {
    uint32_t u;
    std::memcpy(&u, &f, 4);
    return u;
}

void reportValue(const float& v) { std::printf("%.9g (0x%08x)", double(v), bitsOf(v)); }
void reportValue(const int32_t& v) { std::printf("%d", v); }

template <typename T>
void compareArray(const std::string& builder, const std::string& what, const T* reference, const T* ours, size_t n) {
    Tally& tally = g_Tally[builder];
    ++tally.arrays;
    if (n == 0)
        return;
    if (!reference || !ours) {
        ++g_Failures;
        ++tally.mismatches;
        std::printf("FAILED %s: %s: an array is null (reference %p, engine %p)\n", builder.c_str(), what.c_str(),
                    static_cast<const void*>(reference), static_cast<const void*>(ours));
        return;
    }
    uint64_t mismatches = 0;
    for (size_t i = 0; i < n; ++i) {
        if (std::memcmp(&reference[i], &ours[i], sizeof(T)) != 0) {
            if (mismatches < 5) {
                std::printf("  mismatch %s: %s[%zu]: reference ", builder.c_str(), what.c_str(), i);
                reportValue(reference[i]);
                std::printf(", engine ");
                reportValue(ours[i]);
                std::printf("\n");
            }
            ++mismatches;
        }
    }
    tally.elements += n;
    tally.mismatches += mismatches;
    if (mismatches) {
        ++g_Failures;
        std::printf("FAILED %s: %s: %llu of %zu elements differ\n", builder.c_str(), what.c_str(),
                    static_cast<unsigned long long>(mismatches), n);
    }
}

template <typename T>
void compareValue(const std::string& builder, const std::string& what, T reference, T ours) {
    compareArray(builder, what, &reference, &ours, 1);
}

// particleLimit: compare no more particles than this (after a tear: the clones' slots hold what nobody wrote yet).
void compareAsset(const std::string& builder, const std::string& name, const FlexExtAsset* ref, IPhysAsset* ours,
                  int particleLimit = INT32_MAX) {
    const std::string at = name + ": ";
    if (!ref || !ours) {
        if (ref || ours) {
            ++g_Failures;
            std::printf("FAILED %s: %s: reference %s, engine %s\n", builder.c_str(), name.c_str(),
                        ref ? "built an asset" : "returned NULL", ours ? "built an asset" : "failed");
        }
        return;
    }
    compareValue(builder, at + "numParticles", int32_t(ref->numParticles), int32_t(ours->getParticleCount()));
    compareValue(builder, at + "maxParticles", int32_t(ref->maxParticles), int32_t(ours->getMaxParticles()));
    compareValue(builder, at + "numSprings", int32_t(ref->numSprings), int32_t(ours->getSpringCount()));
    compareValue(builder, at + "numShapeIndices", int32_t(ref->numShapeIndices), int32_t(ours->getShapeIndexCount()));
    compareValue(builder, at + "numShapes", int32_t(ref->numShapes), int32_t(ours->getShapeCount()));
    compareValue(builder, at + "numTriangles", int32_t(ref->numTriangles), int32_t(ours->getTriangleCount()));
    compareValue(builder, at + "inflatable", int32_t(ref->inflatable), int32_t(ours->isInflatable()));
    compareValue(builder, at + "inflatableVolume", ref->inflatableVolume, ours->getInflatableVolume());
    compareValue(builder, at + "inflatablePressure", ref->inflatablePressure, ours->getInflatablePressure());
    compareValue(builder, at + "inflatableStiffness", ref->inflatableStiffness, ours->getInflatableStiffness());

    const size_t particles =
        size_t(std::min<int>(std::min<int>(ref->numParticles, int(ours->getParticleCount())), particleLimit));
    const size_t springs = size_t(std::min<int>(ref->numSprings, int(ours->getSpringCount())));
    const size_t shapes = size_t(std::min<int>(ref->numShapes, int(ours->getShapeCount())));
    const size_t shapeIndices = size_t(std::min<int>(ref->numShapeIndices, int(ours->getShapeIndexCount())));
    const size_t triangles = size_t(std::min<int>(ref->numTriangles, int(ours->getTriangleCount())));

    compareArray(builder, at + "particles", ref->particles, ours->getParticles(), particles * 4);
    compareArray(builder, at + "springIndices", ref->springIndices, ours->getSpringIndices(), springs * 2);
    compareArray(builder, at + "springCoefficients", ref->springCoefficients, ours->getSpringCoefficients(), springs);
    compareArray(builder, at + "springRestLengths", ref->springRestLengths, ours->getSpringRestLengths(), springs);
    compareArray(builder, at + "shapeIndices", ref->shapeIndices, ours->getShapeIndices(), shapeIndices);
    compareArray(builder, at + "shapeOffsets", ref->shapeOffsets, ours->getShapeOffsets(), shapes);
    compareArray(builder, at + "shapeCoefficients", ref->shapeCoefficients, ours->getShapeCoefficients(), shapes);
    compareArray(builder, at + "shapeCenters", ref->shapeCenters, ours->getShapeCenters(), shapes * 3);
    compareValue(builder, at + "has plastic thresholds", int32_t(ref->shapePlasticThresholds != nullptr),
                 int32_t(ours->getShapePlasticThresholds() != nullptr));
    compareValue(builder, at + "has plastic creeps", int32_t(ref->shapePlasticCreeps != nullptr),
                 int32_t(ours->getShapePlasticCreeps() != nullptr));
    if (ref->shapePlasticThresholds && ours->getShapePlasticThresholds())
        compareArray(builder, at + "shapePlasticThresholds", ref->shapePlasticThresholds,
                     ours->getShapePlasticThresholds(), shapes);
    if (ref->shapePlasticCreeps && ours->getShapePlasticCreeps())
        compareArray(builder, at + "shapePlasticCreeps", ref->shapePlasticCreeps, ours->getShapePlasticCreeps(), shapes);
    compareArray(builder, at + "triangleIndices", ref->triangleIndices, ours->getTriangleIndices(), triangles * 3);
}

// ---- Meshes ---------------------------------------------------------------------------------------------------

struct MeshData {
    std::string name;
    std::vector<float> vertices;  // 3 per vertex
    std::vector<int32_t> indices;

    int numVertices() const { return int(vertices.size() / 3); }
    int numIndices() const { return int(indices.size()); }
    int numTriangles() const { return int(indices.size() / 3); }
};

std::filesystem::path g_Assets;

// An asset mesh, normalised as the demo does (Mesh::Normalize), scaled and translated.
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

// A sheet of nx x nz quads in the x z plane at height y, with shared vertices.
MeshData gridMesh(int nx, int nz, float spacing, float y) {
    MeshData data;
    data.name = "grid " + std::to_string(nx) + "x" + std::to_string(nz);
    for (int z = 0; z <= nz; ++z)
        for (int x = 0; x <= nx; ++x)
            data.vertices.insert(data.vertices.end(), {float(x) * spacing, y, float(z) * spacing});
    auto index = [&](int x, int z) { return int32_t(z * (nx + 1) + x); };
    for (int z = 0; z < nz; ++z)
        for (int x = 0; x < nx; ++x) {
            // alternate the diagonals
            if ((x + z) & 1) {
                data.indices.insert(data.indices.end(), {index(x, z), index(x + 1, z), index(x + 1, z + 1)});
                data.indices.insert(data.indices.end(), {index(x, z), index(x + 1, z + 1), index(x, z + 1)});
            } else {
                data.indices.insert(data.indices.end(), {index(x, z), index(x + 1, z), index(x, z + 1)});
                data.indices.insert(data.indices.end(), {index(x + 1, z), index(x + 1, z + 1), index(x, z + 1)});
            }
        }
    return data;
}

// Every quad with vertices of its own, jittered below `jitter`: a render mesh to weld.
MeshData splitGridMesh(int nx, int nz, float spacing, float jitter) {
    MeshData data;
    data.name = "split grid";
    scenes::Jitter random(7);
    for (int z = 0; z < nz; ++z)
        for (int x = 0; x < nx; ++x) {
            const int32_t base = int32_t(data.vertices.size() / 3);
            const float corners[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
            for (const auto& c : corners)
                data.vertices.insert(data.vertices.end(),
                                     {(float(x) + c[0]) * spacing + random.next() * jitter, random.next() * jitter,
                                      (float(z) + c[1]) * spacing + random.next() * jitter});
            data.indices.insert(data.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
        }
    return data;
}

// The mesh turned upright, as the demo's tearing scene turns irregular_plane.obj: a rotation by 90 degrees about x
// (z becomes y), lifted by `lift`.
MeshData upright(MeshData mesh, float lift) {
    for (size_t i = 0; i < mesh.vertices.size(); i += 3) {
        const float y = mesh.vertices[i + 1], z = mesh.vertices[i + 2];
        mesh.vertices[i + 1] = z + lift;
        mesh.vertices[i + 2] = -y;
    }
    mesh.name += " upright";
    return mesh;
}

// The mesh with welded vertices (the engine's builder, which the welding cases compare first).
MeshData weld(IPhysAssetBuilder* builder, const MeshData& mesh, float threshold) {
    const uint32_t n = uint32_t(mesh.numVertices());
    std::vector<int32_t> unique(n), remap(n);
    uint32_t count = 0;
    CHECK(builder->createWeldedMeshIndices(mesh.vertices.data(), n, unique.data(), remap.data(), threshold, count) ==
          nvrhi::FS_OK);
    MeshData welded;
    welded.name = mesh.name + " welded";
    for (uint32_t i = 0; i < count; ++i)
        welded.vertices.insert(welded.vertices.end(), &mesh.vertices[size_t(unique[i]) * 3],
                               &mesh.vertices[size_t(unique[i]) * 3] + 3);
    for (int32_t index : mesh.indices)
        welded.indices.push_back(remap[size_t(index)]);
    return welded;
}

// x y z 1/mass: the particles of a mesh, fixed (1/mass 0) where pinned(x, y, z).
template <typename F>
std::vector<float> particlesOf(const MeshData& mesh, F pinned) {
    std::vector<float> particles;
    for (int i = 0; i < mesh.numVertices(); ++i) {
        const float* p = &mesh.vertices[size_t(i) * 3];
        particles.insert(particles.end(), {p[0], p[1], p[2], pinned(p[0], p[1], p[2]) ? 0.0f : 1.0f});
    }
    return particles;
}

float maxY(const MeshData& mesh) {
    float y = -FLT_MAX;
    for (int i = 0; i < mesh.numVertices(); ++i)
        y = std::max(y, mesh.vertices[size_t(i) * 3 + 1]);
    return y;
}

// ---- Skinning weights ------------------------------------------------------------------------------------------
// NvFlexExtCreateSoftMeshSkinning normalises a vertex's four weights by dividing them by their sum. The extension
// library NVIDIA shipped was compiled otherwise (its CreateSkinning, disassembled: rcpps of the sum, one Newton-
// Raphson step 2r - r * r * sum, then a multiplication), and rcpps is an approximation whose bits are the CPU's;
// a build of the FleX checkout's source divides, as the engine does. A vertex whose weights differ is accepted
// only when both sides are explained exactly from the same unnormalised weights, recomputed here as FleX's source
// computes them: the engine's by the division, the reference's by the instructions above (on this CPU).

std::map<std::string, uint64_t> g_RcppsExplained;  // by builder

// FleX's unnormalised weights of vertex v for the bones the reference chose (FLT_MAX distance for -1).
void unnormalisedWeights(const float* v, const float* bones, const int32_t* chosen, float falloff, float maxDistance,
                         float u[4], float& sum) {
    sum = 0.0f;
    for (int k = 0; k < 4; ++k) {
        float dSq = FLT_MAX;
        if (chosen[k] >= 0) {
            const float* b = &bones[size_t(chosen[k]) * 3];
            const float dx = v[0] - b[0], dy = v[1] - b[1], dz = v[2] - b[2];
            dSq = dx * dx + dy * dy + dz * dz;
        }
        u[k] = dSq > maxDistance * maxDistance ? 0.0f : 1.0f / (powf(dSq, falloff) + 0.0001f);
        sum += u[k];
    }
}

void compareSkinningWeights(const std::string& name, const MeshData& mesh, const float* bones, float falloff,
                            float maxDistance, const std::vector<int32_t>& chosen, const std::vector<float>& reference,
                            const std::vector<float>& ours) {
    Tally& tally = g_Tally["skinning"];
    ++tally.arrays;
    const size_t n = size_t(mesh.numVertices());
    uint64_t explained = 0, unexplained = 0;
    for (size_t i = 0; i < n; ++i) {
        if (std::memcmp(&reference[i * 4], &ours[i * 4], 16) == 0)
            continue;
        float u[4], sum;
        unnormalisedWeights(&mesh.vertices[i * 3], bones, &chosen[i * 4], falloff, maxDistance, u, sum);
        bool engine = sum != 0.0f, shipped = sum != 0.0f;  // with no weight both give (1, 0, 0, 0)
        if (sum != 0.0f) {
            const __m128 s = _mm_set1_ps(sum);
            const __m128 r = _mm_rcp_ps(s);
            const __m128 refined = _mm_sub_ps(_mm_add_ps(r, r), _mm_mul_ps(_mm_mul_ps(r, r), s));
            float viaRcp[4];
            _mm_storeu_ps(viaRcp, _mm_mul_ps(refined, _mm_loadu_ps(u)));
            for (int k = 0; k < 4; ++k) {
                const float divided = u[k] / sum;
                engine = engine && std::memcmp(&divided, &ours[i * 4 + size_t(k)], 4) == 0;
                shipped = shipped && std::memcmp(&viaRcp[k], &reference[i * 4 + size_t(k)], 4) == 0;
            }
        }
        if (engine && shipped) {
            ++explained;
        } else {
            if (unexplained < 5)
                std::printf("  mismatch skinning: %s: weights of vertex %zu: reference %.9g %.9g %.9g %.9g, engine %.9g "
                            "%.9g %.9g %.9g\n",
                            name.c_str(), i, double(reference[i * 4]), double(reference[i * 4 + 1]),
                            double(reference[i * 4 + 2]), double(reference[i * 4 + 3]), double(ours[i * 4]),
                            double(ours[i * 4 + 1]), double(ours[i * 4 + 2]), double(ours[i * 4 + 3]));
            ++unexplained;
        }
    }
    tally.elements += n * 4;
    tally.mismatches += unexplained;
    g_RcppsExplained["skinning"] += explained;
    if (explained)
        std::printf("  skinning %s: %llu of %zu vertices differ only by the reference's rcpps normalisation\n",
                    name.c_str(), static_cast<unsigned long long>(explained), n);
    if (unexplained) {
        ++g_Failures;
        std::printf("FAILED skinning: %s: weights: %llu of %zu vertices differ unexplained\n", name.c_str(),
                    static_cast<unsigned long long>(unexplained), n);
    }
}

// ---- The cases --------------------------------------------------------------------------------------------------

void weldCase(IPhysAssetBuilder* builder, const MeshData& mesh, float threshold) {
    const std::string name = mesh.name + " @" + std::to_string(threshold);
    const int n = mesh.numVertices();
    std::vector<int32_t> refUnique(size_t(n), -7), refMap(size_t(n), -7), unique(size_t(n), -7), map(size_t(n), -7);
    const int refCount = g_Flex.CreateWeldedMeshIndices(mesh.vertices.data(), n, refUnique.data(), refMap.data(), threshold);
    uint32_t count = 0;
    CHECK(builder->createWeldedMeshIndices(mesh.vertices.data(), uint32_t(n), unique.data(), map.data(), threshold,
                                           count) == nvrhi::FS_OK);
    ++g_Tally["weld"].cases;
    compareValue("weld", name + ": count", int32_t(refCount), int32_t(count));
    compareArray("weld", name + ": uniqueVerts", refUnique.data(), unique.data(), size_t(std::min<int>(refCount, int(count))));
    compareArray("weld", name + ": originalToUniqueMap", refMap.data(), map.data(), size_t(n));
    std::printf("weld %s: %d vertices -> %u\n", name.c_str(), n, count);
}

void clothCase(IPhysAssetBuilder* builder, const std::string& name, const std::vector<float>& particles,
               const MeshData& mesh, const ephys::ClothAssetParams& params) {
    const int n = int(particles.size() / 4);
    FlexExtAsset* ref = g_Flex.CreateClothFromMesh(particles.data(), n, mesh.indices.data(), mesh.numTriangles(),
                                                   params.stretchStiffness, params.bendStiffness, params.tetherStiffness,
                                                   params.tetherGive, params.pressure);
    ephys::PhysAssetHandle ours;
    const nvrhi::FRESULT result = builder->createClothFromMesh(particles.data(), uint32_t(n), mesh.indices.data(),
                                                               uint32_t(mesh.numTriangles()), params, &ours);
    CHECK((result == nvrhi::FS_OK) == (ours != nullptr));
    ++g_Tally["cloth"].cases;
    compareAsset("cloth", name, ref, ours);
    std::printf("cloth %s: %s, %d particles, %u springs%s\n", name.c_str(), ref ? "built" : "rejected", n,
                ours ? ours->getSpringCount() : 0u, ref && ref->inflatable ? ", inflatable" : "");
    if (ref)
        g_Flex.DestroyAsset(ref);
}

void rigidCase(IPhysAssetBuilder* builder, const MeshData& mesh, float spacing, float expand) {
    const std::string name = mesh.name + " @" + std::to_string(spacing);
    FlexExtAsset* ref = g_Flex.CreateRigidFromMesh(mesh.vertices.data(), mesh.numVertices(), mesh.indices.data(),
                                                   mesh.numIndices(), spacing, expand);
    ephys::PhysAssetHandle ours;
    const nvrhi::FRESULT result =
        builder->createRigidFromMesh(mesh.vertices.data(), uint32_t(mesh.numVertices()), mesh.indices.data(),
                                     uint32_t(mesh.numIndices()), spacing, expand, &ours);
    CHECK((result == nvrhi::FS_OK) == (ours != nullptr));
    ++g_Tally["rigid"].cases;
    compareAsset("rigid", name, ref, ours);
    std::printf("rigid %s: %s, %d particles\n", name.c_str(), ref ? "built" : "rejected", ref ? ref->numParticles : 0);
    if (ref)
        g_Flex.DestroyAsset(ref);
}

// A soft body, then the skinning of its mesh to its clusters.
void softCase(IPhysAssetBuilder* builder, const MeshData& mesh, const ephys::SoftAssetParams& p, float falloff,
              float maxDistance) {
    const std::string& name = mesh.name;
    FlexExtAsset* ref = g_Flex.CreateSoftFromMesh(
        mesh.vertices.data(), mesh.numVertices(), mesh.indices.data(), mesh.numIndices(), p.particleSpacing,
        p.volumeSampling, p.surfaceSampling, p.clusterSpacing, p.clusterRadius, p.clusterStiffness, p.linkRadius,
        p.linkStiffness, p.globalStiffness, p.clusterPlasticThreshold, p.clusterPlasticCreep);
    ephys::PhysAssetHandle ours;
    const nvrhi::FRESULT result = builder->createSoftFromMesh(mesh.vertices.data(), uint32_t(mesh.numVertices()),
                                                              mesh.indices.data(), uint32_t(mesh.numIndices()), p, &ours);
    CHECK((result == nvrhi::FS_OK) == (ours != nullptr));
    ++g_Tally["soft"].cases;
    compareAsset("soft", name, ref, ours);
    std::printf("soft %s: %d particles, %d clusters, %d links\n", name.c_str(), ref ? ref->numParticles : 0,
                ref ? ref->numShapes : 0, ref ? ref->numSprings : 0);

    // the skinning of the mesh to the clusters of the reference's asset (identical when the above passed)
    if (ref && ref->numShapes) {
        const size_t n = size_t(mesh.numVertices());
        std::vector<float> refWeights(n * 4, -1.0f), weights(n * 4, -1.0f);
        std::vector<int32_t> refIndices(n * 4, -7), indices(n * 4, -7);
        g_Flex.CreateSoftMeshSkinning(mesh.vertices.data(), int(n), ref->shapeCenters, ref->numShapes, falloff,
                                      maxDistance, refWeights.data(), refIndices.data());
        CHECK(builder->createSoftMeshSkinning(mesh.vertices.data(), uint32_t(n), ref->shapeCenters,
                                              uint32_t(ref->numShapes), falloff, maxDistance, weights.data(),
                                              indices.data()) == nvrhi::FS_OK);
        ++g_Tally["skinning"].cases;
        compareArray("skinning", name + ": indices", refIndices.data(), indices.data(), n * 4);
        compareSkinningWeights(name, mesh, ref->shapeCenters, falloff, maxDistance, refIndices, refWeights, weights);
    }
    if (ref)
        g_Flex.DestroyAsset(ref);
}

// A tearing cloth stretched further in each round and torn on both sides; the clones are applied as the demo
// does (demo/scenes/tearing.h), to one particle array both sides read.
void tearingCase(IPhysAssetBuilder* builder, const std::string& name, const MeshData& mesh, int extraParticles,
                 int rounds) {
    const std::string b = "tearing";
    const float top = maxY(mesh);
    std::vector<float> particles = particlesOf(mesh, [&](float, float y, float) { return y == top; });
    const int n = mesh.numVertices();
    const int maxParticles = n + extraParticles;

    FlexExtAsset* ref = g_Flex.CreateTearingClothFromMesh(particles.data(), n, maxParticles, mesh.indices.data(),
                                                          mesh.numTriangles(), 0.8f, 0.8f, 0.0f);
    ephys::TearingClothAssetParams params;
    params.stretchStiffness = 0.8f;
    params.bendStiffness = 0.8f;
    ephys::PhysTearingClothHandle ours;
    CHECK(builder->createTearingClothFromMesh(particles.data(), uint32_t(n), uint32_t(maxParticles), mesh.indices.data(),
                                              uint32_t(mesh.numTriangles()), params, &ours) == nvrhi::FS_OK);
    ++g_Tally[b].cases;
    compareAsset(b, name + " (built)", ref, ours);
    if (!ref || !ours) {
        if (ref)
            g_Flex.DestroyTearingCloth(ref);
        return;
    }

    // the rest positions, which the stretching starts from each round, and the particles both sides read
    std::vector<float> rest(size_t(maxParticles) * 4, 0.0f);
    std::copy(particles.begin(), particles.end(), rest.begin());
    particles.resize(size_t(maxParticles) * 4, 0.0f);
    int count = n;

    int totalCopies = 0, totalEdits = 0;
    for (int round = 0; round < rounds; ++round) {
        // stretched downwards from the pinned top, more each round, with a sideways wave
        const float stretch = 1.0f + 0.35f * float(round + 1);
        for (int i = 0; i < count; ++i) {
            float* p = &particles[size_t(i) * 4];
            const float* r = &rest[size_t(i) * 4];
            if (p[3] == 0.0f)
                continue;
            p[0] = r[0] * (1.0f + 0.1f * std::sin(3.0f * r[1] + float(round)));
            p[1] = top - (top - r[1]) * stretch;
            p[2] = r[2];
        }

        std::memcpy(ref->particles, particles.data(), size_t(count) * 16);
        CHECK(ours->setParticles(particles.data(), 0, uint32_t(count)) == nvrhi::FS_OK);

        // NvFlexExtTearClothMesh measures the particles it cloned earlier in the same call at what the asset's
        // copy holds for their index, which nothing has written: uninitialised memory in FleX (new[] without
        // initialisation), zero in the engine. The reference gets the engine's zeros, so that both read the
        // same input.
        std::memset(ref->particles + size_t(count) * 4, 0, size_t(maxParticles - count) * 16);

        // round 3 reports fewer edits than happen, round 5 allows few copies (both: FleX's truncation)
        const int maxCopies = round == 5 ? 3 : 2048;
        const int maxEdits = round == 3 ? 8 : 2048;
        const int maxSplits = 4;
        const float maxStrain = 1.5f;

        std::vector<FlexTearingParticleClone> refCopies(size_t(maxCopies), {-7, -7});
        std::vector<FlexTearingMeshEdit> refEdits(size_t(maxEdits), {-7, -7});
        int refNumCopies = -1, refNumEdits = -1;
        g_Flex.TearClothMesh(ref, maxStrain, maxSplits, refCopies.data(), &refNumCopies, maxCopies, refEdits.data(),
                             &refNumEdits, maxEdits);

        std::vector<ephys::TearingParticleClone> copies(size_t(maxCopies), {-7, -7});
        std::vector<ephys::TearingMeshEdit> edits(size_t(maxEdits), {-7, -7});
        uint32_t numCopies = 0, numEdits = 0;
        CHECK(ours->tearClothMesh(maxStrain, uint32_t(maxSplits), copies.data(), uint32_t(maxCopies), numCopies,
                                  edits.data(), uint32_t(maxEdits), numEdits) == nvrhi::FS_OK);

        const std::string at = name + " round " + std::to_string(round) + ": ";
        compareValue(b, at + "numParticleCopies", int32_t(refNumCopies), int32_t(numCopies));
        compareValue(b, at + "numTriangleEdits", int32_t(refNumEdits), int32_t(numEdits));
        const size_t c = size_t(std::min<int>(refNumCopies, int(numCopies)));
        const size_t e = size_t(std::min<int>(refNumEdits, int(numEdits)));
        compareArray(b, at + "particleCopies", reinterpret_cast<const int32_t*>(refCopies.data()),
                     reinterpret_cast<const int32_t*>(copies.data()), c * 2);
        compareArray(b, at + "triangleEdits", reinterpret_cast<const int32_t*>(refEdits.data()),
                     reinterpret_cast<const int32_t*>(edits.data()), e * 2);
        compareAsset(b, at + "after", ref, ours, count);
        totalCopies += refNumCopies;
        totalEdits += refNumEdits;

        // the clones, as the demo copies them (with mass 1: a picked particle may have had none)
        for (size_t k = 0; k < c; ++k) {
            const int src = copies[k].srcIndex, dst = copies[k].destIndex;
            std::memcpy(&particles[size_t(dst) * 4], &particles[size_t(src) * 4], 12);
            particles[size_t(dst) * 4 + 3] = 1.0f;
            std::memcpy(&rest[size_t(dst) * 4], &rest[size_t(src) * 4], 16);
        }
        count = int(ours->getParticleCount());
    }
    std::printf("tearing %s: %d rounds, %d clones, %d edits, %d particles at the end\n", name.c_str(), rounds,
                totalCopies, totalEdits, count);
    CHECK(totalCopies > 0);
    g_Flex.DestroyTearingCloth(ref);
}

void run(Env& env, D3DMessages&) {
    ephys::PhysAssetBuilderHandle builder;
    CHECK(env.phys->createAssetBuilder(&builder) == nvrhi::FS_OK);
    if (!builder)
        return;

    // 1. Welding.
    weldCase(builder, splitGridMesh(20, 20, 0.05f, 1e-4f), 1e-3f);
    weldCase(builder, loadMesh("box_high.ply", dm::float3(1.0f)), 1e-5f);
    weldCase(builder, loadMesh("sphere_high.ply", dm::float3(1.0f)), 1e-5f);
    weldCase(builder, loadMesh("irregular_plane.obj", dm::float3(2.0f)), 1e-5f);
    weldCase(builder, loadMesh("bunny.ply", dm::float3(1.0f)), 0.01f);
    weldCase(builder, loadMesh("teapot.ply", dm::float3(1.0f)), 1e-4f);
    weldCase(builder, loadMesh("dragon.obj", dm::float3(1.0f)), 1e-5f);

    // 2. Cloth, with tethers, give and pressure.
    {
        const MeshData grid = gridMesh(32, 32, 0.05f, 1.5f);
        ephys::ClothAssetParams params;
        params.stretchStiffness = 0.9f;
        params.bendStiffness = 0.8f;
        params.tetherStiffness = 0.6f;
        params.tetherGive = 0.05f;
        clothCase(builder, "grid, tethers", particlesOf(grid, [](float, float, float z) { return z == 0.0f; }), grid,
                  params);

        const MeshData plane = upright(weld(builder, loadMesh("irregular_plane.obj", dm::float3(2.0f)), 1e-5f), 1.5f);
        const float top = maxY(plane);
        params = ephys::ClothAssetParams();
        params.tetherStiffness = 1.0f;
        clothCase(builder, "irregular_plane, tethers", particlesOf(plane, [&](float, float y, float) { return y == top; }),
                  plane, params);

        const MeshData sphere = weld(builder, loadMesh("sphere.ply", dm::float3(1.0f)), 1e-5f);
        params = ephys::ClothAssetParams();
        params.bendStiffness = 0.5f;
        params.pressure = 1.5f;
        clothCase(builder, "sphere, pressure", particlesOf(sphere, [](float, float, float) { return false; }), sphere,
                  params);

        const MeshData box = weld(builder, loadMesh("box_high.ply", dm::float3(0.5f)), 1e-5f);
        params = ephys::ClothAssetParams();
        params.pressure = 1.0f;
        params.bendStiffness = 0.0f;
        clothCase(builder, "box_high, pressure, no bending",
                  particlesOf(box, [](float, float, float) { return false; }), box, params);

        // three triangles on one edge: both reject it
        MeshData fan;
        fan.name = "non-manifold";
        fan.vertices = {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 1};
        fan.indices = {0, 1, 2, 1, 0, 3, 0, 1, 4};
        clothCase(builder, "non-manifold", particlesOf(fan, [](float, float, float) { return false; }), fan,
                  ephys::ClothAssetParams());
    }

    // 3. Tearing, the random generator at zero.
    tearingCase(builder, "grid 24 (generator at zero)", upright(gridMesh(24, 24, 0.05f, 0.0f), 1.5f), 1000, 8);
    tearingCase(builder, "irregular_plane (generator at zero)",
                upright(weld(builder, loadMesh("irregular_plane.obj", dm::float3(2.0f)), 1e-5f), 1.5f), 1000, 8);

    // 4. Rigid bodies.
    rigidCase(builder, loadMesh("box.ply", dm::float3(1.0f)), 0.05f, -0.025f);
    rigidCase(builder, loadMesh("sphere.ply", dm::float3(1.0f)), 0.05f, -0.025f);
    rigidCase(builder, loadMesh("bunny.ply", dm::float3(1.0f)), 0.04f, -0.02f);
    rigidCase(builder, loadMesh("banana.obj", dm::float3(1.0f)), 0.05f, 0.0f);
    rigidCase(builder, loadMesh("torus.obj", dm::float3(1.0f)), 0.03f, -0.01f);
    rigidCase(builder, loadMesh("rocka.ply", dm::float3(0.4f)), 0.02f, -0.01f);
    rigidCase(builder, loadMesh("armadillo.ply", dm::float3(1.0f)), 0.025f, -0.0125f);
    rigidCase(builder, loadMesh("box.ply", dm::float3(1.0f)), 0.01f, 0.0f);  // > 64 voxels: both reject

    // 5. Soft bodies (the demo's instances, demo/main.cpp, at its radius 0.1) and their skinning.
    {
        ephys::SoftAssetParams p;
        p.particleSpacing = 0.1f;
        p.volumeSampling = 4.0f;
        p.clusterSpacing = 0.35f;
        p.clusterRadius = 0.0f;
        p.clusterStiffness = 0.2f;
        softCase(builder, loadMesh("bunny.ply", dm::float3(2.0f)), p, 2.0f, 100.0f);

        p = ephys::SoftAssetParams();
        p.clusterSpacing = 0.275f;
        p.clusterRadius = 0.3f;
        p.clusterStiffness = 0.15f;
        p.surfaceSampling = 1.0f;  // seeds the generator and draws 150000 numbers
        softCase(builder, loadMesh("softs/octopus.obj", dm::float3(3.2f)), p, 2.0f, 100.0f);

        p = ephys::SoftAssetParams();
        p.clusterSpacing = 0.1f;
        p.clusterRadius = 0.0f;
        p.clusterStiffness = 0.0f;
        p.globalStiffness = 1.0f;
        p.clusterPlasticThreshold = 0.0015f;
        p.clusterPlasticCreep = 0.15f;
        softCase(builder, loadMesh("bunny.ply", dm::float3(1.0f)), p, 2.0f, 100.0f);

        p = ephys::SoftAssetParams();
        p.particleSpacing = 0.05f;
        p.clusterSpacing = 0.05f;
        p.clusterRadius = 0.1f;
        p.clusterStiffness = 0.2f;
        p.linkRadius = 0.1f;
        p.linkStiffness = 1.0f;
        softCase(builder, loadMesh("box_very_high.ply", dm::float3(1.0f, 0.02f, 1.0f)), p, 1.0f, 100.0f);

        p = ephys::SoftAssetParams();
        p.clusterSpacing = 0.15f;
        p.clusterStiffness = 0.55f;
        softCase(builder, loadMesh("rope.obj", dm::float3(5.0f)), p, 2.0f, 1.0f);
    }

    // 6. Tearing again, the generator seeded by the octopus.
    tearingCase(builder, "grid 24 (generator seeded)", upright(gridMesh(24, 24, 0.05f, 0.0f), 1.5f), 1000, 8);
    tearingCase(builder, "grid 40 (generator seeded)", upright(gridMesh(40, 30, 0.04f, 0.0f), 1.5f), 2000, 10);

    std::printf("\n%-9s %6s %8s %12s %11s %11s\n", "builder", "cases", "arrays", "elements", "mismatches",
                "rcpps-only");
    for (const auto& [name, tally] : g_Tally)
        std::printf("%-9s %6d %8llu %12llu %11llu %11llu\n", name.c_str(), tally.cases,
                    static_cast<unsigned long long>(tally.arrays), static_cast<unsigned long long>(tally.elements),
                    static_cast<unsigned long long>(tally.mismatches),
                    static_cast<unsigned long long>(g_RcppsExplained[name]));
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    parse(argc, argv, options);

    std::filesystem::path dll;
    for (size_t i = 0; i < options.rest.size(); ++i) {
        if (options.rest[i] == "--flex-ext-dll" && i + 1 < options.rest.size())
            dll = options.rest[++i];
        else if (options.rest[i] == "--assets" && i + 1 < options.rest.size())
            g_Assets = options.rest[++i];
    }
    if (dll.empty() || g_Assets.empty()) {
        std::printf("usage: ethereal_phys_ext_parity_test <api> --flex-ext-dll <file> --assets <dir>\n");
        return 2;
    }
    const std::string problem = g_Flex.load(dll);
    if (!problem.empty()) {
        std::printf("SKIPPED: the reference %s: %s\n", dll.string().c_str(), problem.c_str());
        return 77;
    }
    std::printf("reference: %s\n", dll.string().c_str());

    return runWithDevice(options, run);
}
