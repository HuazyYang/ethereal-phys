// ethereal_phys_helpers_test: the samples' scene-authoring library (ethereal_phys_helpers, samples/helpers) and the
// voxeliser and distance field ethereal_phys_utils gained for it, without a device.
//
//     ethereal_phys_helpers_test --assets <dir>
//
// - Voxelize and MakeSDF: a cube voxelised on a grid twice its size fills exactly the voxels whose centres it
//   contains; the distance field is negative inside, positive outside and rises from the centre to the faces.
// - The random generator is FleX's (the first values of a sequence computed here from its recurrence), restarts
//   with RandInit, and the sampling functions keep their contracts (unit vectors, points in the ball, Poisson
//   separation, the packing inside the ball).
// - The authoring functions give the counts FleX's demo shows for the same calls: a 64 x 32 spring grid has 3906
//   triangles and 11810 springs (FleX's Flag Cloth: 15902 springs with its 4092 tethers); CreateParticleShape fills
//   the GranularPile sphere with 1237 particles and its sand castle with 41084 (FleX's demo: 42321 together), a
//   RigidPile box with 8 and a FluidBlock torus with 28 (FleX's demo: 8000 particles for 1000 boxes, 896 for 32
//   tori); rigid offsets, rest normals and skinning data are consistent; the skinning of an unmoved rigid leaves its
//   mesh where it is.
// - AddRandomConvex: its planes, their bounds and a closed flat-shaded render mesh; GetShapeBounds of a box and a
//   rotated box; PickParticle and the rigid centres of mass.
//
// Exit code: 0 passed, 1 failed.

#include <ethereal/phys_helpers/helpers.h>
#include <ethereal/phys_helpers/random.h>
#include <ethereal/phys_utils/mesh.h>
#include <ethereal/phys_utils/voxelize.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <vector>

namespace {

int g_Failures = 0;

#define CHECK(expr)                                                        \
    do {                                                                   \
        if (!(expr)) {                                                     \
            ++g_Failures;                                                  \
            std::printf("FAILED %s(%d): %s\n", __FILE__, __LINE__, #expr); \
        }                                                                  \
    } while (false)

using namespace ephys;
using namespace ephys::helpers;

std::filesystem::path g_Assets;

dm::float3 xyz(const dm::float4& v) { return dm::float3(v.x, v.y, v.z); }

void voxels() {
    std::printf("voxelize, sdf\n");
    std::unique_ptr<utils::Mesh> cube = utils::CreateCubeMesh();  // [-0.5, 0.5]^3
    const uint32_t n = 8;                                           // over [-1, 1]^3: voxels of 0.25
    std::vector<uint32_t> volume(n * n * n, 7);
    std::vector<int> indices(cube->m_indices.begin(), cube->m_indices.end());
    utils::Voxelize(cube->m_positions.data(), int(cube->m_positions.size()), indices.data(), int(indices.size()), n,
                    n, n, volume.data(), dm::float3(-1.0f), dm::float3(1.0f));

    int filled = 0, wrong = 0;
    for (uint32_t z = 0; z < n; ++z)
        for (uint32_t y = 0; y < n; ++y)
            for (uint32_t x = 0; x < n; ++x) {
                const bool inside = x >= 2 && x <= 5 && y >= 2 && y <= 5 && z >= 2 && z <= 5;
                const uint32_t v = volume[z * n * n + y * n + x];
                filled += v != 0;
                wrong += (v != 0) != inside || (v != 0 && v != uint32_t(-1));
            }
    CHECK(filled == 64);
    CHECK(wrong == 0);

    std::vector<float> sdf(n * n * n);
    utils::MakeSDF(volume.data(), n, n, n, sdf.data());
    auto at = [&](uint32_t x, uint32_t y, uint32_t z) { return sdf[z * n * n + y * n + x]; };
    CHECK(at(3, 3, 3) < 0.0f && at(4, 4, 4) < 0.0f);
    CHECK(at(0, 0, 0) > 0.0f && at(7, 7, 7) > 0.0f);
    CHECK(at(3, 3, 3) < at(2, 3, 3));  // deeper towards the centre
    CHECK(at(0, 3, 3) > at(1, 3, 3));  // further outside away from the face
}

void random() {
    std::printf("random\n");
    // FleX's maths.cpp / maths.h, computed here
    uint32_t s1 = 315645664;
    uint32_t s2 = s1 ^ 0x13ab45fe;
    uint32_t expected[4];
    for (uint32_t& e : expected) {
        s1 = (s2 ^ ((s1 << 5) | (s1 >> 27))) ^ (s1 * s2);
        s2 = s1 ^ ((s2 << 12) | (s2 >> 20));
        e = s1;
    }
    RandInit();
    for (uint32_t e : expected)
        CHECK(Rand() == e);

    RandInit();
    const float a = Randf();
    const dm::float3 u = RandomUnitVector();
    RandInit();
    CHECK(Randf() == a);
    CHECK(dm::all(RandomUnitVector() == u));
    CHECK(std::fabs(dm::length(u) - 1.0f) < 1e-5f);

    for (int i = 0; i < 200; ++i) {
        const float f = Randf(-2.0f, 3.0f);
        CHECK(f >= -2.0f && f <= 3.0f);
        CHECK(dm::length(UniformSampleSphereVolume()) < 1.0f);
        CHECK(std::fabs(dm::length(UniformSampleSphere()) - 1.0f) < 1e-4f);
        const dm::float3 h = UniformSampleHemisphere();
        CHECK(h.z >= 0.0f && std::fabs(dm::length(h) - 1.0f) < 1e-4f);
        CHECK(dm::length(UniformSampleDisc()) <= 1.0f + 1e-6f);
    }

    std::vector<dm::float3> points(200);
    const int poisson = PoissonSample3D(1.0f, 0.2f, points.data(), int(points.size()), 50);
    CHECK(poisson > 20);
    bool separated = true, inside = true;
    for (int i = 0; i < poisson; ++i) {
        inside = inside && dm::length(points[i]) <= 1.0f;
        for (int j = 0; j < i; ++j)
            separated = separated && dm::length(points[i] - points[j]) >= 0.2f;
    }
    CHECK(separated && inside);

    const int packed = TightPack3D(1.0f, 0.25f, points.data(), int(points.size()));
    CHECK(packed > 100);
    bool packedInside = true, packedApart = true;
    for (int i = 0; i < packed; ++i) {
        packedInside = packedInside && dm::length(points[i]) <= 1.0f && dm::dot(points[i], points[i]) > 0.0f;
        for (int j = 0; j < i; ++j)
            packedApart = packedApart && dm::length(points[i] - points[j]) >= 0.25f * 0.999f;
    }
    CHECK(packedInside && packedApart);
}

void grids() {
    std::printf("particle grids, springs\n");
    RandInit();
    SimBuffers b;
    CreateParticleGrid(b, dm::float3(0.0f), 3, 4, 5, 0.1f, dm::float3(1.0f, 0.0f, 0.0f), 0.5f, true, 0.8f,
                       makePhase(1, 0));
    CHECK(b.positions.size() == 60 && b.velocities.size() == 60 && b.phases.size() == 60);
    CHECK(b.rigidOffsets.size() == 2 && b.rigidOffsets[0] == 0 && b.rigidOffsets[1] == 60);
    CHECK(b.rigidIndices.size() == 60 && b.rigidCoefficients.size() == 1 && b.rigidCoefficients[0] == 0.8f);
    CHECK(b.positions[59].w == 0.5f && b.velocities[0].x == 1.0f);
    // jittered by at most the jitter
    CHECK(dm::length(xyz(b.positions[59]) - dm::float3(0.2f, 0.3f, 0.4f)) <= 0.005f + 1e-6f);

    SimBuffers cloth;
    CreateSpringGrid(cloth, dm::float3(0.0f, 0.0f, -3.0f), 64, 32, 1, 0.05f, makePhase(0, PhaseSelfCollide), 0.9f,
                     1.0f, 0.9f, dm::float3(0.0f), 1.0f);
    CHECK(cloth.positions.size() == 2048);
    CHECK(cloth.triangles.size() == 3906 * 3 && cloth.triangleNormals.size() == 3906);
    CHECK(cloth.springLengths.size() == 11810 && cloth.springIndices.size() == 2 * 11810);
    bool restLengths = true;
    for (size_t i = 0; i < cloth.springLengths.size(); ++i) {
        const float d = dm::length(xyz(cloth.positions[size_t(cloth.springIndices[i * 2])]) -
                                   xyz(cloth.positions[size_t(cloth.springIndices[i * 2 + 1])]));
        restLengths = restLengths && std::fabs(d - cloth.springLengths[i]) < 1e-5f;
    }
    CHECK(restLengths);

    CreateSpring(cloth, 0, 5, -0.8f, 0.1f);
    CHECK(std::fabs(cloth.springLengths.back() - 1.1f * 0.25f) < 1e-5f && cloth.springStiffness.back() == -0.8f);
}

void shapes() {
    std::printf("CreateParticleShape\n");
    RandInit();
    // GranularPile
    SimBuffers granular;
    const float radius = 0.075f;
    CreateParticleShape(granular, g_Assets / "sphere.ply", dm::float3(8.0f, 4.0f, 2.0f), dm::float3(1.0f), 0.0f,
                        radius, dm::float3(0.0f), 0.0f, true, 1.0f, makePhase(1, 0), true, 0.00f);
    std::printf("  sphere: %zu particles\n", granular.positions.size());
    CHECK(granular.positions.size() == 1237);
    CHECK(granular.rigidOffsets.size() == 2 && granular.rigidOffsets[1] == 1237);
    CHECK(granular.rigidLocalNormals.size() == 1237);
    CHECK(granular.mesh && granular.rigidMeshSize.size() == 1 &&
          granular.rigidMeshSize[0] == int(granular.mesh->GetNumVertices()));
    CHECK(granular.meshSkinIndices.size() == 4 * granular.mesh->GetNumVertices());
    CHECK(granular.meshSkinWeights.size() == granular.meshSkinIndices.size());
    float weightError = 0.0f;
    for (size_t v = 0; v < granular.mesh->GetNumVertices(); ++v) {
        float sum = 0.0f;
        for (int w = 0; w < 4; ++w)
            sum += granular.meshSkinWeights[v * 4 + size_t(w)];
        weightError = std::max(weightError, std::fabs(sum - 1.0f));
    }
    CHECK(weightError < 1e-5f);

    CreateParticleShape(granular, g_Assets / "sandcastle.obj", dm::float3(-2.0f, -radius * 0.15f, 0.0f),
                        dm::float3(4.0f), 0.0f, radius * 1.0001f, dm::float3(0.0f), 1.0f, false, 0.0f,
                        makePhase(0, PhaseSelfCollide), false, 0.00f);
    std::printf("  sphere + sand castle: %zu particles\n", granular.positions.size());
    CHECK(granular.positions.size() == 42321);
    CHECK(granular.rigidOffsets.size() == 2);  // the castle is not rigid

    // the particles fill the sphere: within its radius (0.5 at scale 1) of its centre, with a jitter of 0
    dm::float3 lower, upper;
    SimBuffers ball;
    CreateParticleShape(ball, g_Assets / "sphere.ply", dm::float3(0.0f), dm::float3(1.0f), 0.0f, radius,
                        dm::float3(0.0f), 1.0f, true, 1.0f, makePhase(1, 0), false, 0.0f);
    GetParticleBounds(ball, lower, upper);
    CHECK(upper.x - lower.x > 0.8f && upper.x - lower.x < 1.0f);

    // a RigidPile box (radius 0.15, sx = sz = 2, sy = 2) and a FluidBlock torus
    std::unique_ptr<utils::Mesh> box = utils::ImportMesh(g_Assets / "box.ply");
    CHECK(box != nullptr);
    SimBuffers pile;
    const float r = 0.15f;
    CreateParticleShape(pile, box.get(), dm::float3(0.0f, 1.5f, 0.0f), r * 0.9f * dm::float3(2.0f), 0.0f, r * 0.9f,
                        dm::float3(0.0f), 1.0f, true, 1.0f, makePhase(1, 0), true, 0.002f);
    CHECK(pile.positions.size() == 8);
    SimBuffers torus;
    CreateParticleShape(torus, g_Assets / "torus.obj", dm::float3(4.5f, 2.0f, 1.0f), dm::float3(0.3f), 0.0f, 0.05f,
                        dm::float3(0.0f), 0.125f, true, 1.0f, makePhase(0, 0), true, 0.0f);
    CHECK(torus.positions.size() == 28);
    CHECK(torus.positions[0].w == 0.125f);

    // skinning: with the rigid at rest the mesh stays where it is
    pile.restPositions = pile.positions;
    pile.rigidRotations.assign(1, dm::float4(0.0f, 0.0f, 0.0f, 1.0f));
    pile.meshRestPositions = pile.mesh->m_positions;
    SkinMesh(pile);
    float skinError = 0.0f;
    for (size_t i = 0; i < pile.meshRestPositions.size(); ++i)
        skinError = std::max(skinError, dm::length(pile.mesh->m_positions[i] - pile.meshRestPositions[i]));
    CHECK(skinError < 1e-5f);

    // the centres of mass
    std::vector<dm::float3> translations(1);
    CalculateRigidCentersOfMass(pile.positions.data(), int(pile.positions.size()), pile.rigidOffsets.data(),
                                translations.data(), pile.rigidIndices.data(), 1);
    dm::float3 mean(0.0f);
    for (const dm::float4& p : pile.positions)
        mean += xyz(p);
    mean /= float(pile.positions.size());
    CHECK(dm::length(translations[0] - mean) < 1e-5f);
    std::vector<dm::float3> local(pile.positions.size());
    CalculateRigidLocalPositions(pile.positions.data(), pile.rigidOffsets.data(), translations.data(),
                                 pile.rigidIndices.data(), 1, local.data());
    CHECK(dm::length(local[0] + translations[0] - xyz(pile.positions[0])) < 1e-6f);
}

void collision() {
    std::printf("collision shapes\n");
    RandInit();
    SimBuffers b;
    AddRandomConvex(b, 12, dm::float3(3.8f, 0.0f, 1.0f), 0.5f, 0.7f, dm::float3(1.0f, 0.0f, 0.0f), 0.3f);
    CHECK(b.shapes.size() == 1);
    const ShapeDesc& convex = b.shapes[0];
    CHECK(convex.type == CollisionShapeType::ConvexMesh && convex.convexPlanes.size() == 12);
    for (const dm::float4& p : convex.convexPlanes)
        CHECK(-p.w >= 0.5f && -p.w <= 0.7f && std::fabs(dm::length(xyz(p)) - 1.0f) < 1e-5f);
    CHECK(convex.convexLower.x < 0.0f && convex.convexUpper.x > 0.0f);
    CHECK(convex.renderMesh && convex.renderMesh->GetNumFaces() >= 12 &&
          convex.renderMesh->m_normals.size() == convex.renderMesh->m_positions.size());
    // every corner of the hull on the inner side of every plane
    bool insidePlanes = true;
    for (const dm::float3& v : convex.renderMesh->m_positions)
        for (const dm::float4& p : convex.convexPlanes)
            insidePlanes = insidePlanes && dm::dot(xyz(p), v) + p.w <= 1e-4f;
    CHECK(insidePlanes);
    CHECK((convex.GetFlags() & ShapeFlagTypeMask) == int(CollisionShapeType::ConvexMesh));

    SimBuffers boxes;
    AddBox(boxes, dm::float3(1.0f, 0.5f, 2.0f), dm::float3(0.0f, 1.0f, 0.0f));
    dm::float3 lower, upper;
    GetShapeBounds(boxes, lower, upper);
    CHECK(dm::length(lower - dm::float3(-1.0f, 0.5f, -2.0f)) < 1e-5f &&
          dm::length(upper - dm::float3(1.0f, 1.5f, 2.0f)) < 1e-5f);
    // a quarter turn about y swaps the x and z extents
    AddBox(boxes, dm::float3(1.0f, 0.5f, 2.0f), dm::float3(10.0f, 1.0f, 0.0f),
           QuatFromAxisAngle(dm::float3(0.0f, 1.0f, 0.0f), 3.14159265f * 0.5f));
    GetShapeBounds(boxes, lower, upper);
    CHECK(std::fabs(upper.x - 12.0f) < 1e-4f && std::fabs(upper.z - 2.0f) < 1e-4f);

    SimBuffers none;
    GetShapeBounds(none, lower, upper);
    CHECK(lower.x == FLT_MAX && upper.x == -FLT_MAX);

    // AddPlinth: under the particles, its top at y = 1
    SimBuffers plinth;
    plinth.positions = {dm::float4(2.0f, 3.0f, 4.0f, 1.0f), dm::float4(4.0f, 5.0f, 6.0f, 1.0f)};
    AddPlinth(plinth);
    CHECK(plinth.shapes.size() == 1 && dm::all(plinth.shapes[0].position == dm::float3(3.0f, 0.5f, 5.0f)) &&
          dm::all(plinth.shapes[0].halfExtents == dm::float3(2.0f, 0.5f, 2.0f)));

    // picking: the nearest non-fluid particle within the radius of the ray
    const dm::float4 particles[3] = {dm::float4(0.0f, 0.0f, 5.0f, 1.0f), dm::float4(0.05f, 0.0f, 3.0f, 1.0f),
                                     dm::float4(0.0f, 0.0f, 2.0f, 1.0f)};
    const int phases[3] = {makePhase(0, 0), makePhase(0, 0), makePhase(0, PhaseFluid)};
    float t = 0.0f;
    CHECK(PickParticle(dm::float3(0.0f), dm::float3(0.0f, 0.0f, 1.0f), particles, phases, 3, 0.1f, t) == 1);
    CHECK(std::fabs(t - 3.0f) < 1e-6f);
    CHECK(PickParticle(dm::float3(0.0f), dm::float3(0.0f, 0.0f, -1.0f), particles, phases, 3, 0.1f, t) == -1);
}

}  // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i)
        if (std::strcmp(argv[i], "--assets") == 0 && i + 1 < argc)
            g_Assets = argv[++i];
    if (g_Assets.empty()) {
        std::printf("usage: ethereal_phys_helpers_test --assets <dir>\n");
        return 1;
    }

    voxels();
    random();
    grids();
    shapes();
    collision();

    std::printf(g_Failures ? "%d check(s) FAILED\n" : "passed\n", g_Failures);
    return g_Failures ? 1 : 0;
}
