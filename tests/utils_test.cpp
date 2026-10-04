// ethereal_phys_utils_test: the samples' utility library (ethereal_phys_utils, samples/common), without a device.
//
//     ethereal_phys_utils_test --assets <dir> [--temp <dir>]
//
// - Every mesh of assets/ (.obj, .ply, .bin) loads, with sane counts (a PLY's vertex count is its header's),
//   indices in range, finite positions with a non-degenerate bounding box, and unit normals; every distance field
//   (.pfm) loads at its header's size, finite, negative somewhere (inside) and positive somewhere (outside).
// - The flat binary mesh format and the PFM format round-trip.
// - The procedural primitives have FleX's counts and lie where they should; Normalize maps the bounds to [0, 1].
// - ConvexMeshBuilder: the hull of a plane set has its corners on the planes' inner side, outward-wound
//   triangles, and contains every interior point of the plane set; a cube's planes give its eight corners.
// - Extrude: the ring and index counts, the rings around the curve, unit normals.
// - Perlin noise: zero at the lattice points (one octave), bounded, periodic where asked.
//
// Exit code: 0 passed, 1 failed.

#include <ethereal/phys_utils/convex.h>
#include <ethereal/phys_utils/extrude.h>
#include <ethereal/phys_utils/mesh.h>
#include <ethereal/phys_utils/perlin.h>
#include <ethereal/phys_utils/pfm.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
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

using ephys::utils::Mesh;

bool finite3(const dm::float3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

// The vertex count of a PLY header.
uint32_t plyHeaderVertices(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios_base::binary);
    std::string word;
    while (file >> word) {
        if (word == "vertex") {
            uint32_t n = 0;
            file >> n;
            return n;
        }
        if (word == "end_header")
            break;
    }
    return 0;
}

void checkMesh(const std::filesystem::path& path) {
    const std::unique_ptr<Mesh> mesh = ephys::utils::ImportMesh(path);
    CHECK(mesh != nullptr);
    if (!mesh) {
        std::printf("  %s does not load\n", path.filename().string().c_str());
        return;
    }
    const uint32_t vertices = mesh->GetNumVertices();
    const uint32_t faces = mesh->GetNumFaces();
    CHECK(vertices > 0 && faces > 0);
    CHECK(mesh->m_indices.size() == size_t(faces) * 3);
    CHECK(mesh->m_normals.size() == vertices);

    uint32_t badIndices = 0, badPositions = 0, badNormals = 0;
    for (uint32_t index : mesh->m_indices)
        badIndices += index >= vertices ? 1 : 0;
    for (const dm::float3& p : mesh->m_positions)
        badPositions += finite3(p) ? 0 : 1;
    for (const dm::float3& n : mesh->m_normals)
        badNormals += finite3(n) && std::fabs(dm::length(n) - 1.0f) < 1e-3f ? 0 : 1;
    CHECK(badIndices == 0);
    CHECK(badPositions == 0);
    CHECK(badNormals == 0);

    dm::float3 lower, upper;
    mesh->GetBounds(lower, upper);
    const dm::float3 extent = upper - lower;
    const int flat = (extent.x > 0.0f ? 0 : 1) + (extent.y > 0.0f ? 0 : 1) + (extent.z > 0.0f ? 0 : 1);
    CHECK(flat <= 1);  // a plane is flat in one dimension, nothing in more

    std::string ext = path.extension().string();
    if (ext == ".ply")
        CHECK(vertices == plyHeaderVertices(path));

    std::printf("  %-22s %7u vertices %7u triangles  bounds (%.3g %.3g %.3g) .. (%.3g %.3g %.3g)%s%s%s\n",
                path.lexically_relative(path.parent_path().parent_path()).generic_string().c_str(), vertices, faces,
                double(lower.x), double(lower.y), double(lower.z), double(upper.x), double(upper.y), double(upper.z),
                badIndices ? " BAD INDICES" : "", badPositions ? " BAD POSITIONS" : "", badNormals ? " BAD NORMALS" : "");
}

void checkPfm(const std::filesystem::path& path) {
    ephys::utils::PfmImage image;
    CHECK(ephys::utils::PfmLoad(path, image));
    CHECK(image.m_width > 0 && image.m_height > 0 && image.m_depth > 0);
    CHECK(image.m_data.size() == size_t(image.m_width) * image.m_height * image.m_depth);
    float lo = FLT_MAX, hi = -FLT_MAX;
    uint64_t notFinite = 0;
    for (float v : image.m_data) {
        if (!std::isfinite(v)) {
            ++notFinite;
            continue;
        }
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    CHECK(notFinite == 0);
    CHECK(lo < 0.0f && hi > 0.0f);
    CHECK(std::isfinite(image.m_maxDepth));
    std::printf("  %-22s %u x %u x %u, %g .. %g (header %g)\n", path.filename().string().c_str(), image.m_width,
                image.m_height, image.m_depth, double(lo), double(hi), double(image.m_maxDepth));
}

void assets(const std::filesystem::path& dir) {
    std::printf("-- assets in %s\n", dir.generic_string().c_str());
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(dir))
        if (entry.is_regular_file())
            files.push_back(entry.path());
    std::sort(files.begin(), files.end());
    int meshes = 0, fields = 0;
    for (const std::filesystem::path& file : files) {
        const std::string ext = file.extension().string();
        if (ext == ".obj" || ext == ".ply" || ext == ".bin") {
            checkMesh(file);
            ++meshes;
        } else if (ext == ".pfm") {
            checkPfm(file);
            ++fields;
        }
    }
    std::printf("  %d meshes, %d distance fields\n", meshes, fields);
    CHECK(meshes >= 25 && fields >= 3);
}

void roundTrips(const std::filesystem::path& temp) {
    std::printf("-- round trips in %s\n", temp.generic_string().c_str());
    std::filesystem::create_directories(temp);

    const std::unique_ptr<Mesh> sphere = ephys::utils::CreateSphere(12, 16, 0.5f);
    const std::filesystem::path bin = temp / "utils_test_sphere.bin";
    CHECK(ephys::utils::ExportMeshToBin(bin, *sphere));
    const std::unique_ptr<Mesh> back = ephys::utils::ImportMesh(bin);
    CHECK(back != nullptr);
    if (back) {
        CHECK(back->m_positions.size() == sphere->m_positions.size());
        CHECK(back->m_indices == sphere->m_indices);
        CHECK(std::memcmp(back->m_positions.data(), sphere->m_positions.data(), sphere->m_positions.size() * 12) == 0);
        CHECK(std::memcmp(back->m_normals.data(), sphere->m_normals.data(), sphere->m_normals.size() * 12) == 0);
    }

    ephys::utils::PfmImage image;
    image.m_width = 4;
    image.m_height = 3;
    image.m_depth = 2;
    for (int i = 0; i < 24; ++i)
        image.m_data.push_back(float(i) * 0.25f - 2.0f);
    const std::filesystem::path pfm = temp / "utils_test.pfm";
    CHECK(ephys::utils::PfmSave(pfm, image));
    ephys::utils::PfmImage loaded;
    CHECK(ephys::utils::PfmLoad(pfm, loaded));
    CHECK(loaded.m_width == 4 && loaded.m_height == 3 && loaded.m_depth == 2);
    CHECK(loaded.m_data == image.m_data);
    CHECK(loaded.m_maxDepth == 3.75f);

    std::filesystem::remove(bin);
    std::filesystem::remove(pfm);
}

void primitives() {
    std::printf("-- primitives\n");
    auto sphere = ephys::utils::CreateSphere(20, 24, 2.0f);
    CHECK(sphere->GetNumVertices() == 21 * 25 && sphere->GetNumFaces() == 20 * 24 * 2);
    for (const dm::float3& p : sphere->m_positions)
        CHECK(std::fabs(dm::length(p) - 2.0f) < 1e-5f);

    auto capsule = ephys::utils::CreateCapsule(6, 12, 0.5f, 1.0f);
    CHECK(capsule->GetNumVertices() == (2 * 6 + 2) * 13 && capsule->GetNumFaces() == (2 * 6 + 1) * 12 * 2);
    dm::float3 lower, upper;
    capsule->GetBounds(lower, upper);
    CHECK(std::fabs(upper.y - 1.5f) < 1e-5f && std::fabs(lower.y + 1.5f) < 1e-5f);

    auto disc = ephys::utils::CreateDiscMesh(1.0f, 8);
    CHECK(disc->GetNumVertices() == 9 && disc->GetNumFaces() == 8);
    for (uint32_t i = 1; i < 9; ++i)
        CHECK(std::fabs(dm::length(disc->m_positions[i]) - 1.0f) < 1e-5f);

    auto cube = ephys::utils::CreateCubeMesh();
    CHECK(cube->GetNumVertices() == 24 && cube->GetNumFaces() == 12);
    cube->GetBounds(lower, upper);
    CHECK(lower.x == -0.5f && upper.z == 0.5f);

    auto tet = ephys::utils::CreateTetrahedron();
    CHECK(tet->GetNumVertices() == 4 && tet->GetNumFaces() == 4 && tet->m_normals.size() == 4);
    CHECK(ephys::utils::CreateTriMesh(1.0f)->GetNumFaces() == 1);
    CHECK(ephys::utils::CreateQuadMesh(1.0f)->GetNumFaces() == 2);

    // Normalize: lower bound at the origin, the largest dimension 1
    sphere->Normalize();
    sphere->GetBounds(lower, upper);
    CHECK(std::fabs(lower.x) < 1e-6f && std::fabs(lower.y) < 1e-6f && std::fabs(lower.z) < 1e-6f);
    CHECK(std::fabs(std::max(upper.x, std::max(upper.y, upper.z)) - 1.0f) < 1e-5f);
}

// The hull of a plane set against the definition of the convex: { x : n . x + d <= 0 for every plane }.
void checkHull(const char* name, const std::vector<dm::float4>& planes) {
    ephys::utils::ConvexMeshBuilder builder(planes.data());
    builder(uint32_t(planes.size()));
    const std::vector<dm::float3>& v = builder.mVertices;
    const std::vector<uint16_t>& t = builder.mIndices;
    CHECK(v.size() >= 4 && t.size() >= 12 && t.size() % 3 == 0);
    if (v.empty())
        return;

    auto inside = [&](const dm::float3& p, float eps) {
        for (const dm::float4& plane : planes)
            if (plane.x * p.x + plane.y * p.y + plane.z * p.z + plane.w > eps)
                return false;
        return true;
    };

    // the corners satisfy every plane
    uint32_t outside = 0;
    for (const dm::float3& p : v)
        outside += inside(p, 1e-4f) ? 0 : 1;
    CHECK(outside == 0);

    // the triangles are wound outwards
    dm::float3 centroid(0.0f);
    for (const dm::float3& p : v)
        centroid += p;
    centroid /= float(v.size());
    uint32_t inward = 0;
    for (size_t k = 0; k < t.size(); k += 3) {
        CHECK(t[k] < v.size() && t[k + 1] < v.size() && t[k + 2] < v.size());
        const dm::float3 n = dm::cross(v[t[k + 1]] - v[t[k]], v[t[k + 2]] - v[t[k]]);
        if (dm::dot(n, v[t[k]] - centroid) <= 0.0f)
            ++inward;
    }
    CHECK(inward == 0);

    // the interior of the plane set is inside the hull: sample points of the convex's bounding box
    dm::float3 lower(FLT_MAX), upper(-FLT_MAX);
    for (const dm::float3& p : v) {
        lower = dm::min(lower, p);
        upper = dm::max(upper, p);
    }
    uint32_t samples = 0, escaped = 0;
    const int steps = 12;
    for (int i = 0; i <= steps; ++i)
        for (int j = 0; j <= steps; ++j)
            for (int k = 0; k <= steps; ++k) {
                const dm::float3 p = lower + (upper - lower) * dm::float3(float(i), float(j), float(k)) / float(steps);
                if (!inside(p, -1e-3f))
                    continue;
                ++samples;
                for (size_t q = 0; q < t.size(); q += 3) {
                    const dm::float3 n = dm::cross(v[t[q + 1]] - v[t[q]], v[t[q + 2]] - v[t[q]]);
                    if (dm::dot(n, p - v[t[q]]) > 1e-5f * dm::length(n)) {
                        ++escaped;
                        break;
                    }
                }
            }
    CHECK(samples > 0 && escaped == 0);
    std::printf("  %s: %zu planes, %zu corners, %zu triangles, %u interior points inside\n", name, planes.size(),
                v.size(), t.size() / 3, samples - escaped);
}

void convex() {
    std::printf("-- ConvexMeshBuilder\n");
    // a cube of half-size 1: its eight corners
    std::vector<dm::float4> cube = {{1, 0, 0, -1}, {0, 1, 0, -1}, {0, 0, 1, -1}, {-1, 0, 0, -1}, {0, -1, 0, -1},
                                    {0, 0, -1, -1}};
    checkHull("cube", cube);
    {
        ephys::utils::ConvexMeshBuilder builder(cube.data());
        builder(6);
        CHECK(builder.mVertices.size() == 8);
        for (const dm::float3& p : builder.mVertices)
            CHECK(std::fabs(std::fabs(p.x) - 1.0f) < 1e-5f && std::fabs(std::fabs(p.y) - 1.0f) < 1e-5f &&
                  std::fabs(std::fabs(p.z) - 1.0f) < 1e-5f);
    }

    // the 12-kdop of the demo's CreateRandomConvex, with distances of a fixed sequence
    const dm::float3 directions[] = {{1, 0, 0},  {0, 1, 0},   {0, 0, 1},   {-1, 0, 0}, {0, -1, 0}, {0, 0, -1},
                                     {1, 1, 0},  {-1, -1, 0}, {1, 0, 1},   {-1, 0, -1}, {0, 1, 1}, {0, -1, -1}};
    std::vector<dm::float4> kdop;
    for (int i = 0; i < 12; ++i) {
        const dm::float3 n = dm::normalize(directions[i]);
        kdop.push_back(dm::float4(n, -(0.4f + 0.05f * float((i * 7) % 5))));
    }
    checkHull("12-kdop", kdop);

    // a cut tetrahedron
    std::vector<dm::float4> tet = {{0, -1, 0, -0.2f}, {0.8f, 0.3f, 0.5f, -0.6f}, {-0.8f, 0.3f, 0.5f, -0.6f},
                                   {0, 0.3f, -0.95f, -0.6f}, {0, 1, 0, -0.5f}};
    for (dm::float4& p : tet) {
        const dm::float3 n = dm::normalize(dm::float3(p.x, p.y, p.z));
        p = dm::float4(n, p.w);
    }
    checkHull("cut tetrahedron", tet);

    // fewer than 4 planes: nothing
    ephys::utils::ConvexMeshBuilder none(cube.data());
    none(3);
    CHECK(none.mVertices.empty() && none.mIndices.empty());
}

void extrude() {
    std::printf("-- Extrude\n");
    std::vector<dm::float3> points;
    for (int i = 0; i < 5; ++i)
        points.push_back(dm::float3(std::cos(0.6f * float(i)), 0.3f * float(i), std::sin(0.6f * float(i))));
    std::vector<dm::float3> vertices, normals;
    std::vector<int> indices;
    const int resolution = 8, smoothing = 4;
    ephys::utils::Extrude(points.data(), int(points.size()), vertices, normals, indices, 0.1f, resolution, smoothing);
    const int rings = 3 * smoothing + (smoothing + 1);
    CHECK(int(vertices.size()) == rings * resolution && normals.size() == vertices.size());
    CHECK(int(indices.size()) == (rings - 1) * resolution * 6);
    for (int index : indices)
        CHECK(index >= 0 && size_t(index) < vertices.size());
    for (const dm::float3& n : normals)
        CHECK(std::fabs(dm::length(n) - 1.0f) < 1e-4f);
    // the first ring around the first point, the last around the last
    for (int k = 0; k < resolution; ++k) {
        CHECK(std::fabs(dm::length(vertices[size_t(k)] - points.front()) - 0.1f) < 1e-4f);
        CHECK(std::fabs(dm::length(vertices[vertices.size() - 1 - size_t(k)] - points.back()) - 0.1f) < 1e-4f);
    }
    // appending: indices count what the arrays held before
    const size_t before = vertices.size();
    std::vector<int> more;
    ephys::utils::Extrude(points.data(), 2, vertices, normals, more, 0.1f, resolution, smoothing);
    for (int index : more)
        CHECK(size_t(index) >= before && size_t(index) < vertices.size());
    std::printf("  %zu vertices, %zu triangles\n", before, indices.size() / 3);
}

void perlin() {
    std::printf("-- Perlin noise\n");
    float lo = FLT_MAX, hi = -FLT_MAX;
    for (int i = -20; i <= 20; ++i) {
        CHECK(ephys::utils::Perlin1D(float(i), 1, 0.5f) == 0.0f);
        CHECK(ephys::utils::Perlin2D(float(i), float(3 - i), 1, 0.5f) == 0.0f);
        CHECK(ephys::utils::Perlin3D(float(i), 2.0f, float(-i), 1, 0.5f) == 0.0f);
        for (int j = 0; j < 10; ++j) {
            const float x = float(i) + 0.1f * float(j) + 0.05f;
            const float v = ephys::utils::Perlin3D(x, 0.37f * x, 1.0f - x, 4, 0.5f);
            lo = std::min(lo, v);
            hi = std::max(hi, v);
            CHECK(std::isfinite(v) && std::fabs(v) < 2.0f);
            // periodic up to the rounding of x + 4, for x >= 0 (FleX's quirk: see perlin.h)
            CHECK(x < 0.0f || std::fabs(ephys::utils::Perlin3DPeriodic(x, 0.3f, 0.7f, 4, 4, 4, 1, 0.5f) -
                            ephys::utils::Perlin3DPeriodic(x + 4.0f, 0.3f, 0.7f, 4, 4, 4, 1, 0.5f)) < 1e-5f);
        }
    }
    CHECK(lo < -0.1f && hi > 0.1f);
    CHECK(ephys::utils::Perlin1D(0.3f, 3, 0.5f) == ephys::utils::Perlin1D(0.3f, 3, 0.5f));
    std::printf("  Perlin3D over 400 samples: %.3f .. %.3f\n", double(lo), double(hi));
}

}  // namespace

int main(int argc, char** argv) {
    std::filesystem::path assetDir, temp = std::filesystem::temp_directory_path() / "ethereal_phys_utils_test";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--assets") == 0 && i + 1 < argc)
            assetDir = argv[++i];
        else if (std::strcmp(argv[i], "--temp") == 0 && i + 1 < argc)
            temp = argv[++i];
    }
    if (assetDir.empty()) {
        std::printf("usage: ethereal_phys_utils_test --assets <dir> [--temp <dir>]\n");
        return 2;
    }

    assets(assetDir);
    roundTrips(temp);
    primitives();
    convex();
    extrude();
    perlin();

    if (g_Failures) {
        std::printf("FAILED: %d check(s)\n", g_Failures);
        return 1;
    }
    std::printf("PASSED\n");
    return 0;
}
