#include <ethereal/phys_utils/mesh.h>

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <string>

// A port of FleX/core/mesh.cpp over Donut's math (mesh.h says what changed).

namespace ephys::utils {
namespace {

constexpr float kPi = 3.141592653589f;  // FleX's
constexpr float k2Pi = 2.0f * kPi;

// FleX's SafeNormalize: v / |v|, or the fallback for a zero vector.
dm::float3 safeNormalize(const dm::float3& v, const dm::float3& fallback) {
    const float l = dm::dot(v, v);
    return l > 0.0f ? v * (1.0f / std::sqrt(l)) : fallback;
}

enum class PlyFormat { Ascii, BinaryBigEndian };

template <typename T>
T plyRead(std::ifstream& s, PlyFormat format) {
    T data{};
    if (format == PlyFormat::Ascii) {
        s >> data;
    } else {
        char c[sizeof(T)];
        s.read(c, sizeof(T));
        std::reverse(c, c + sizeof(T));
        std::memcpy(&data, c, sizeof(T));
    }
    return data;
}

// Reads one face index of an ASCII PLY: as uint32_t (FleX reads uint32_t for ascii and uint8_t counts for binary).
uint32_t plyReadCount(std::ifstream& s, PlyFormat format) {
    return format == PlyFormat::Ascii ? plyRead<uint32_t>(s, format) : uint32_t(plyRead<uint8_t>(s, format));
}

struct VertexKey {
    uint32_t v = 0, vt = 0, vn = 0;

    bool operator<(const VertexKey& rhs) const {
        if (v != rhs.v)
            return v < rhs.v;
        if (vt != rhs.vt)
            return vt < rhs.vt;
        return vn < rhs.vn;
    }
};

std::string lowerExtension(const std::filesystem::path& path) {
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return char(std::tolower(c)); });
    return ext;
}

}  // namespace

void Mesh::DuplicateVertex(uint32_t i) {
    m_positions.push_back(m_positions[i]);

    if (m_normals.size() > i)
        m_normals.push_back(m_normals[i]);
    if (m_colours.size() > i)
        m_colours.push_back(m_colours[i]);
    if (m_texcoords[0].size() > i)
        m_texcoords[0].push_back(m_texcoords[0][i]);
    if (m_texcoords[1].size() > i)
        m_texcoords[1].push_back(m_texcoords[1][i]);
}

void Mesh::Normalize(float s) {
    dm::float3 lower, upper;
    GetBounds(lower, upper);
    const dm::float3 edges = upper - lower;

    Transform(dm::translation(-lower));

    const float maxEdge = std::max(edges.x, std::max(edges.y, edges.z));
    Transform(dm::scaling(dm::float3(s / maxEdge)));
}

void Mesh::CalculateNormals() {
    m_normals.assign(m_positions.size(), dm::float3(0.0f));

    const uint32_t numTris = GetNumFaces();
    for (uint32_t i = 0; i < numTris; ++i) {
        const uint32_t a = m_indices[i * 3 + 0];
        const uint32_t b = m_indices[i * 3 + 1];
        const uint32_t c = m_indices[i * 3 + 2];

        const dm::float3 n = dm::cross(m_positions[b] - m_positions[a], m_positions[c] - m_positions[a]);

        m_normals[a] += n;
        m_normals[b] += n;
        m_normals[c] += n;
    }

    for (dm::float3& n : m_normals)
        n = dm::normalize(n);
}

void Mesh::AddMesh(const Mesh& m) {
    const uint32_t offset = uint32_t(m_positions.size());

    m_positions.insert(m_positions.end(), m.m_positions.begin(), m.m_positions.end());
    m_normals.insert(m_normals.end(), m.m_normals.begin(), m.m_normals.end());
    m_colours.insert(m_colours.end(), m.m_colours.begin(), m.m_colours.end());

    for (uint32_t index : m.m_indices)
        m_indices.push_back(index + offset);
}

void Mesh::Transform(const dm::affine3& m) {
    for (size_t i = 0; i < m_positions.size(); ++i) {
        m_positions[i] = m.transformPoint(m_positions[i]);
        if (i < m_normals.size())
            m_normals[i] = m.transformVector(m_normals[i]);
    }
}

void Mesh::GetBounds(dm::float3& outMinExtents, dm::float3& outMaxExtents) const {
    dm::float3 minExtents(FLT_MAX);
    dm::float3 maxExtents(-FLT_MAX);

    for (const dm::float3& a : m_positions) {
        minExtents = dm::min(a, minExtents);
        maxExtents = dm::max(a, maxExtents);
    }

    outMinExtents = minExtents;
    outMaxExtents = maxExtents;
}

std::unique_ptr<Mesh> ImportMesh(const std::filesystem::path& path) {
    const std::string ext = lowerExtension(path);
    if (ext == ".ply")
        return ImportMeshFromPly(path);
    if (ext == ".obj")
        return ImportMeshFromObj(path);
    if (ext == ".bin")
        return ImportMeshFromBin(path);
    return nullptr;
}

std::unique_ptr<Mesh> ImportMeshFromBin(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios_base::in | std::ios_base::binary);
    if (!file)
        return nullptr;

    int32_t numVertices = 0;
    int32_t numIndices = 0;
    file.read(reinterpret_cast<char*>(&numVertices), sizeof(numVertices));
    file.read(reinterpret_cast<char*>(&numIndices), sizeof(numIndices));
    if (!file || numVertices < 0 || numIndices < 0)
        return nullptr;

    // FleX read the counts and trusted them; here they must match the size of the file.
    const auto expected = uint64_t(8) + uint64_t(numVertices) * 24 + uint64_t(numIndices) * 4;
    std::error_code error;
    if (std::filesystem::file_size(path, error) != expected || error)
        return nullptr;

    auto m = std::make_unique<Mesh>();
    m->m_positions.resize(size_t(numVertices));
    m->m_normals.resize(size_t(numVertices));
    m->m_indices.resize(size_t(numIndices));

    static_assert(sizeof(dm::float3) == 12, "dm::float3 must be 3 packed floats");
    file.read(reinterpret_cast<char*>(m->m_positions.data()), std::streamsize(sizeof(dm::float3) * size_t(numVertices)));
    file.read(reinterpret_cast<char*>(m->m_normals.data()), std::streamsize(sizeof(dm::float3) * size_t(numVertices)));
    file.read(reinterpret_cast<char*>(m->m_indices.data()), std::streamsize(sizeof(uint32_t) * size_t(numIndices)));
    if (!file)
        return nullptr;

    return m;
}

bool ExportMeshToBin(const std::filesystem::path& path, const Mesh& m) {
    std::ofstream file(path, std::ios_base::out | std::ios_base::binary);
    if (!file)
        return false;

    const int32_t numVertices = int32_t(m.m_positions.size());
    const int32_t numIndices = int32_t(m.m_indices.size());
    file.write(reinterpret_cast<const char*>(&numVertices), sizeof(numVertices));
    file.write(reinterpret_cast<const char*>(&numIndices), sizeof(numIndices));

    std::vector<dm::float3> normals = m.m_normals;
    normals.resize(m.m_positions.size(), dm::float3(0.0f));
    file.write(reinterpret_cast<const char*>(m.m_positions.data()), std::streamsize(sizeof(dm::float3) * m.m_positions.size()));
    file.write(reinterpret_cast<const char*>(normals.data()), std::streamsize(sizeof(dm::float3) * normals.size()));
    file.write(reinterpret_cast<const char*>(m.m_indices.data()), std::streamsize(sizeof(uint32_t) * m.m_indices.size()));
    return bool(file);
}

std::unique_ptr<Mesh> ImportMeshFromPly(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios_base::in | std::ios_base::binary);
    if (!file)
        return nullptr;

    std::string buffer;
    file >> buffer;
    if (buffer != "ply")
        return nullptr;

    PlyFormat format = PlyFormat::Ascii;

    uint32_t numFaces = 0;
    uint32_t numVertices = 0;

    const uint32_t kMaxProperties = 16;
    uint32_t numProperties = 0;
    float properties[kMaxProperties] = {};

    bool vertexElement = false;
    bool headerEnded = false;

    while (file >> buffer) {
        if (buffer == "element") {
            file >> buffer;
            if (buffer == "face") {
                vertexElement = false;
                file >> numFaces;
            } else if (buffer == "vertex") {
                vertexElement = true;
                file >> numVertices;
            } else {
                vertexElement = false;
            }
        } else if (buffer == "format") {
            file >> buffer;
            if (buffer == "ascii")
                format = PlyFormat::Ascii;
            else if (buffer == "binary_big_endian")
                format = PlyFormat::BinaryBigEndian;
            else
                return nullptr;  // FleX: "Ply: unknown format"
        } else if (buffer == "property") {
            if (vertexElement)
                ++numProperties;
        } else if (buffer == "end_header") {
            headerEnded = true;
            break;
        }
    }

    // FleX read at most 16 properties into its array without checking; three are the position
    if (!headerEnded || numProperties < 3 || numProperties > kMaxProperties)
        return nullptr;

    // the newline after end_header
    char nl;
    file.read(&nl, 1);

    auto mesh = std::make_unique<Mesh>();

    mesh->m_positions.resize(numVertices);
    mesh->m_normals.resize(numVertices);
    mesh->m_colours.resize(numVertices, dm::float4(1.0f));

    mesh->m_indices.reserve(size_t(numFaces) * 3);

    for (uint32_t v = 0; v < numVertices; ++v) {
        for (uint32_t i = 0; i < numProperties; ++i)
            properties[i] = plyRead<float>(file, format);

        mesh->m_positions[v] = dm::float3(properties[0], properties[1], properties[2]);
        mesh->m_normals[v] = dm::float3(0.0f);
    }

    for (uint32_t f = 0; f < numFaces; ++f) {
        const uint32_t numIndices = plyReadCount(file, format);
        if (numIndices != 3 && numIndices != 4)
            return nullptr;  // FleX asserted: only triangles and quads

        uint32_t indices[4];
        for (uint32_t i = 0; i < numIndices; ++i) {
            indices[i] = plyRead<uint32_t>(file, format);
            if (indices[i] >= numVertices)
                return nullptr;
        }

        mesh->m_indices.push_back(indices[0]);
        mesh->m_indices.push_back(indices[1]);
        mesh->m_indices.push_back(indices[2]);
        if (numIndices == 4) {
            mesh->m_indices.push_back(indices[2]);
            mesh->m_indices.push_back(indices[3]);
            mesh->m_indices.push_back(indices[0]);
        }

        // the vertex normals, as the faces come
        const dm::float3& v0 = mesh->m_positions[indices[0]];
        const dm::float3& v1 = mesh->m_positions[indices[1]];
        const dm::float3& v2 = mesh->m_positions[indices[2]];

        const dm::float3 n = safeNormalize(dm::cross(v1 - v0, v2 - v0), dm::float3(0.0f, 1.0f, 0.0f));

        for (uint32_t i = 0; i < numIndices; ++i)
            mesh->m_normals[indices[i]] += n;
    }

    if (!file)
        return nullptr;

    for (dm::float3& n : mesh->m_normals)
        n = safeNormalize(n, dm::float3(0.0f, 1.0f, 0.0f));

    return mesh;
}

std::unique_ptr<Mesh> ImportMeshFromObj(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file)
        return nullptr;

    auto m = std::make_unique<Mesh>();

    std::vector<dm::float3> positions;
    std::vector<dm::float3> normals;
    std::vector<dm::float2> texcoords;
    std::vector<uint32_t>& indices = m->m_indices;

    std::map<VertexKey, uint32_t> vertexLookup;

    std::string buffer;
    std::string line;

    while (file >> buffer) {
        if (buffer == "vn") {
            float x = 0, y = 0, z = 0;
            file >> x >> y >> z;
            normals.push_back(dm::float3(x, y, z));
        } else if (buffer == "vt") {
            float u = 0, v = 0;
            file >> u >> v;
            texcoords.push_back(dm::float2(u, v));
        } else if (buffer[0] == 'v') {
            float x = 0, y = 0, z = 0;
            file >> x >> y >> z;
            positions.push_back(dm::float3(x, y, z));
        } else if (buffer[0] == 's' || buffer[0] == 'g' || buffer[0] == 'o' || buffer[0] == '#' ||
                   buffer == "mtllib" || buffer == "usemtl") {
            // smoothing groups, groups, objects, comments and materials are skipped
            std::getline(file, line);
        } else if (buffer[0] == 'f') {
            uint32_t faceIndices[4];
            uint32_t faceIndexCount = 0;

            for (int i = 0; i < 4; ++i) {
                VertexKey key;

                // no further index on this face (FleX also dropped an index that ended the file; it is kept here)
                file >> key.v;
                if (file.fail()) {
                    file.clear(file.eof() ? std::ios_base::eofbit : std::ios_base::goodbit);
                    break;
                }

                if (file.peek() == '/') {
                    file.ignore();
                    if (file.peek() != '/')
                        file >> key.vt;
                    if (file.peek() == '/') {
                        file.ignore();
                        file >> key.vn;
                    }
                }

                if (key.v == 0 || key.v > positions.size() || key.vt > texcoords.size() || key.vn > normals.size())
                    return nullptr;  // FleX indexed out of range (it supports no negative indices either)

                const auto iter = vertexLookup.find(key);
                if (iter != vertexLookup.end()) {
                    faceIndices[faceIndexCount++] = iter->second;
                } else {
                    const uint32_t newIndex = uint32_t(m->m_positions.size());
                    faceIndices[faceIndexCount++] = newIndex;

                    vertexLookup.insert(std::make_pair(key, newIndex));

                    m->m_positions.push_back(positions[key.v - 1]);

                    // OBJ has no vertex colours
                    m->m_colours.push_back(dm::float4(1.0f));

                    if (key.vn)
                        m->m_normals.push_back(normals[key.vn - 1]);
                    if (key.vt)
                        m->m_texcoords[0].push_back(texcoords[key.vt - 1]);
                }
            }

            if (faceIndexCount == 3) {
                indices.insert(indices.end(), faceIndices, faceIndices + 3);
            } else if (faceIndexCount == 4) {
                // a quad, triangulated clockwise
                indices.insert(indices.end(), faceIndices, faceIndices + 3);
                indices.push_back(faceIndices[2]);
                indices.push_back(faceIndices[3]);
                indices.push_back(faceIndices[0]);
            }
            // FleX printed "Face with more than 4 vertices are not supported" and skipped the face
        }
    }

    // the normals of the faces are added to those of the file (if any), then normalised
    m->m_normals.resize(m->m_positions.size(), dm::float3(0.0f));

    const uint32_t numFaces = uint32_t(indices.size()) / 3;
    for (uint32_t i = 0; i < numFaces; ++i) {
        const uint32_t a = indices[i * 3 + 0];
        const uint32_t b = indices[i * 3 + 1];
        const uint32_t c = indices[i * 3 + 2];

        const dm::float3& v0 = m->m_positions[a];
        const dm::float3& v1 = m->m_positions[b];
        const dm::float3& v2 = m->m_positions[c];

        const dm::float3 n = safeNormalize(dm::cross(v1 - v0, v2 - v0), dm::float3(0.0f, 1.0f, 0.0f));

        m->m_normals[a] += n;
        m->m_normals[b] += n;
        m->m_normals[c] += n;
    }

    for (dm::float3& n : m->m_normals)
        n = safeNormalize(n, dm::float3(0.0f, 1.0f, 0.0f));

    return m;
}

std::unique_ptr<Mesh> CreateTriMesh(float size, float y) {
    auto m = std::make_unique<Mesh>();
    m->m_indices = {0, 1, 2};
    m->m_positions = {dm::float3(-size, y, size), dm::float3(size, y, size), dm::float3(size, y, -size)};
    m->m_normals.assign(3, dm::float3(0.0f, 1.0f, 0.0f));
    return m;
}

std::unique_ptr<Mesh> CreateCubeMesh() {
    const dm::float3 vertices[24] = {
        {0.5f, 0.5f, 0.5f},   {-0.5f, 0.5f, 0.5f},  {0.5f, -0.5f, 0.5f},   {-0.5f, -0.5f, 0.5f},
        {0.5f, 0.5f, -0.5f},  {-0.5f, 0.5f, -0.5f}, {0.5f, -0.5f, -0.5f},  {-0.5f, -0.5f, -0.5f},
        {0.5f, 0.5f, 0.5f},   {0.5f, -0.5f, 0.5f},  {0.5f, 0.5f, 0.5f},    {0.5f, 0.5f, -0.5f},
        {-0.5f, 0.5f, 0.5f},  {-0.5f, 0.5f, -0.5f}, {0.5f, -0.5f, -0.5f},  {0.5f, 0.5f, -0.5f},
        {-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f, 0.5f},  {0.5f, -0.5f, 0.5f},
        {-0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f, 0.5f}, {-0.5f, 0.5f, -0.5f}, {-0.5f, 0.5f, 0.5f}};

    // FleX's normals (some of them do not match their faces; kept as they are)
    const dm::float3 normals[24] = {
        {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f},  {0.0f, 1.0f, 0.0f},  {1.0f, 0.0f, 0.0f},  {0.0f, 0.0f, -1.0f},
        {1.0f, 0.0f, 0.0f},  {1.0f, 0.0f, 0.0f},  {0.0f, 1.0f, 0.0f},  {0.0f, 1.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},  {0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, -1.0f}, {-0.0f, -0.0f, -1.0f},
        {0.0f, -1.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {-0.0f, -1.0f, -0.0f},
        {-1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {-1.0f, -0.0f, -0.0f}};

    const uint32_t indices[36] = {0,  1,  2,  3,  2,  1,  8,  9,  4,  6,  4,  9,  10, 11, 12, 5,  12, 11,
                                  7,  13, 14, 15, 14, 13, 16, 17, 18, 19, 18, 17, 20, 21, 22, 23, 22, 21};

    auto m = std::make_unique<Mesh>();
    m->m_positions.assign(vertices, vertices + 24);
    m->m_normals.assign(normals, normals + 24);
    m->m_indices.assign(indices, indices + 36);
    return m;
}

std::unique_ptr<Mesh> CreateQuadMesh(float size, float y) {
    auto m = std::make_unique<Mesh>();
    m->m_indices = {0, 1, 2, 2, 3, 0};
    m->m_positions = {dm::float3(-size, y, size), dm::float3(size, y, size), dm::float3(size, y, -size),
                      dm::float3(-size, y, -size)};
    m->m_normals.assign(4, dm::float3(0.0f, 1.0f, 0.0f));
    return m;
}

std::unique_ptr<Mesh> CreateDiscMesh(float radius, uint32_t segments) {
    const uint32_t numVerts = 1 + segments;

    auto m = std::make_unique<Mesh>();
    m->m_positions.resize(numVerts);
    m->m_normals.resize(numVerts, dm::float3(0.0f, 1.0f, 0.0f));

    m->m_positions[0] = dm::float3(0.0f);
    if (segments == 0)
        return m;
    m->m_positions[1] = dm::float3(0.0f, 0.0f, radius);

    for (uint32_t i = 1; i <= segments; ++i) {
        uint32_t nextVert = (i + 1) % numVerts;
        if (nextVert == 0)
            nextVert = 1;

        const float angle = (float(i) / float(segments)) * k2Pi;
        m->m_positions[nextVert] = dm::float3(radius * std::sin(angle), 0.0f, radius * std::cos(angle));

        m->m_indices.push_back(0);
        m->m_indices.push_back(i);
        m->m_indices.push_back(nextVert);
    }

    return m;
}

std::unique_ptr<Mesh> CreateTetrahedron(float ground, float height) {
    auto m = std::make_unique<Mesh>();

    const float dimValue = 1.0f / std::sqrt(2.0f);
    m->m_positions = {dm::float3(-1.0f, ground, -dimValue), dm::float3(1.0f, ground, -dimValue),
                      dm::float3(0.0f, ground + height, dimValue), dm::float3(0.0f, ground, dimValue)};

    // counter-clockwise
    m->m_indices = {0, 2, 1, 2, 3, 1, 2, 0, 3, 3, 0, 1};

    m->CalculateNormals();
    return m;
}

std::unique_ptr<Mesh> CreateSphere(int slices, int segments, float radius) {
    const float dTheta = kPi / float(slices);
    const float dPhi = k2Pi / float(segments);

    const int vertsPerRow = segments + 1;

    auto mesh = std::make_unique<Mesh>();

    for (int i = 0; i <= slices; ++i) {
        const float theta = dTheta * float(i);

        for (int j = 0; j <= segments; ++j) {
            const float phi = dPhi * float(j);

            const float x = std::sin(theta) * std::cos(phi);
            const float y = std::cos(theta);
            const float z = std::sin(theta) * std::sin(phi);

            mesh->m_positions.push_back(dm::float3(x, y, z) * radius);
            mesh->m_normals.push_back(dm::float3(x, y, z));

            if (i > 0 && j > 0) {
                const uint32_t a = uint32_t(i * vertsPerRow + j);
                const uint32_t b = uint32_t((i - 1) * vertsPerRow + j);
                const uint32_t c = uint32_t((i - 1) * vertsPerRow + j - 1);
                const uint32_t d = uint32_t(i * vertsPerRow + j - 1);

                // a quad of this slice
                mesh->m_indices.insert(mesh->m_indices.end(), {b, a, d, b, d, c});
            }
        }
    }

    return mesh;
}

std::unique_ptr<Mesh> CreateCapsule(int slices, int segments, float radius, float halfHeight) {
    const float dTheta = kPi / float(slices * 2);
    const float dPhi = k2Pi / float(segments);

    const int vertsPerRow = segments + 1;

    auto mesh = std::make_unique<Mesh>();

    float theta = 0.0f;

    for (int i = 0; i <= 2 * slices + 1; ++i) {
        for (int j = 0; j <= segments; ++j) {
            const float phi = dPhi * float(j);

            const float x = std::sin(theta) * std::cos(phi);
            const float y = std::cos(theta);
            const float z = std::sin(theta) * std::sin(phi);

            // the y offset of the hemisphere
            const float yoffset = (i < slices) ? halfHeight : -halfHeight;

            mesh->m_positions.push_back(dm::float3(x, y, z) * radius + dm::float3(0.0f, yoffset, 0.0f));
            mesh->m_normals.push_back(dm::float3(x, y, z));

            if (i > 0 && j > 0) {
                const uint32_t a = uint32_t(i * vertsPerRow + j);
                const uint32_t b = uint32_t((i - 1) * vertsPerRow + j);
                const uint32_t c = uint32_t((i - 1) * vertsPerRow + j - 1);
                const uint32_t d = uint32_t(i * vertsPerRow + j - 1);

                mesh->m_indices.insert(mesh->m_indices.end(), {b, a, d, b, d, c});
            }
        }

        // the middle slice keeps theta
        if (i != slices)
            theta += dTheta;
    }

    return mesh;
}

}  // namespace ephys::utils
