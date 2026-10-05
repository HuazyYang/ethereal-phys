#include "ext/ext_assets.h"

#include "ext/sdf.h"
#include "ext/voxelize.h"

#include <algorithm>
#include <cfloat>

// A port of FleX/extensions/flexExtRigid.cpp: NvFlexExtCreateRigidFromMesh.

namespace ephys::detail {

using namespace flex;

namespace {

float SampleSDF(const float* sdf, int dim, int x, int y, int z) { return sdf[z * dim * dim + y * dim + x]; }

// The gradient of the distance field by central differences (one-sided at the border).
Vec3 SampleSDFGrad(const float* sdf, int dim, int x, int y, int z) {
    const int x0 = std::max(x - 1, 0);
    const int x1 = std::min(x + 1, dim - 1);

    const int y0 = std::max(y - 1, 0);
    const int y1 = std::min(y + 1, dim - 1);

    const int z0 = std::max(z - 1, 0);
    const int z1 = std::min(z + 1, dim - 1);

    const float dx = (SampleSDF(sdf, dim, x1, y, z) - SampleSDF(sdf, dim, x0, y, z)) * (float(dim) * 0.5f);
    const float dy = (SampleSDF(sdf, dim, x, y1, z) - SampleSDF(sdf, dim, x, y0, z)) * (float(dim) * 0.5f);
    const float dz = (SampleSDF(sdf, dim, x, y, z1) - SampleSDF(sdf, dim, x, y, z0)) * (float(dim) * 0.5f);

    return Vec3(dx, dy, dz);
}

}  // namespace

bool createRigidFromMesh(const float* vertices, int numVertices, const int* indices, int numTriangleIndices,
                         float spacing, float expand, AssetData& asset) {
    const Vec3* input = reinterpret_cast<const Vec3*>(vertices);
    static_assert(sizeof(Vec3) == 12, "the vertices are read as Vec3");

    // Relative coordinates: the mean of the vertices is subtracted from every vertex, so that the centre of
    // mass is computed precisely (FleX: "the increased precision will prevent ghost forces").
    Vec3 meshOffset(0.0f);
    for (int i = 0; i < numVertices; i++)
        meshOffset += input[i];
    meshOffset /= float(numVertices);

    std::vector<Vec3> relativeVertices(static_cast<size_t>(numVertices));
    for (int i = 0; i < numVertices; i++)
        relativeVertices[size_t(i)] += input[i] - meshOffset;

    const Vec3* positions = relativeVertices.data();

    Vec3 meshLower(FLT_MAX), meshUpper(-FLT_MAX);
    for (int i = 0; i < numVertices; ++i) {
        meshLower = Min(meshLower, positions[i]);
        meshUpper = Max(meshUpper, positions[i]);
    }

    const Vec3 edges = meshUpper - meshLower;

    // a spacing a little smaller, for the case of an edge that is a whole multiple of the spacing
    const float spacingEps = spacing * (1.0f - 1e-4f);

    // at least one particle in each dimension
    const int dx = spacing > edges.x ? 1 : int(edges.x / spacingEps);
    const int dy = spacing > edges.y ? 1 : int(edges.y / spacingEps);
    const int dz = spacing > edges.z ? 1 : int(edges.z / spacingEps);

    int maxDim = std::max(std::max(dx, dy), dz);

    // two voxels of border for the sampling at the edges
    meshLower -= 2.0f * Vec3(spacing);
    meshUpper += 2.0f * Vec3(spacing);
    maxDim += 4;

    // the voxel centres symmetric to the centre of the object
    Vec3 meshShift;
    meshShift.x = 0.5f * (spacing - (edges.x - float(dx - 1) * spacing));
    meshShift.y = 0.5f * (spacing - (edges.y - float(dy - 1) * spacing));
    meshShift.z = 0.5f * (spacing - (edges.z - float(dz - 1) * spacing));
    meshLower -= meshShift;

    // no more than 64 voxels per side
    if (maxDim > 64)
        return false;

    std::vector<uint32_t> voxels(size_t(maxDim) * size_t(maxDim) * size_t(maxDim));

    Voxelize(relativeVertices.data(), numVertices, indices, numTriangleIndices, uint32_t(maxDim), uint32_t(maxDim),
             uint32_t(maxDim), voxels.data(), meshLower, meshLower + Vec3(float(maxDim) * spacing));

    std::vector<float> sdf(voxels.size());
    MakeSDF(voxels.data(), uint32_t(maxDim), uint32_t(maxDim), uint32_t(maxDim), sdf.data());

    std::vector<Vec4> particles;
    Vec3 center;

    for (int x = 0; x < maxDim; ++x) {
        for (int y = 0; y < maxDim; ++y) {
            for (int z = 0; z < maxDim; ++z) {
                const int index = z * maxDim * maxDim + y * maxDim + x;

                // a particle in each occupied voxel
                if (voxels[size_t(index)]) {
                    Vec3 position = meshLower + spacing * Vec3(float(x) + 0.5f, float(y) + 0.5f, float(z) + 0.5f);

                    // moved inside or outside the shape along the normal of the distance field (FleX also
                    // computed the normals and distances into an array it never used)
                    const Vec3 n = SafeNormalize(SampleSDFGrad(sdf.data(), maxDim, x, y, z));
                    position += n * expand;

                    particles.push_back(Vec4(position.x, position.y, position.z, 1.0f));

                    center += position;
                }
            }
        }
    }
    const int numParticles = int(particles.size());

    // back to absolute coordinates
    center /= float(numParticles);
    center += meshOffset;

    for (int i = 0; i < numParticles; i++)
        particles[size_t(i)] += Vec4(meshOffset, 0.0f);

    if (numParticles) {
        asset.numParticles = uint32_t(numParticles);
        asset.maxParticles = uint32_t(numParticles);

        asset.particles.resize(size_t(numParticles) * 4);
        for (int i = 0; i < numParticles; ++i) {
            const Vec4& p = particles[size_t(i)];
            float* out = &asset.particles[size_t(i) * 4];
            out[0] = p.x;
            out[1] = p.y;
            out[2] = p.z;
            out[3] = p.w;
        }

        // one shape of all particles
        asset.shapeIndices.resize(size_t(numParticles));
        for (int i = 0; i < numParticles; ++i)
            asset.shapeIndices[size_t(i)] = i;

        // its centre of mass (FleX allocated 4 floats and set 3)
        asset.shapeCenters = {center.x, center.y, center.z};
        asset.shapeCoefficients = {1.0f};
        asset.shapeOffsets = {numParticles};
    }

    return true;
}

}  // namespace ephys::detail
