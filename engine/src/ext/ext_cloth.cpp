#include "ext/ext_assets.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>

// A port of FleX/extensions/flexExtCloth.cpp: NvFlexExtCreateWeldedMeshIndices,
// NvFlexExtCreateClothFromMesh, NvFlexExtCreateTearingClothFromMesh and NvFlexExtTearClothMesh.

namespace ephys::detail {

using namespace flex;

namespace {

struct Key {
    Key(int i, float d) : index(i), depth(d) {}

    int index;
    float depth;

    bool operator<(const Key& rhs) const { return depth < rhs.depth; }
};

Vec3 VertexAt(const float* vertices, int i) { return Vec3(&vertices[size_t(i) * 3]); }
Vec4 ParticleAt(const float* particles, int i) { return Vec4(&particles[size_t(i) * 4]); }

void copyInflatable(const ClothMesh& cloth, float pressure, bool& inflatable, float& volume, float& stiffness,
                    float& outPressure) {
    if (pressure > 0.0f) {
        inflatable = true;
        volume = cloth.mRestVolume;
        stiffness = cloth.mConstraintScale;
        outPressure = pressure;
    }
}

}  // namespace

int createWeldedMeshIndices(const float* vertices, int numVertices, int* uniqueIndices, int* originalToUniqueMap,
                            float threshold) {
    std::memset(originalToUniqueMap, -1, size_t(numVertices) * sizeof(int));

    // a sweep and prune style search along z
    std::vector<Key> keys;
    for (int i = 0; i < numVertices; i++)
        keys.push_back(Key(i, VertexAt(vertices, i).z));

    std::sort(keys.begin(), keys.end());

    int uniqueCount = 0;

    for (int i = 0; i < numVertices; ++i) {
        // a duplicate
        if (originalToUniqueMap[keys[size_t(i)].index] != -1)
            continue;

        // forward until no vertex can be closer than the threshold
        for (int j = i + 1; j < numVertices && (keys[size_t(j)].depth - keys[size_t(i)].depth) <= threshold; ++j) {
            const float distance =
                Length(VertexAt(vertices, keys[size_t(i)].index) - VertexAt(vertices, keys[size_t(j)].index));

            if (distance <= threshold)
                originalToUniqueMap[keys[size_t(j)].index] = uniqueCount;
        }

        originalToUniqueMap[keys[size_t(i)].index] = uniqueCount;

        uniqueIndices[uniqueCount++] = keys[size_t(i)].index;
    }

    return uniqueCount;
}

bool createClothFromMesh(const float* particles, int numVertices, const int* indices, int numTriangles,
                         const ClothAssetParams& params, AssetData& asset) {
    asset.particles.assign(particles, particles + size_t(numVertices) * 4);
    asset.triangleIndices.assign(indices, indices + size_t(numTriangles) * 3);
    asset.numParticles = uint32_t(numVertices);
    asset.maxParticles = uint32_t(numVertices);

    ClothMesh cloth(particles, numVertices, indices, numTriangles * 3, params.stretchStiffness, params.bendStiffness);
    if (!cloth.mValid)
        return false;

    // the tethers
    if (params.tetherStiffness > 0.0f) {
        std::vector<int> anchors;
        anchors.reserve(size_t(numVertices));

        // the fixed particles
        for (int i = 0; i < numVertices; ++i) {
            if (ParticleAt(particles, i).w == 0.0f)
                anchors.push_back(i);
        }

        if (anchors.size()) {
            for (int i = 0; i < numVertices; ++i) {
                const Vec4 particle = ParticleAt(particles, i);
                if (particle.w == 0.0f)
                    continue;

                float minSqrDist = FLT_MAX;
                int minIndex = -1;

                // the closest attachment
                for (int a = 0; a < int(anchors.size()); ++a) {
                    const Vec4 attachment = ParticleAt(particles, anchors[size_t(a)]);

                    const float distSqr = LengthSq(XYZ(particle) - XYZ(attachment));
                    if (distSqr < minSqrDist) {
                        minSqrDist = distSqr;
                        minIndex = anchors[size_t(a)];
                    }
                }

                if (minIndex != -1) {
                    cloth.mConstraintIndices.push_back(i);
                    cloth.mConstraintIndices.push_back(minIndex);
                    cloth.mConstraintRestLengths.push_back(sqrtf(minSqrDist) * (1.0f + params.tetherGive));

                    // a negative stiffness marks a tether (a unilateral constraint)
                    cloth.mConstraintCoefficients.push_back(-params.tetherStiffness);
                }
            }
        }
    }

    asset.springIndices = cloth.mConstraintIndices;
    asset.springCoefficients = cloth.mConstraintCoefficients;
    asset.springRestLengths = cloth.mConstraintRestLengths;

    copyInflatable(cloth, params.pressure, asset.inflatable, asset.inflatableVolume, asset.inflatableStiffness,
                   asset.inflatablePressure);
    return true;
}

bool createTearingClothFromMesh(const float* particles, int numParticles, int maxParticles, const int* indices,
                                int numTriangles, const TearingClothAssetParams& params, TearingClothData& cloth) {
    // FleX left the storage beyond numParticles uninitialised; it is zero here.
    cloth.particles.assign(size_t(maxParticles) * 4, 0.0f);
    std::copy(particles, particles + size_t(numParticles) * 4, cloth.particles.begin());
    cloth.triangleIndices.assign(indices, indices + size_t(numTriangles) * 3);
    cloth.numParticles = uint32_t(numParticles);
    cloth.maxParticles = uint32_t(maxParticles);

    cloth.mesh = std::make_unique<ClothMesh>(particles, numParticles, indices, numTriangles * 3,
                                             params.stretchStiffness, params.bendStiffness);
    if (!cloth.mesh->mValid)
        return false;

    copyInflatable(*cloth.mesh, params.pressure, cloth.inflatable, cloth.inflatableVolume, cloth.inflatableStiffness,
                   cloth.inflatablePressure);
    return true;
}

void tearClothMesh(TearingClothData& cloth, Random& random, float maxStrain, int maxSplits,
                   TearingParticleClone* particleCopies, int* numParticleCopies, int maxCopies,
                   TearingMeshEdit* triangleEdits, int* numTriangleEdits, int maxEdits) {
    ClothMesh& mesh = *cloth.mesh;
    float* particles = cloth.particles.data();

    std::vector<ClothMesh::TriangleUpdate> edits;
    std::vector<ClothMesh::VertexCopy> copies;

    int splits = 0;

    maxCopies = Min(maxCopies, int(cloth.maxParticles) - int(cloth.numParticles));

    // FleX iterates over the spring count of the asset, which tearing does not change
    const int numSprings = int(mesh.mConstraintCoefficients.size());

    // tear every edge strained beyond the maximum
    for (int i = 0; i < numSprings && int(copies.size()) < maxCopies && splits < maxSplits; ++i) {
        const int a = mesh.mConstraintIndices[size_t(i) * 2 + 0];
        const int b = mesh.mConstraintIndices[size_t(i) * 2 + 1];

        const Vec3 p = Vec3(&particles[size_t(a) * 4]);
        const Vec3 q = Vec3(&particles[size_t(b) * 4]);

        if (Length(p - q) > mesh.mConstraintRestLengths[size_t(i)] * maxStrain) {
            // fixed particles are not split
            if (particles[size_t(a) * 4 + 3] == 0.0f)
                continue;
            if (particles[size_t(b) * 4 + 3] == 0.0f)
                continue;

            // the vertex of the edge to split
            const int splitIndex = random.Randf() > 0.5f ? a : b;
            const Vec3 splitPlane = Normalize(p - q);  // FleX: "todo: use plane perpendicular to normal and edge"

            std::vector<int> adjacentTriangles;
            std::vector<int> adjacentVertices;

            const int newIndex = mesh.SplitVertex(particles, splitIndex, splitPlane, adjacentTriangles,
                                                  adjacentVertices, edits, copies, maxCopies - int(copies.size()));

            if (newIndex != -1) {
                ++splits;

                // separate each adjacent vertex that is singular now
                for (int s = 0; s < int(adjacentVertices.size()); ++s)
                    mesh.SeparateVertex(adjacentVertices[size_t(s)], edits, copies, maxCopies - int(copies.size()));

                // and the new vertex, which can become singular too
                mesh.SeparateVertex(newIndex, edits, copies, maxCopies - int(copies.size()));
            }
        }
    }

    cloth.numParticles = uint32_t(mesh.mNumVertices);

    for (int c = 0; c < int(copies.size()); ++c) {
        TearingParticleClone clone;
        clone.srcIndex = copies[size_t(c)].srcIndex;
        clone.destIndex = copies[size_t(c)].destIndex;
        particleCopies[c] = clone;
    }

    // edits beyond maxEdits are not reported, and not applied to the triangle indices of the asset
    const int numEdits = Min(int(edits.size()), maxEdits);

    for (int u = 0; u < numEdits; ++u) {
        TearingMeshEdit update;
        update.triIndex = edits[size_t(u)].triangle;
        update.newParticleIndex = edits[size_t(u)].vertex;

        cloth.triangleIndices[size_t(update.triIndex)] = update.newParticleIndex;

        triangleEdits[u] = update;
    }

    *numTriangleEdits = numEdits;
    *numParticleCopies = int(copies.size());
}

}  // namespace ephys::detail
