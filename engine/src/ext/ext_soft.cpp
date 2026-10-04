#include "ext/ext_assets.h"

#include "ext/voxelize.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

// A port of reference/flex/extensions/flexExtSoft.cpp: NvFlexExtCreateSoftFromMesh and
// NvFlexExtCreateSoftMeshSkinning, with FleX's sweep-and-prune point queries, clustering, links and skinning.

namespace ephys::detail {

using namespace flex;

namespace {

Vec3 CalculateMean(const Vec3* particles, const int* indices, int numIndices) {
    Vec3 sum;

    for (int i = 0; i < numIndices; ++i)
        sum += particles[indices[i]];

    if (numIndices)
        return sum / float(numIndices);
    return sum;
}

struct Cluster {
    Vec3 mean;
    float radius = 0.0f;

    // the particles of the cluster
    std::vector<int> indices;
};

struct Seed {
    int index;
    float priority;

    bool operator<(const Seed& rhs) const { return priority < rhs.priority; }
};

// FleX's basic sweep-and-prune structure for point cloud queries: the points sorted along their longest axis.
struct SweepAndPrune {
    struct Entry {
        Entry(Vec3 p, int i) : point(p), index(i) {}

        Vec3 point;
        int index;
    };

    SweepAndPrune(const Vec3* points, int n) {
        entries.reserve(size_t(n));
        for (int i = 0; i < n; ++i)
            entries.push_back(Entry(points[i], i));

        struct SortOnAxis {
            int axis;

            explicit SortOnAxis(int a) : axis(a) {}

            bool operator()(const Entry& lhs, const Entry& rhs) const { return lhs.point[axis] < rhs.point[axis]; }
        };

        // the bounds of the points and their longest axis
        Vec3 lower(FLT_MAX), upper(-FLT_MAX);
        for (int i = 0; i < n; ++i) {
            lower = Min(points[i], lower);
            upper = Max(points[i], upper);
        }

        const Vec3 edges = upper - lower;

        if (edges.x > edges.y && edges.x > edges.z)
            longestAxis = 0;
        else if (edges.y > edges.z)
            longestAxis = 1;
        else
            longestAxis = 2;

        std::sort(entries.begin(), entries.end(), SortOnAxis(longestAxis));
    }

    void QuerySphere(Vec3 center, float radius, std::vector<int>& indices) const {
        // the first entry that may overlap, by binary search
        int low = 0;
        int high = int(entries.size());

        const float queryLower = center[longestAxis] - radius;
        const float queryUpper = center[longestAxis] + radius;

        while (low < high) {
            const int mid = (high + low) / 2;

            if (queryLower > entries[size_t(mid)].point[longestAxis])
                low = mid + 1;
            else
                high = mid;
        }

        // forward over the possible overlaps
        const float radiusSq = radius * radius;

        for (int i = low; i < int(entries.size()); ++i) {
            const Vec3 p = entries[size_t(i)].point;

            if (LengthSq(p - center) < radiusSq) {
                indices.push_back(entries[size_t(i)].index);
            } else if (entries[size_t(i)].point[longestAxis] > queryUpper) {
                // no more candidates
                break;
            }
        }
    }

    int longestAxis;  // 0, 1, 2: x, y, z

    std::vector<Entry> entries;
};

int CreateClusters(Vec3* particles, const float* priority, int numParticles, std::vector<int>& outClusterOffsets,
                   std::vector<int>& outClusterIndices, std::vector<Vec3>& outClusterPositions, float radius,
                   float smoothing = 0.0f) {
    std::vector<Seed> seeds;
    std::vector<Cluster> clusters;

    // whether a particle belongs to a cluster already
    std::vector<bool> used(size_t(numParticles), false);

    for (int i = 0; i < numParticles; ++i) {
        Seed s;
        s.index = i;
        s.priority = priority[i];
        seeds.push_back(s);
    }

    // by priority
    std::stable_sort(seeds.begin(), seeds.end());

    const SweepAndPrune sap(particles, numParticles);

    while (seeds.size()) {
        // the highest unused particle of the seeds
        const Seed seed = seeds.back();
        seeds.pop_back();

        if (!used[size_t(seed.index)]) {
            Cluster c;

            sap.QuerySphere(particles[seed.index], radius, c.indices);

            // the overlapping particles are no longer seeds
            for (int i = 0; i < int(c.indices.size()); ++i)
                used[size_t(c.indices[size_t(i)])] = true;

            c.mean = CalculateMean(particles, c.indices.data(), int(c.indices.size()));

            clusters.push_back(c);
        }
    }

    if (smoothing > 0.0f) {
        for (int i = 0; i < int(clusters.size()); ++i) {
            Cluster& c = clusters[size_t(i)];

            // the particles within the smoothing radius of the cluster mean
            c.indices.resize(0);
            sap.QuerySphere(c.mean, smoothing, c.indices);

            c.mean = CalculateMean(particles, c.indices.data(), int(c.indices.size()));
        }
    }

    int count = 0;

    for (int c = 0; c < int(clusters.size()); ++c) {
        const Cluster& cluster = clusters[size_t(c)];

        const int clusterSize = int(cluster.indices.size());

        // empty clusters are skipped
        if (clusterSize) {
            for (int i = 0; i < clusterSize; ++i)
                outClusterIndices.push_back(cluster.indices[size_t(i)]);

            outClusterOffsets.push_back(int(outClusterIndices.size()));
            outClusterPositions.push_back(cluster.mean);

            ++count;
        }
    }

    return count;
}

// distance constraints between the particles within a radius of each other
int CreateLinks(const Vec3* particles, int numParticles, std::vector<int>& outSpringIndices,
                std::vector<float>& outSpringLengths, std::vector<float>& outSpringStiffness, float radius,
                float stiffness = 1.0f) {
    int count = 0;

    std::vector<int> neighbors;
    const SweepAndPrune sap(particles, numParticles);

    for (int i = 0; i < numParticles; ++i) {
        neighbors.resize(0);

        sap.QuerySphere(particles[i], radius, neighbors);

        for (int j = 0; j < int(neighbors.size()); ++j) {
            const int nj = neighbors[size_t(j)];

            if (nj != i) {
                outSpringIndices.push_back(i);
                outSpringIndices.push_back(nj);
                outSpringLengths.push_back(Length(particles[i] - particles[nj]));
                outSpringStiffness.push_back(stiffness);

                ++count;
            }
        }
    }

    return count;
}

void CreateSkinning(const Vec3* vertices, int numVertices, const Vec3* clusters, int numClusters, float* outWeights,
                    int* outIndices, float falloff, float maxdist) {
    const int maxBones = 4;

    const SweepAndPrune sap(clusters, numClusters);

    std::vector<int> influences;

    // the closest bones of each vertex
    for (int i = 0; i < numVertices; ++i) {
        int indices[4] = {-1, -1, -1, -1};
        float distances[4] = {FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX};
        float weights[maxBones];

        influences.resize(0);
        sap.QuerySphere(vertices[i], maxdist, influences);

        for (int c = 0; c < int(influences.size()); ++c) {
            const float dSq = LengthSq(vertices[i] - clusters[influences[size_t(c)]]);

            // insertion sort
            int w = 0;
            for (; w < maxBones; ++w)
                if (dSq < distances[w])
                    break;

            if (w < maxBones) {
                // shuffle down
                for (int s = maxBones - 1; s > w; --s) {
                    indices[s] = indices[s - 1];
                    distances[s] = distances[s - 1];
                }

                distances[w] = dSq;
                indices[w] = influences[size_t(c)];
            }
        }

        // the weights by distance
        float wSum = 0.0f;

        for (int w = 0; w < maxBones; ++w) {
            if (distances[w] > Sqr(maxdist)) {
                // bones beyond the distance do not count
                weights[w] = 0.0f;
            } else {
                // the weight falls off inversely with the distance
                weights[w] = 1.0f / (powf(distances[w], falloff) + 0.0001f);
            }

            wSum += weights[w];
        }

        if (wSum == 0.0f) {
            // no weight: rigidly to the closest bone
            weights[0] = 1.0f;
        } else {
            for (int w = 0; w < maxBones; ++w)
                weights[w] = weights[w] / wSum;
        }

        for (int j = 0; j < maxBones; ++j) {
            outWeights[i * maxBones + j] = weights[j];
            outIndices[i * maxBones + j] = indices[j];
        }
    }
}

// the interior and surface samples of a mesh, clustered into particles
void SampleMesh(const Vec3* vertices, int numVertices, const int* indices, int numIndices, float radius,
                float volumeSampling, float surfaceSampling, Random& random, std::vector<Vec3>& outPositions) {
    Vec3 meshLower(FLT_MAX);
    Vec3 meshUpper(-FLT_MAX);

    for (int i = 0; i < numVertices; ++i) {
        meshLower = Min(meshLower, vertices[i]);
        meshUpper = Max(meshUpper, vertices[i]);
    }

    std::vector<Vec3> samples;

    if (volumeSampling > 0.0f) {
        const Vec3 edges = meshUpper - meshLower;

        // a finer voxelisation as the basis of the particles
        const float spacing = radius / volumeSampling;

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

        std::vector<uint32_t> voxels(size_t(maxDim) * size_t(maxDim) * size_t(maxDim));

        // the voxel centres symmetric to the centre of the object
        Vec3 meshOffset;
        meshOffset.x = 0.5f * (spacing - (edges.x - float(dx - 1) * spacing));
        meshOffset.y = 0.5f * (spacing - (edges.y - float(dy - 1) * spacing));
        meshOffset.z = 0.5f * (spacing - (edges.z - float(dz - 1) * spacing));
        meshLower -= meshOffset;

        Voxelize(vertices, numVertices, indices, numIndices, uint32_t(maxDim), uint32_t(maxDim), uint32_t(maxDim),
                 voxels.data(), meshLower, meshLower + Vec3(float(maxDim) * spacing));

        // the interior
        for (int x = 0; x < maxDim; ++x) {
            for (int y = 0; y < maxDim; ++y) {
                for (int z = 0; z < maxDim; ++z) {
                    const int index = z * maxDim * maxDim + y * maxDim + x;

                    if (voxels[size_t(index)])
                        samples.push_back(meshLower + spacing * Vec3(float(x) + 0.5f, float(y) + 0.5f, float(z) + 0.5f));
                }
            }
        }
    }

    if (surfaceSampling > 0.0f) {
        // the vertices
        for (int i = 0; i < numVertices; ++i)
            samples.push_back(vertices[i]);

        // random (non-uniform) samples of the surface
        const int numSamples = int(50000 * surfaceSampling);
        const int numTriangles = numIndices / 3;

        random.RandInit();

        for (int i = 0; i < numSamples; ++i) {
            const int t = int(random.Rand() % uint32_t(numTriangles));
            const float u = random.Randf();
            const float v = random.Randf() * (1.0f - u);
            const float w = 1.0f - u - v;

            const int a = indices[t * 3 + 0];
            const int b = indices[t * 3 + 1];
            const int c = indices[t * 3 + 2];

            const Vec3 p = vertices[a] * u + vertices[b] * v + vertices[c] * w;

            samples.push_back(p);
        }
    }

    std::vector<int> clusterIndices;
    std::vector<int> clusterOffsets;
    std::vector<float> priority(samples.size());

    // the samples clustered into the particles
    CreateClusters(samples.data(), priority.data(), int(samples.size()), clusterOffsets, clusterIndices, outPositions,
                   radius);
}

}  // namespace

bool createSoftFromMesh(const float* vertices, int numVertices, const int* indices, int numIndices,
                        const SoftAssetParams& params, Random& random, AssetData& asset) {
    const Vec3* input = reinterpret_cast<const Vec3*>(vertices);
    static_assert(sizeof(Vec3) == 12, "the vertices are read as Vec3");

    // relative coordinates, as in the rigid builder
    Vec3 meshOffset(0.0f);
    for (int i = 0; i < numVertices; i++)
        meshOffset += input[i];
    meshOffset /= float(numVertices);

    std::vector<Vec3> relativeVertices(static_cast<size_t>(numVertices));
    for (int i = 0; i < numVertices; i++)
        relativeVertices[size_t(i)] += input[i] - meshOffset;

    // the particles
    std::vector<Vec3> samples;
    SampleMesh(relativeVertices.data(), numVertices, indices, numIndices, params.particleSpacing,
               params.volumeSampling, params.surfaceSampling, random, samples);

    const int numParticles = int(samples.size());
    if (numParticles == 0)
        return false;

    std::vector<int> clusterIndices;
    std::vector<int> clusterOffsets;
    std::vector<Vec3> clusterPositions;
    std::vector<float> clusterCoefficients;
    std::vector<float> clusterPlasticThresholds;
    std::vector<float> clusterPlasticCreeps;

    // priority (FleX: "not currently used")
    std::vector<float> priority(static_cast<size_t>(numParticles));
    for (int i = 0; i < int(priority.size()); ++i)
        priority[size_t(i)] = 0.0f;

    // the particles clustered into shape-matching groups
    int numClusters = CreateClusters(samples.data(), priority.data(), int(samples.size()), clusterOffsets,
                                     clusterIndices, clusterPositions, params.clusterSpacing, params.clusterRadius);

    // one stiffness for all clusters
    clusterCoefficients.resize(size_t(numClusters), params.clusterStiffness);

    if (params.clusterPlasticCreep) {
        clusterPlasticThresholds.resize(size_t(numClusters), params.clusterPlasticThreshold);
        clusterPlasticCreeps.resize(size_t(numClusters), params.clusterPlasticCreep);
    }

    // links between the particles
    if (params.linkRadius > 0.0f) {
        std::vector<int> springIndices;
        std::vector<float> springLengths;
        std::vector<float> springStiffness;

        const int numLinks = CreateLinks(samples.data(), int(samples.size()), springIndices, springLengths,
                                         springStiffness, params.linkRadius, params.linkStiffness);

        if (numLinks) {
            asset.springIndices = springIndices;
            asset.springCoefficients = springStiffness;
            asset.springRestLengths = springLengths;
        }
    }

    // one more cluster of all particles, with the global stiffness
    if (params.globalStiffness > 0.0f) {
        numClusters += 1;
        clusterCoefficients.push_back(params.globalStiffness);
        if (params.clusterPlasticCreep) {
            clusterPlasticThresholds.push_back(params.clusterPlasticThreshold);
            clusterPlasticCreeps.push_back(params.clusterPlasticCreep);
        }

        for (int i = 0; i < numParticles; ++i)
            clusterIndices.push_back(i);
        clusterOffsets.push_back(int(clusterIndices.size()));

        // its centre is the mean of all particles
        Vec3 globalMeanPosition(0.0f);
        for (int i = 0; i < numParticles; ++i)
            globalMeanPosition += samples[size_t(i)];
        globalMeanPosition /= float(numParticles);
        clusterPositions.push_back(globalMeanPosition);
    }

    // back to absolute coordinates
    for (int i = 0; i < numParticles; ++i)
        samples[size_t(i)] += meshOffset;
    for (int i = 0; i < numClusters; ++i)
        clusterPositions[size_t(i)] += meshOffset;

    asset.particles.resize(size_t(numParticles) * 4);
    asset.numParticles = uint32_t(numParticles);
    asset.maxParticles = uint32_t(numParticles);

    for (int i = 0; i < numParticles; ++i) {
        asset.particles[size_t(i) * 4 + 0] = samples[size_t(i)].x;
        asset.particles[size_t(i) * 4 + 1] = samples[size_t(i)].y;
        asset.particles[size_t(i) * 4 + 2] = samples[size_t(i)].z;
        asset.particles[size_t(i) * 4 + 3] = 1.0f;
    }

    asset.shapeIndices.assign(clusterIndices.begin(), clusterIndices.end());
    asset.shapeOffsets.assign(clusterOffsets.begin(), clusterOffsets.begin() + numClusters);
    asset.shapeCenters.resize(size_t(numClusters) * 3);
    for (int i = 0; i < numClusters; ++i) {
        asset.shapeCenters[size_t(i) * 3 + 0] = clusterPositions[size_t(i)].x;
        asset.shapeCenters[size_t(i) * 3 + 1] = clusterPositions[size_t(i)].y;
        asset.shapeCenters[size_t(i) * 3 + 2] = clusterPositions[size_t(i)].z;
    }
    asset.shapeCoefficients.assign(clusterCoefficients.begin(), clusterCoefficients.begin() + numClusters);

    if (params.clusterPlasticCreep) {
        asset.shapePlasticThresholds = clusterPlasticThresholds;
        asset.shapePlasticCreeps = clusterPlasticCreeps;
    }

    return true;
}

void createSoftMeshSkinning(const float* vertices, int numVertices, const float* bones, int numBones, float falloff,
                            float maxDistance, float* skinningWeights, int* skinningIndices) {
    CreateSkinning(reinterpret_cast<const Vec3*>(vertices), numVertices, reinterpret_cast<const Vec3*>(bones),
                   numBones, skinningWeights, skinningIndices, falloff, maxDistance);
}

}  // namespace ephys::detail
