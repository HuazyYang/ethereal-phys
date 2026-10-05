#include <ethereal/phys_helpers/helpers.h>

#include <ethereal/phys_utils/convex.h>
#include <ethereal/phys_utils/extrude.h>
#include <ethereal/phys_utils/pfm.h>
#include <ethereal/phys_utils/voxelize.h>

#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <map>

// A port of reference/flex/demo/helpers.h (see helpers.h for what changed). The bodies follow FleX's, statement
// for statement, with g_buffers -> buffers and FleX's Vec3 / Vec4 / Quat -> dm::float3 / float4 / float4.

namespace ephys::helpers {

namespace {

dm::float3 xyz(const dm::float4& v) { return dm::float3(v.x, v.y, v.z); }

// FleX's SafeNormalize: v / |v|, or the fallback for a zero vector.
dm::float3 safeNormalize(const dm::float3& v, const dm::float3& fallback = dm::float3(0.0f)) {
    const float l = dm::dot(v, v);
    if (l > 0.0f)
        return v * (1.0f / sqrtf(l));
    return fallback;
}

float sampleSDF(const float* sdf, int dim, int x, int y, int z) {
    assert(x < dim && x >= 0);
    assert(y < dim && y >= 0);
    assert(z < dim && z >= 0);

    return sdf[z * dim * dim + y * dim + x];
}

// The gradient of a distance field (FleX: SampleSDFGrad).
dm::float3 sampleSDFGrad(const float* sdf, int dim, int x, int y, int z) {
    const int x0 = std::max(x - 1, 0);
    const int x1 = std::min(x + 1, dim - 1);

    const int y0 = std::max(y - 1, 0);
    const int y1 = std::min(y + 1, dim - 1);

    const int z0 = std::max(z - 1, 0);
    const int z1 = std::min(z + 1, dim - 1);

    const float dx = (sampleSDF(sdf, dim, x1, y, z) - sampleSDF(sdf, dim, x0, y, z)) * (float(dim) * 0.5f);
    const float dy = (sampleSDF(sdf, dim, x, y1, z) - sampleSDF(sdf, dim, x, y0, z)) * (float(dim) * 0.5f);
    const float dz = (sampleSDF(sdf, dim, x, y, z1) - sampleSDF(sdf, dim, x, y, z0)) * (float(dim) * 0.5f);

    return dm::float3(dx, dy, dz);
}

// A shape appended with its previous transform equal to its current one, as FleX's helpers do.
ShapeDesc& addShape(SimBuffers& buffers, CollisionShapeType type, const dm::float3& position,
                    const dm::float4& rotation) {
    ShapeDesc& shape = buffers.shapes.emplace_back();
    shape.type = type;
    shape.position = position;
    shape.rotation = rotation;
    shape.prevPosition = position;
    shape.prevRotation = rotation;
    return shape;
}

}  // namespace

// ---- Quaternions ---------------------------------------------------------------------------------------------

dm::float4 QuatFromAxisAngle(const dm::float3& axis, float angle) {
    dm::float3 v = dm::normalize(axis);

    const float half = angle * 0.5f;
    const float w = cosf(half);

    const float sinThetaOverTwo = sinf(half);
    v *= sinThetaOverTwo;

    return dm::float4(v.x, v.y, v.z, w);
}

dm::float3 QuatRotate(const dm::float4& q, const dm::float3& x) {
    const dm::float3 v = xyz(q);
    return x * (2.0f * q.w * q.w - 1.0f) + dm::cross(v, x) * q.w * 2.0f + v * dm::dot(v, x) * 2.0f;
}

void QuatToMatrix33(const dm::float4& q, dm::float3 cols[3]) {
    cols[0] = QuatRotate(q, dm::float3(1.0f, 0.0f, 0.0f));
    cols[1] = QuatRotate(q, dm::float3(0.0f, 1.0f, 0.0f));
    cols[2] = QuatRotate(q, dm::float3(0.0f, 0.0f, 1.0f));
}

// ---- Particles ---------------------------------------------------------------------------------------------

void GetParticleBounds(const SimBuffers& buffers, dm::float3& lower, dm::float3& upper) {
    lower = dm::float3(FLT_MAX);
    upper = dm::float3(-FLT_MAX);

    for (const dm::float4& p : buffers.positions) {
        lower = dm::min(xyz(p), lower);
        upper = dm::max(xyz(p), upper);
    }
}

void CreateParticleGrid(SimBuffers& buffers, const dm::float3& lower, int dimx, int dimy, int dimz, float radius,
                        const dm::float3& velocity, float invMass, bool rigid, float rigidStiffness, int phase,
                        float jitter) {
    if (rigid && buffers.rigidIndices.empty())
        buffers.rigidOffsets.push_back(0);

    for (int x = 0; x < dimx; ++x) {
        for (int y = 0; y < dimy; ++y) {
            for (int z = 0; z < dimz; ++z) {
                if (rigid)
                    buffers.rigidIndices.push_back(int(buffers.positions.size()));

                const dm::float3 position =
                    lower + dm::float3(float(x), float(y), float(z)) * radius + RandomUnitVector() * jitter;

                buffers.positions.push_back(dm::float4(position, invMass));
                buffers.velocities.push_back(velocity);
                buffers.phases.push_back(phase);
            }
        }
    }

    if (rigid) {
        buffers.rigidCoefficients.push_back(rigidStiffness);
        buffers.rigidOffsets.push_back(int(buffers.rigidIndices.size()));
    }
}

void CreateParticleSphere(SimBuffers& buffers, const dm::float3& center, int dim, float radius,
                          const dm::float3& velocity, float invMass, bool rigid, float rigidStiffness, int phase,
                          float jitter) {
    if (rigid && buffers.rigidIndices.empty())
        buffers.rigidOffsets.push_back(0);

    for (int x = 0; x <= dim; ++x) {
        for (int y = 0; y <= dim; ++y) {
            for (int z = 0; z <= dim; ++z) {
                const float sx = float(x) - float(dim) * 0.5f;
                const float sy = float(y) - float(dim) * 0.5f;
                const float sz = float(z) - float(dim) * 0.5f;

                if (sx * sx + sy * sy + sz * sz <= float(dim * dim) * 0.25f) {
                    if (rigid)
                        buffers.rigidIndices.push_back(int(buffers.positions.size()));

                    const dm::float3 position = center + radius * dm::float3(sx, sy, sz) + RandomUnitVector() * jitter;

                    buffers.positions.push_back(dm::float4(position, invMass));
                    buffers.velocities.push_back(velocity);
                    buffers.phases.push_back(phase);
                }
            }
        }
    }

    if (rigid) {
        buffers.rigidCoefficients.push_back(rigidStiffness);
        buffers.rigidOffsets.push_back(int(buffers.rigidIndices.size()));
    }
}

void CreateSpring(SimBuffers& buffers, int i, int j, float stiffness, float give) {
    buffers.springIndices.push_back(i);
    buffers.springIndices.push_back(j);
    buffers.springLengths.push_back((1.0f + give) * dm::length(xyz(buffers.positions[i]) - xyz(buffers.positions[j])));
    buffers.springStiffness.push_back(stiffness);
}

void CreateParticleShape(SimBuffers& buffers, const Mesh* srcMesh, const dm::float3& lower, const dm::float3& scale,
                         float rotation, float spacing, const dm::float3& velocity, float invMass, bool rigid,
                         float rigidStiffness, int phase, bool skin, float jitter, const dm::float3& skinOffset,
                         float skinExpand, const dm::float4& color, float springStiffness) {
    if (rigid && buffers.rigidIndices.empty())
        buffers.rigidOffsets.push_back(0);

    if (!srcMesh)
        return;

    // duplicate the mesh
    Mesh mesh;
    mesh.AddMesh(*srcMesh);

    const int startIndex = int(buffers.positions.size());

    dm::float3 meshLower, meshUpper;
    {
        mesh.Transform(dm::rotation(dm::float3(0.0f, 1.0f, 0.0f), rotation));

        mesh.GetBounds(meshLower, meshUpper);

        dm::float3 edges = meshUpper - meshLower;
        float maxEdge = std::max(std::max(edges.x, edges.y), edges.z);

        // the mesh at the origin, scaled to the size asked for (FleX: ScaleMatrix(scale / maxEdge) *
        // TranslationMatrix(-meshLower), the translation first)
        mesh.Transform(dm::translation(-meshLower) * dm::scaling(scale / maxEdge));
        mesh.GetBounds(meshLower, meshUpper);

        // the edges again, after the scale
        edges = meshUpper - meshLower;
        maxEdge = std::max(std::max(edges.x, edges.y), edges.z);

        // a spacing a little smaller, so that an edge that is a whole multiple of it does not put particles on
        // the boundary
        const float spacingEps = spacing * (1.0f - 1e-4f);

        // at least one particle in each dimension
        const int dx = spacing > edges.x ? 1 : int(edges.x / spacingEps);
        const int dy = spacing > edges.y ? 1 : int(edges.y / spacingEps);
        const int dz = spacing > edges.z ? 1 : int(edges.z / spacingEps);

        int maxDim = std::max(std::max(dx, dy), dz);

        // two voxels of border, for the sampling at the edges
        meshLower -= 2.0f * dm::float3(spacing);
        meshUpper += 2.0f * dm::float3(spacing);
        maxDim += 4;

        std::vector<uint32_t> voxels(size_t(maxDim) * maxDim * maxDim);

        // the voxelisation bounds shifted so that the voxel centres lie symmetrically about the centre of the
        // object: fewer missed features, particles better aligned with the mesh
        dm::float3 meshOffset;
        meshOffset.x = 0.5f * (spacing - (edges.x - float(dx - 1) * spacing));
        meshOffset.y = 0.5f * (spacing - (edges.y - float(dy - 1) * spacing));
        meshOffset.z = 0.5f * (spacing - (edges.z - float(dz - 1) * spacing));
        meshLower -= meshOffset;

        utils::Voxelize(mesh.m_positions.data(), int(mesh.m_positions.size()),
                        reinterpret_cast<const int*>(mesh.m_indices.data()), int(mesh.m_indices.size()),
                        uint32_t(maxDim), uint32_t(maxDim), uint32_t(maxDim), voxels.data(), meshLower,
                        meshLower + dm::float3(float(maxDim) * spacing));

        std::vector<int> indices(size_t(maxDim) * maxDim * maxDim);
        std::vector<float> sdf(size_t(maxDim) * maxDim * maxDim);
        utils::MakeSDF(voxels.data(), uint32_t(maxDim), uint32_t(maxDim), uint32_t(maxDim), sdf.data());

        for (int x = 0; x < maxDim; ++x) {
            for (int y = 0; y < maxDim; ++y) {
                for (int z = 0; z < maxDim; ++z) {
                    const int index = z * maxDim * maxDim + y * maxDim + x;

                    // a particle per occupied voxel
                    if (voxels[index]) {
                        if (rigid)
                            buffers.rigidIndices.push_back(int(buffers.positions.size()));

                        const dm::float3 position = lower + meshLower +
                                                    spacing * dm::float3(float(x) + 0.5f, float(y) + 0.5f, float(z) + 0.5f) +
                                                    RandomUnitVector() * jitter;

                        // the distance field normalised and in world scale
                        const dm::float3 n = safeNormalize(sampleSDFGrad(sdf.data(), maxDim, x, y, z));
                        const float d = sdf[index] * maxEdge;

                        if (rigid)
                            buffers.rigidLocalNormals.push_back(dm::float4(n, d));

                        // which particle is in which cell
                        indices[index] = int(buffers.positions.size());

                        buffers.positions.push_back(dm::float4(position, invMass));
                        buffers.velocities.push_back(velocity);
                        buffers.phases.push_back(phase);
                    }
                }
            }
        }
        mesh.Transform(dm::translation(-0.5f * (meshUpper + meshLower)) * dm::scaling(dm::float3(1.0f + skinExpand)));
        mesh.Transform(dm::translation(lower + 0.5f * (meshUpper + meshLower)));

        if (springStiffness > 0.0f) {
            // cross-link springs between occupied cells
            for (int x = 0; x < maxDim; ++x) {
                for (int y = 0; y < maxDim; ++y) {
                    for (int z = 0; z < maxDim; ++z) {
                        const int centerCell = z * maxDim * maxDim + y * maxDim + x;

                        if (voxels[centerCell]) {
                            const int width = 1;

                            // springs to every neighbour within the width
                            for (int i = x - width; i <= x + width; ++i) {
                                for (int j = y - width; j <= y + width; ++j) {
                                    for (int k = z - width; k <= z + width; ++k) {
                                        const int neighborCell = k * maxDim * maxDim + j * maxDim + i;

                                        if (neighborCell > 0 && neighborCell < int(voxels.size()) &&
                                            voxels[neighborCell] && neighborCell != centerCell) {
                                            CreateSpring(buffers, indices[neighborCell], indices[centerCell],
                                                         springStiffness);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (skin) {
        buffers.rigidMeshSize.push_back(int(mesh.GetNumVertices()));

        if (!buffers.mesh)
            buffers.mesh = std::make_unique<Mesh>();

        // append to the render mesh
        const uint32_t startVertex = buffers.mesh->GetNumVertices();

        buffers.mesh->Transform(dm::translation(skinOffset));
        buffers.mesh->AddMesh(mesh);

        static const dm::float4 colors[7] = {
            dm::float4(0.0f, 0.5f, 1.0f, 1.0f),   dm::float4(0.797f, 0.354f, 0.000f, 1.0f),
            dm::float4(0.000f, 0.349f, 0.173f, 1.0f), dm::float4(0.875f, 0.782f, 0.051f, 1.0f),
            dm::float4(0.01f, 0.170f, 0.453f, 1.0f),  dm::float4(0.673f, 0.111f, 0.000f, 1.0f),
            dm::float4(0.612f, 0.194f, 0.394f, 1.0f)};

        for (uint32_t i = startVertex; i < buffers.mesh->GetNumVertices(); ++i) {
            int indices[c_NumSkinWeights] = {-1, -1, -1, -1};
            float distances[c_NumSkinWeights] = {FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX};

            // FleX's Colour * float scales all four channels, alpha included
            if (dm::dot(color, color) == 0.0f)
                buffers.mesh->m_colours[i] = 1.25f * colors[(unsigned int)(phase) % 7];
            else
                buffers.mesh->m_colours[i] = color;

            // the closest 4 particles
            for (int j = startIndex; j < int(buffers.positions.size()); ++j) {
                const dm::float3 delta = buffers.mesh->m_positions[i] - xyz(buffers.positions[j]);
                const float dSq = dm::dot(delta, delta);

                // insertion sort
                int w = 0;
                for (; w < 4; ++w)
                    if (dSq < distances[w])
                        break;

                if (w < 4) {
                    // shuffle down
                    for (int s = 3; s > w; --s) {
                        indices[s] = indices[s - 1];
                        distances[s] = distances[s - 1];
                    }

                    distances[w] = dSq;
                    indices[w] = j;
                }
            }

            // weights by distance
            float wSum = 0.0f;

            for (int w = 0; w < 4; ++w) {
                // to an inverse distance
                distances[w] = 1.0f / (0.1f + powf(distances[w], .125f));

                wSum += distances[w];
            }

            float weights[4];
            for (int w = 0; w < 4; ++w)
                weights[w] = distances[w] / wSum;

            for (int j = 0; j < 4; ++j) {
                buffers.meshSkinIndices.push_back(indices[j]);
                buffers.meshSkinWeights.push_back(weights[j]);
            }
        }
    }

    if (rigid) {
        buffers.rigidCoefficients.push_back(rigidStiffness);
        buffers.rigidOffsets.push_back(int(buffers.rigidIndices.size()));
    }
}

void CreateParticleShape(SimBuffers& buffers, const std::filesystem::path& filename, const dm::float3& lower,
                         const dm::float3& scale, float rotation, float spacing, const dm::float3& velocity,
                         float invMass, bool rigid, float rigidStiffness, int phase, bool skin, float jitter,
                         const dm::float3& skinOffset, float skinExpand, const dm::float4& color,
                         float springStiffness) {
    std::unique_ptr<Mesh> mesh = utils::ImportMesh(filename);
    if (mesh)
        CreateParticleShape(buffers, mesh.get(), lower, scale, rotation, spacing, velocity, invMass, rigid,
                            rigidStiffness, phase, skin, jitter, skinOffset, skinExpand, color, springStiffness);
}

void SkinMesh(SimBuffers& buffers) {
    if (!buffers.mesh)
        return;

    uint32_t startVertex = 0;

    for (size_t r = 0; r < buffers.rigidRotations.size(); ++r) {
        dm::float3 rotation[3];
        QuatToMatrix33(buffers.rigidRotations[r], rotation);
        const uint32_t numVertices = uint32_t(buffers.rigidMeshSize[r]);

        for (uint32_t i = startVertex; i < numVertices + startVertex; ++i) {
            dm::float3 skinPos(0.0f);

            for (int w = 0; w < 4; ++w) {
                // a small shape can have fewer than 4 particles
                if (buffers.meshSkinIndices[i * 4 + w] > -1) {
                    assert(buffers.meshSkinWeights[i * 4 + w] < FLT_MAX);

                    const int index = buffers.meshSkinIndices[i * 4 + w];
                    const float weight = buffers.meshSkinWeights[i * 4 + w];

                    const dm::float3 rest = buffers.meshRestPositions[i] - xyz(buffers.restPositions[index]);
                    const dm::float3 rotated = rotation[0] * rest.x + rotation[1] * rest.y + rotation[2] * rest.z;
                    skinPos += (rotated + xyz(buffers.positions[index])) * weight;
                }
            }

            buffers.mesh->m_positions[i] = skinPos;
        }

        startVertex += numVertices;
    }

    buffers.mesh->CalculateNormals();
}

void CreateSpringGrid(SimBuffers& buffers, const dm::float3& lower, int dx, int dy, int dz, float radius, int phase,
                      float stretchStiffness, float bendStiffness, float shearStiffness, const dm::float3& velocity,
                      float invMass) {
    const int baseIndex = int(buffers.positions.size());
    auto gridIndex = [](int x, int y, int width) { return y * width + x; };

    for (int z = 0; z < dz; ++z) {
        for (int y = 0; y < dy; ++y) {
            for (int x = 0; x < dx; ++x) {
                const dm::float3 position = lower + radius * dm::float3(float(x), float(z), float(y));

                buffers.positions.push_back(dm::float4(position, invMass));
                buffers.velocities.push_back(velocity);
                buffers.phases.push_back(phase);

                if (x > 0 && y > 0) {
                    buffers.triangles.push_back(baseIndex + gridIndex(x - 1, y - 1, dx));
                    buffers.triangles.push_back(baseIndex + gridIndex(x, y - 1, dx));
                    buffers.triangles.push_back(baseIndex + gridIndex(x, y, dx));

                    buffers.triangles.push_back(baseIndex + gridIndex(x - 1, y - 1, dx));
                    buffers.triangles.push_back(baseIndex + gridIndex(x, y, dx));
                    buffers.triangles.push_back(baseIndex + gridIndex(x - 1, y, dx));

                    buffers.triangleNormals.push_back(dm::float3(0.0f, 1.0f, 0.0f));
                    buffers.triangleNormals.push_back(dm::float3(0.0f, 1.0f, 0.0f));
                }
            }
        }
    }

    // horizontal
    for (int y = 0; y < dy; ++y) {
        for (int x = 0; x < dx; ++x) {
            const int index0 = y * dx + x;

            if (x > 0) {
                const int index1 = y * dx + x - 1;
                CreateSpring(buffers, baseIndex + index0, baseIndex + index1, stretchStiffness);
            }

            if (x > 1) {
                const int index2 = y * dx + x - 2;
                CreateSpring(buffers, baseIndex + index0, baseIndex + index2, bendStiffness);
            }

            if (y > 0 && x < dx - 1) {
                const int indexDiag = (y - 1) * dx + x + 1;
                CreateSpring(buffers, baseIndex + index0, baseIndex + indexDiag, shearStiffness);
            }

            if (y > 0 && x > 0) {
                const int indexDiag = (y - 1) * dx + x - 1;
                CreateSpring(buffers, baseIndex + index0, baseIndex + indexDiag, shearStiffness);
            }
        }
    }

    // vertical
    for (int x = 0; x < dx; ++x) {
        for (int y = 0; y < dy; ++y) {
            const int index0 = y * dx + x;

            if (y > 0) {
                const int index1 = (y - 1) * dx + x;
                CreateSpring(buffers, baseIndex + index0, baseIndex + index1, stretchStiffness);
            }

            if (y > 1) {
                const int index2 = (y - 2) * dx + x;
                CreateSpring(buffers, baseIndex + index0, baseIndex + index2, bendStiffness);
            }
        }
    }
}

int PickParticle(const dm::float3& origin, const dm::float3& dir, const dm::float4* particles, const int* phases,
                 int n, float radius, float& outT) {
    const float maxDistSq = radius * radius;
    float minT = FLT_MAX;
    int minIndex = -1;

    for (int i = 0; i < n; ++i) {
        if (phases[i] & PhaseFluid)
            continue;

        const dm::float3 delta = xyz(particles[i]) - origin;
        const float t = dm::dot(delta, dir);

        if (t > 0.0f) {
            const dm::float3 perp = delta - t * dir;

            const float dSq = dm::dot(perp, perp);

            if (dSq < maxDistSq && t < minT) {
                minT = t;
                minIndex = i;
            }
        }
    }

    outT = minT;

    return minIndex;
}

void CalculateRigidCentersOfMass(const dm::float4* restPositions, int numRestPositions, const int* offsets,
                                 dm::float3* translations, const int* indices, int numRigids) {
    // relative to the mean of all positions, for accuracy: far from the origin, absolute coordinates would give
    // ghost forces
    dm::float3 shapeOffset(0.0f);

    for (int i = 0; i < numRestPositions; i++)
        shapeOffset += xyz(restPositions[i]);

    shapeOffset /= float(numRestPositions);

    for (int i = 0; i < numRigids; ++i) {
        const int startIndex = offsets[i];
        const int endIndex = offsets[i + 1];

        const int n = endIndex - startIndex;

        assert(n);

        dm::float3 com(0.0f);

        for (int j = startIndex; j < endIndex; ++j) {
            const int r = indices[j];

            com += xyz(restPositions[r]) - shapeOffset;
        }

        com /= float(n);

        // back to absolute coordinates
        com += shapeOffset;

        translations[i] = com;
    }
}

void CalculateRigidLocalPositions(const dm::float4* restPositions, const int* offsets,
                                  const dm::float3* translations, const int* indices, int numRigids,
                                  dm::float3* localPositions) {
    int count = 0;

    for (int i = 0; i < numRigids; ++i) {
        const int startIndex = offsets[i];
        const int endIndex = offsets[i + 1];

        assert(endIndex - startIndex);

        for (int j = startIndex; j < endIndex; ++j) {
            const int r = indices[j];

            localPositions[count++] = xyz(restPositions[r]) - translations[i];
        }
    }
}

// ---- Collision shapes ----------------------------------------------------------------------------------------

void AddBox(SimBuffers& buffers, const dm::float3& halfEdge, const dm::float3& center, const dm::float4& quat,
            bool dynamic, int channels) {
    ShapeDesc& shape = addShape(buffers, CollisionShapeType::Box, center, quat);
    shape.halfExtents = halfEdge;
    shape.dynamic = dynamic;
    shape.channels = channels & PhaseShapeChannelMask;
}

void AddPlinth(SimBuffers& buffers) {
    dm::float3 lower, upper;
    GetParticleBounds(buffers, lower, upper);

    dm::float3 center = (lower + upper) * 0.5f;
    center.y = 0.5f;

    AddBox(buffers, dm::float3(2.0f, 0.5f, 2.0f), center);
}

void AddSphere(SimBuffers& buffers, float radius, const dm::float3& position, const dm::float4& rotation) {
    ShapeDesc& shape = addShape(buffers, CollisionShapeType::Sphere, position, rotation);
    shape.radius = radius;
}

void AddCapsule(SimBuffers& buffers, float radius, float halfHeight, const dm::float3& position,
                const dm::float4& rotation) {
    ShapeDesc& shape = addShape(buffers, CollisionShapeType::Capsule, position, rotation);
    shape.radius = radius;
    shape.halfHeight = halfHeight;
}

void AddRandomConvex(SimBuffers& buffers, int numPlanes, const dm::float3& position, float minDist, float maxDist,
                     const dm::float3& axis, float angle) {
    const int maxPlanes = 12;

    // a 12-DOP
    const dm::float3 directions[maxPlanes] = {
        dm::float3(1.0f, 0.0f, 0.0f),  dm::float3(0.0f, 1.0f, 0.0f),   dm::float3(0.0f, 0.0f, 1.0f),
        dm::float3(-1.0f, 0.0f, 0.0f), dm::float3(0.0f, -1.0f, 0.0f),  dm::float3(0.0f, 0.0f, -1.0f),
        dm::float3(1.0f, 1.0f, 0.0f),  dm::float3(-1.0f, -1.0f, 0.0f), dm::float3(1.0f, 0.0f, 1.0f),
        dm::float3(-1.0f, 0.0f, -1.0f), dm::float3(0.0f, 1.0f, 1.0f),  dm::float3(0.0f, -1.0f, -1.0f),
    };

    // FleX's Clamp(6, numPlanes, maxPlanes), with its swap of the bounds, is numPlanes clamped to [6, 12]
    numPlanes = std::min(std::max(6, numPlanes), maxPlanes);

    std::vector<dm::float4> planes;
    planes.reserve(size_t(numPlanes));

    // the planes, each at a random distance
    for (int i = 0; i < numPlanes; ++i)
        planes.push_back(dm::float4(dm::normalize(directions[i]), -Randf(minDist, maxDist)));

    ShapeDesc& shape = addShape(buffers, CollisionShapeType::ConvexMesh, position, QuatFromAxisAngle(axis, angle));

    // the bounds
    utils::ConvexMeshBuilder builder(planes.data());
    builder(uint32_t(numPlanes));

    dm::float3 lower(FLT_MAX), upper(-FLT_MAX);
    for (const dm::float3& p : builder.mVertices) {
        lower = dm::min(lower, p);
        upper = dm::max(upper, p);
    }

    shape.convexPlanes = std::move(planes);
    shape.convexLower = lower;
    shape.convexUpper = upper;
    shape.scale = dm::float3(1.0f);

    // the render mesh of the convex: flat shaded, three vertices per face
    auto renderMesh = std::make_shared<Mesh>();

    for (size_t j = 0; j < builder.mIndices.size(); j += 3) {
        const uint32_t a = builder.mIndices[j + 0];
        const uint32_t b = builder.mIndices[j + 1];
        const uint32_t c = builder.mIndices[j + 2];

        const dm::float3 n = dm::normalize(
            dm::cross(builder.mVertices[b] - builder.mVertices[a], builder.mVertices[c] - builder.mVertices[a]));

        const uint32_t startIndex = uint32_t(renderMesh->m_positions.size());

        renderMesh->m_positions.push_back(builder.mVertices[a]);
        renderMesh->m_normals.push_back(n);

        renderMesh->m_positions.push_back(builder.mVertices[b]);
        renderMesh->m_normals.push_back(n);

        renderMesh->m_positions.push_back(builder.mVertices[c]);
        renderMesh->m_normals.push_back(n);

        renderMesh->m_indices.push_back(startIndex + 0);
        renderMesh->m_indices.push_back(startIndex + 1);
        renderMesh->m_indices.push_back(startIndex + 2);
    }

    shape.renderMesh = std::move(renderMesh);
}

void ClearShapes(SimBuffers& buffers) {
    buffers.shapes.clear();
    buffers.shapesChanged = true;
}

void UpdateShapes(SimBuffers& buffers) { buffers.shapesChanged = true; }

void GetShapeBounds(const SimBuffers& buffers, dm::float3& totalLower, dm::float3& totalUpper) {
    totalLower = dm::float3(FLT_MAX);
    totalUpper = dm::float3(-FLT_MAX);

    for (const ShapeDesc& shape : buffers.shapes) {
        dm::float3 localLower(0.0f);
        dm::float3 localUpper(0.0f);

        switch (shape.type) {
            case CollisionShapeType::Box:
                localLower = -shape.halfExtents;
                localUpper = shape.halfExtents;
                break;
            case CollisionShapeType::Sphere:
                localLower = dm::float3(-shape.radius);
                localUpper = dm::float3(shape.radius);
                break;
            case CollisionShapeType::Capsule:
                localLower = -dm::float3(shape.halfHeight, 0.0f, 0.0f) - dm::float3(shape.radius);
                localUpper = dm::float3(shape.halfHeight, 0.0f, 0.0f) + dm::float3(shape.radius);
                break;
            case CollisionShapeType::ConvexMesh:
                // with the instance scale
                localLower = shape.convexLower * shape.scale;
                localUpper = shape.convexUpper * shape.scale;
                break;
            case CollisionShapeType::TriangleMesh:
                if (shape.triangleMesh) {
                    // with the instance scaling
                    localLower = shape.triangleMesh->lower * shape.scale;
                    localUpper = shape.triangleMesh->upper * shape.scale;
                }
                break;
            case CollisionShapeType::Sdf:
                localLower = dm::float3(0.0f);
                localUpper = dm::float3(shape.sdfScale);
                break;
            default:
                break;
        }

        // the local bounds to world space (FleX: TransformBounds)
        dm::float3 cols[3];
        QuatToMatrix33(shape.rotation, cols);

        const dm::float3 extents = localUpper - localLower;
        cols[0] *= extents.x;
        cols[1] *= extents.y;
        cols[2] *= extents.z;

        const float ex = fabsf(cols[0].x) + fabsf(cols[1].x) + fabsf(cols[2].x);
        const float ey = fabsf(cols[0].y) + fabsf(cols[1].y) + fabsf(cols[2].y);
        const float ez = fabsf(cols[0].z) + fabsf(cols[1].z) + fabsf(cols[2].z);

        const dm::float3 center = (localUpper + localLower) * 0.5f;
        const dm::float3 worldCenter = QuatRotate(shape.rotation, center) + shape.position;

        totalLower = dm::min(totalLower, worldCenter - dm::float3(ex, ey, ez) * 0.5f);
        totalUpper = dm::max(totalUpper, worldCenter + dm::float3(ex, ey, ez) * 0.5f);
    }
}

// ---- Ropes, tetrahedral meshes, random bodies --------------------------------------------------------------

void CreateRope(SimBuffers& buffers, Rope& rope, const dm::float3& start, const dm::float3& dirIn, float stiffness,
                int segments, float length, int phase, float spiralAngle, float invmass, float give) {
    dm::float3 dir = dirIn;

    rope.mIndices.push_back(int(buffers.positions.size()));

    buffers.positions.push_back(dm::float4(start, invmass));
    buffers.velocities.push_back(dm::float3(0.0f));
    buffers.phases.push_back(phase);

    dm::float3 left, right;
    utils::BasisFromVector(dir, left, right);

    const float segmentLength = length / float(segments);
    const dm::float3 spiralAxis = dir;
    const float spiralHeight = spiralAngle / (2.0f * kPi) * (length / float(segments));

    if (spiralAngle > 0.0f)
        dir = left;

    dm::float3 p = start;

    for (int i = 0; i < segments; ++i) {
        const int prev = int(buffers.positions.size()) - 1;

        p += dir * segmentLength;

        // rotate
        if (spiralAngle > 0.0f) {
            p += spiralAxis * spiralHeight;

            dir = dm::rotation(spiralAxis, spiralAngle).transformVector(dir);
        }

        rope.mIndices.push_back(int(buffers.positions.size()));

        buffers.positions.push_back(dm::float4(p, 1.0f));
        buffers.velocities.push_back(dm::float3(0.0f));
        buffers.phases.push_back(phase);

        // stretch
        CreateSpring(buffers, prev, prev + 1, stiffness, give);

        // bending spring
        if (i > 0)
            CreateSpring(buffers, prev - 1, prev + 1, stiffness * 0.5f, give);
    }
}

namespace {

// A face of a tetrahedron, keyed by its sorted vertices (FleX's TriKey), with its original winding.
struct TriKey {
    int orig[3];
    int indices[3];

    TriKey(int a, int b, int c) {
        orig[0] = a;
        orig[1] = b;
        orig[2] = c;

        indices[0] = a;
        indices[1] = b;
        indices[2] = c;

        std::sort(indices, indices + 3);
    }

    bool operator<(const TriKey& rhs) const {
        if (indices[0] != rhs.indices[0])
            return indices[0] < rhs.indices[0];
        else if (indices[1] != rhs.indices[1])
            return indices[1] < rhs.indices[1];
        else
            return indices[2] < rhs.indices[2];
    }
};

}  // namespace

void CreateTetMesh(SimBuffers& buffers, const std::filesystem::path& filename, const dm::float3& lower, float scale,
                   float stiffness, int phase) {
    FILE* f = nullptr;
    if (_wfopen_s(&f, filename.c_str(), L"r") != 0 || !f)
        return;

    char line[2048];

    std::map<TriKey, int> triCount;

    const int vertOffset = int(buffers.positions.size());

    dm::float3 meshLower(FLT_MAX);
    dm::float3 meshUpper(-FLT_MAX);

    bool firstTet = true;

    while (!feof(f)) {
        if (fgets(line, 2048, f)) {
            switch (line[0]) {
                case '#':
                    break;
                case 'v': {
                    dm::float3 pos(0.0f);
                    sscanf_s(line, "v %f %f %f", &pos.x, &pos.y, &pos.z);

                    buffers.positions.push_back(dm::float4(pos, 1.0f));
                    buffers.velocities.push_back(dm::float3(0.0f));
                    buffers.phases.push_back(phase);

                    meshLower = dm::min(pos, meshLower);
                    meshUpper = dm::max(pos, meshUpper);
                    break;
                }
                case 't': {
                    if (firstTet) {
                        const dm::float3 edges = meshUpper - meshLower;
                        const float maxEdge = std::max(edges.x, std::max(edges.y, edges.z));

                        // normalize positions
                        for (int i = vertOffset; i < int(buffers.positions.size()); ++i) {
                            const dm::float3 p =
                                lower + (xyz(buffers.positions[size_t(i)]) - meshLower) * scale / maxEdge;
                            buffers.positions[size_t(i)] = dm::float4(p, buffers.positions[size_t(i)].w);
                        }

                        firstTet = false;
                    }

                    int indices[4] = {0, 0, 0, 0};
                    sscanf_s(line, "t %d %d %d %d", &indices[0], &indices[1], &indices[2], &indices[3]);

                    indices[0] += vertOffset;
                    indices[1] += vertOffset;
                    indices[2] += vertOffset;
                    indices[3] += vertOffset;

                    CreateSpring(buffers, indices[0], indices[1], stiffness);
                    CreateSpring(buffers, indices[0], indices[2], stiffness);
                    CreateSpring(buffers, indices[0], indices[3], stiffness);

                    CreateSpring(buffers, indices[1], indices[2], stiffness);
                    CreateSpring(buffers, indices[1], indices[3], stiffness);
                    CreateSpring(buffers, indices[2], indices[3], stiffness);

                    triCount[TriKey(indices[0], indices[2], indices[1])] += 1;
                    triCount[TriKey(indices[1], indices[2], indices[3])] += 1;
                    triCount[TriKey(indices[0], indices[1], indices[3])] += 1;
                    triCount[TriKey(indices[0], indices[3], indices[2])] += 1;
                    break;
                }
                default:
                    break;
            }
        }
    }

    for (const auto& [key, count] : triCount) {
        // only output faces that are referenced by one tet (open faces)
        if (count == 1) {
            buffers.triangles.push_back(key.orig[0]);
            buffers.triangles.push_back(key.orig[1]);
            buffers.triangles.push_back(key.orig[2]);
            buffers.triangleNormals.push_back(dm::float3(0.0f));
        }
    }

    fclose(f);
}

void CreateRandomBody(SimBuffers& buffers, int numPlanes, const dm::float3& position, float minDist, float maxDist,
                      const dm::float3& axis, float angle, float invMass, int phase, float stiffness) {
    // 12-kdop
    const dm::float3 directions[] = {
        dm::float3(1.0f, 0.0f, 0.0f),  dm::float3(0.0f, 1.0f, 0.0f),   dm::float3(0.0f, 0.0f, 1.0f),
        dm::float3(-1.0f, 0.0f, 0.0f), dm::float3(0.0f, -1.0f, 0.0f),  dm::float3(0.0f, 0.0f, -1.0f),
        dm::float3(1.0f, 1.0f, 0.0f),  dm::float3(-1.0f, -1.0f, 0.0f), dm::float3(1.0f, 0.0f, 1.0f),
        dm::float3(-1.0f, 0.0f, -1.0f), dm::float3(0.0f, 1.0f, 1.0f),  dm::float3(0.0f, -1.0f, -1.0f),
    };

    // FleX reads directions[i] for every i < numPlanes without a bound; more than 12 planes would read past the
    // table, so the count is clamped to it as well
    numPlanes = std::min(std::max(4, numPlanes), 12);

    std::vector<dm::float4> planes;

    // create a box
    for (int i = 0; i < numPlanes; ++i) {
        // pick random dir and distance
        const dm::float3 dir = dm::normalize(directions[i]);
        const float dist = Randf(minDist, maxDist);

        planes.push_back(dm::float4(dir.x, dir.y, dir.z, -dist));
    }

    // set aabbs
    utils::ConvexMeshBuilder builder(planes.data());
    builder(uint32_t(numPlanes));

    const int startIndex = int(buffers.positions.size());
    const dm::float4 q = QuatFromAxisAngle(axis, angle);

    for (size_t v = 0; v < builder.mVertices.size(); ++v) {
        const dm::float3 p = QuatRotate(q, builder.mVertices[v]) + position;

        buffers.positions.push_back(dm::float4(p, invMass));
        buffers.velocities.push_back(dm::float3(0.0f));
        buffers.phases.push_back(phase);

        // add spring to all verts with higher index
        for (size_t i = v + 1; i < builder.mVertices.size(); ++i) {
            const int a = startIndex + int(v);
            const int b = startIndex + int(i);

            buffers.springIndices.push_back(a);
            buffers.springIndices.push_back(b);
            buffers.springLengths.push_back(dm::length(builder.mVertices[v] - builder.mVertices[i]));
            buffers.springStiffness.push_back(stiffness);
        }
    }

    for (size_t t = 0; t < builder.mIndices.size(); ++t)
        buffers.triangles.push_back(startIndex + int(builder.mIndices[t]));

    // lazy
    buffers.triangleNormals.resize(buffers.triangleNormals.size() + builder.mIndices.size() / 3, dm::float3(0.0f));
}

// ---- Soft body building blocks -------------------------------------------------------------------------------

dm::float3 CalculateMean(const dm::float3* particles, const int* indices, int numIndices) {
    dm::float3 sum(0.0f);

    for (int i = 0; i < numIndices; ++i)
        sum += particles[indices[i]];

    if (numIndices)
        return sum / float(numIndices);
    else
        return sum;
}

float CalculateRadius(const dm::float3* particles, const dm::float3& center, const int* indices, int numIndices) {
    float radiusSq = 0.0f;

    for (int i = 0; i < numIndices; ++i) {
        const float dSq = dm::lengthSquared(particles[indices[i]] - center);
        if (dSq > radiusSq)
            radiusSq = dSq;
    }

    return sqrtf(radiusSq);
}

namespace {

struct Cluster {
    dm::float3 mean = dm::float3(0.0f);
    float radius = 0.0f;

    // indices of particles belonging to this cluster
    std::vector<int> indices;
};

struct Seed {
    int index;
    float priority;

    bool operator<(const Seed& rhs) const { return priority < rhs.priority; }
};

}  // namespace

int CreateClusters(const dm::float3* particles, const float* priority, int numParticles,
                   std::vector<int>& outClusterOffsets, std::vector<int>& outClusterIndices,
                   std::vector<dm::float3>& outClusterPositions, float radius, float smoothing) {
    std::vector<Seed> seeds;
    std::vector<Cluster> clusters;

    // flags a particle as belonging to at least one cluster
    std::vector<bool> used(size_t(numParticles), false);

    // initialize seeds
    for (int i = 0; i < numParticles; ++i)
        seeds.push_back(Seed{i, priority[i]});

    std::stable_sort(seeds.begin(), seeds.end());

    while (!seeds.empty()) {
        // pick highest unused particle from the seeds list
        const Seed seed = seeds.back();
        seeds.pop_back();

        if (!used[size_t(seed.index)]) {
            Cluster c;

            const float radiusSq = radius * radius;

            // push all neighbors within radius
            for (int p = 0; p < numParticles; ++p) {
                const float dSq = dm::lengthSquared(particles[seed.index] - particles[p]);
                if (dSq <= radiusSq) {
                    c.indices.push_back(p);

                    used[size_t(p)] = true;
                }
            }

            c.mean = CalculateMean(particles, c.indices.data(), int(c.indices.size()));

            clusters.push_back(std::move(c));
        }
    }

    if (smoothing > 0.0f) {
        // expand clusters by smoothing radius
        const float radiusSmoothSq = smoothing * smoothing;

        for (Cluster& c : clusters) {
            // clear cluster indices
            c.indices.resize(0);

            // push all neighbors within radius
            for (int p = 0; p < numParticles; ++p) {
                const float dSq = dm::lengthSquared(c.mean - particles[p]);
                if (dSq <= radiusSmoothSq)
                    c.indices.push_back(p);
            }

            c.mean = CalculateMean(particles, c.indices.data(), int(c.indices.size()));
        }
    }

    // write out cluster indices
    int count = 0;

    for (const Cluster& cluster : clusters) {
        const int clusterSize = int(cluster.indices.size());

        // skip empty clusters
        if (clusterSize) {
            // write cluster indices
            for (int i = 0; i < clusterSize; ++i)
                outClusterIndices.push_back(cluster.indices[size_t(i)]);

            // write cluster offset
            outClusterOffsets.push_back(int(outClusterIndices.size()));

            // write center
            outClusterPositions.push_back(cluster.mean);

            ++count;
        }
    }

    return count;
}

int CreateLinks(const dm::float3* particles, int numParticles, std::vector<int>& outSpringIndices,
                std::vector<float>& outSpringLengths, std::vector<float>& outSpringStiffness, float radius,
                float stiffness) {
    const float radiusSq = radius * radius;

    int count = 0;

    for (int i = 0; i < numParticles; ++i) {
        for (int j = i + 1; j < numParticles; ++j) {
            const float dSq = dm::lengthSquared(particles[i] - particles[j]);

            if (dSq < radiusSq) {
                outSpringIndices.push_back(i);
                outSpringIndices.push_back(j);
                outSpringLengths.push_back(sqrtf(dSq));
                outSpringStiffness.push_back(stiffness);

                ++count;
            }
        }
    }

    return count;
}

void CreateSkinning(const dm::float3* vertices, int numVertices, const dm::float3* clusters, int numClusters,
                    float* outWeights, int* outIndices, float falloff, float maxdist) {
    const int maxBones = 4;

    // for each vertex, find the closest n clusters
    for (int i = 0; i < numVertices; ++i) {
        int indices[4] = {-1, -1, -1, -1};
        float distances[4] = {FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX};
        float weights[maxBones];

        for (int c = 0; c < numClusters; ++c) {
            const float dSq = dm::lengthSquared(vertices[i] - clusters[c]);

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
                indices[w] = c;
            }
        }

        // weight particles according to distance
        float wSum = 0.0f;

        for (int w = 0; w < maxBones; ++w) {
            if (distances[w] > maxdist * maxdist) {
                // clamp bones over a given distance to zero
                weights[w] = 0.0f;
            } else {
                // weight falls off inversely with distance
                weights[w] = 1.0f / (powf(distances[w], falloff) + 0.0001f);
            }

            wSum += weights[w];
        }

        if (wSum == 0.0f) {
            // if all weights are zero then just rigidly skin to the closest bone
            weights[0] = 1.0f;
        } else {
            // normalize weights
            for (int w = 0; w < maxBones; ++w)
                weights[w] = weights[w] / wSum;
        }

        // output
        for (int j = 0; j < maxBones; ++j) {
            outWeights[i * maxBones + j] = weights[j];
            outIndices[i * maxBones + j] = indices[j];
        }
    }
}

void SampleMesh(Mesh* mesh, const dm::float3& lower, const dm::float3& scale, float rotation, float radius,
                float volumeSampling, float surfaceSampling, std::vector<dm::float3>& outPositions) {
    if (!mesh)
        return;

    mesh->Transform(dm::rotation(dm::float3(0.0f, 1.0f, 0.0f), rotation));

    dm::float3 meshLower, meshUpper;
    mesh->GetBounds(meshLower, meshUpper);

    dm::float3 edges = meshUpper - meshLower;
    float maxEdge = std::max(std::max(edges.x, edges.y), edges.z);

    // put mesh at the origin and scale to specified size
    mesh->Transform(dm::translation(-meshLower) * dm::scaling(scale / maxEdge));
    mesh->GetBounds(meshLower, meshUpper);

    std::vector<dm::float3> samples;

    if (volumeSampling > 0.0f) {
        // recompute expanded edges
        edges = meshUpper - meshLower;
        maxEdge = std::max(std::max(edges.x, edges.y), edges.z);

        // use a higher resolution voxelization as a basis for the particle decomposition
        const float spacing = radius / volumeSampling;

        // tweak spacing to avoid edge cases for particles laying on the boundary
        // just covers the case where an edge is a whole multiple of the spacing.
        const float spacingEps = spacing * (1.0f - 1e-4f);

        // make sure to have at least one particle in each dimension
        const int dx = spacing > edges.x ? 1 : int(edges.x / spacingEps);
        const int dy = spacing > edges.y ? 1 : int(edges.y / spacingEps);
        const int dz = spacing > edges.z ? 1 : int(edges.z / spacingEps);

        int maxDim = std::max(std::max(dx, dy), dz);

        // expand border by two voxels to ensure adequate sampling at edges
        meshLower -= 2.0f * dm::float3(spacing);
        meshUpper += 2.0f * dm::float3(spacing);
        maxDim += 4;

        std::vector<uint32_t> voxels(size_t(maxDim) * size_t(maxDim) * size_t(maxDim));

        // we shift the voxelization bounds so that the voxel centers lie symmetrically to the center of the
        // object. this reduces the chance of missing features, and also better aligns the particles with the mesh
        dm::float3 meshOffset;
        meshOffset.x = 0.5f * (spacing - (edges.x - float(dx - 1) * spacing));
        meshOffset.y = 0.5f * (spacing - (edges.y - float(dy - 1) * spacing));
        meshOffset.z = 0.5f * (spacing - (edges.z - float(dz - 1) * spacing));
        meshLower -= meshOffset;

        const std::vector<int> indices(mesh->m_indices.begin(), mesh->m_indices.end());
        utils::Voxelize(mesh->m_positions.data(), int(mesh->m_positions.size()), indices.data(), int(indices.size()),
                        uint32_t(maxDim), uint32_t(maxDim), uint32_t(maxDim), voxels.data(), meshLower,
                        meshLower + dm::float3(float(maxDim) * spacing));

        // sample interior
        for (int x = 0; x < maxDim; ++x) {
            for (int y = 0; y < maxDim; ++y) {
                for (int z = 0; z < maxDim; ++z) {
                    const int index = z * maxDim * maxDim + y * maxDim + x;

                    // if voxel is marked as occupied the add a particle
                    if (voxels[size_t(index)]) {
                        const dm::float3 position =
                            lower + meshLower + spacing * dm::float3(float(x) + 0.5f, float(y) + 0.5f, float(z) + 0.5f);

                        samples.push_back(position);
                    }
                }
            }
        }
    }

    // move back
    mesh->Transform(dm::translation(-0.5f * (meshUpper + meshLower)));
    mesh->Transform(dm::translation(lower + 0.5f * (meshUpper + meshLower)));

    if (surfaceSampling > 0.0f) {
        // sample vertices
        for (const dm::float3& v : mesh->m_positions)
            samples.push_back(v);

        // random surface sampling
        for (int i = 0; i < 50000; ++i) {
            const uint32_t t = Rand() % mesh->GetNumFaces();
            const float u = Randf();
            const float v = Randf() * (1.0f - u);
            const float w = 1.0f - u - v;

            const uint32_t a = mesh->m_indices[t * 3 + 0];
            const uint32_t b = mesh->m_indices[t * 3 + 1];
            const uint32_t c = mesh->m_indices[t * 3 + 2];

            samples.push_back(mesh->m_positions[a] * u + mesh->m_positions[b] * v + mesh->m_positions[c] * w);
        }
    }

    std::vector<int> clusterIndices;
    std::vector<int> clusterOffsets;
    std::vector<float> priority(samples.size());

    CreateClusters(samples.data(), priority.data(), int(samples.size()), clusterOffsets, clusterIndices, outPositions,
                   radius);
}

// ---- Triangle meshes and distance fields ---------------------------------------------------------------------

TriangleMeshId CreateTriangleMesh(const Mesh* m) {
    if (!m)
        return nullptr;

    auto data = std::make_shared<TriangleMeshData>();
    data->mesh = *m;
    m->GetBounds(data->lower, data->upper);
    return data;
}

void AddTriangleMesh(SimBuffers& buffers, const TriangleMeshId& mesh, const dm::float3& translation,
                     const dm::float4& rotation, const dm::float3& scale) {
    ShapeDesc& shape = addShape(buffers, CollisionShapeType::TriangleMesh, translation, rotation);
    shape.triangleMesh = mesh;
    shape.scale = scale;
}

DistanceFieldId CreateSDF(const std::filesystem::path& meshFile, int dim, float margin, float expand) {
    std::unique_ptr<Mesh> mesh = utils::ImportMesh(meshFile);
    if (!mesh)
        return nullptr;

    // include small margin to ensure valid gradients near the boundary
    mesh->Normalize(1.0f - margin);
    mesh->Transform(dm::translation(dm::float3(margin, margin, margin) * 0.5f));

    const dm::float3 lower(0.0f);
    const dm::float3 upper(1.0f);

    auto data = std::make_shared<DistanceFieldData>();
    utils::PfmImage& pfm = data->field;

    // try and load the sdf from disc if it exists
    std::filesystem::path sdfFile = meshFile;
    sdfFile.replace_extension(".pfm");

    if (!utils::PfmLoad(sdfFile, pfm)) {
        pfm.m_width = uint32_t(dim);
        pfm.m_height = uint32_t(dim);
        pfm.m_depth = uint32_t(dim);
        pfm.m_data.assign(size_t(dim) * size_t(dim) * size_t(dim), 0.0f);

        std::printf("Cooking SDF: %s - dim: %d^3\n", sdfFile.string().c_str(), dim);

        // FleX's CreateSDF(mesh, dim, lower, upper, sdf)
        std::vector<uint32_t> volume(size_t(dim) * size_t(dim) * size_t(dim));
        const std::vector<int> indices(mesh->m_indices.begin(), mesh->m_indices.end());
        utils::Voxelize(mesh->m_positions.data(), int(mesh->m_positions.size()), indices.data(), int(indices.size()),
                        uint32_t(dim), uint32_t(dim), uint32_t(dim), volume.data(), lower, upper);
        utils::MakeSDF(volume.data(), uint32_t(dim), uint32_t(dim), uint32_t(dim), pfm.m_data.data());
    }

    assert(pfm.m_width == pfm.m_height && pfm.m_width == pfm.m_depth);

    // cheap collision offset
    for (float& d : pfm.m_data)
        d += expand;

    data->mesh = std::move(*mesh);
    return data;
}

void AddSDF(SimBuffers& buffers, const DistanceFieldId& sdf, const dm::float3& translation, const dm::float4& rotation,
            float width) {
    ShapeDesc& shape = addShape(buffers, CollisionShapeType::Sdf, translation, rotation);
    shape.sdf = sdf;
    shape.sdfScale = width;
}

}  // namespace ephys::helpers
