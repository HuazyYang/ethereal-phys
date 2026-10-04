#include <ethereal/phys_helpers/helpers.h>

#include <ethereal/phys_utils/convex.h>
#include <ethereal/phys_utils/voxelize.h>

#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>

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
            default:
                // triangle meshes and distance fields come with step 10b
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

}  // namespace ephys::helpers
