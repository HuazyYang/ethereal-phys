#include "KernelParams.hlsli"

#define BLOCK_DIM_X 256
#define BVH_STACK_SIZE 40
#define BVH_OVERFLOW_SIZE 24
#define MAX_TRIANGLE_SHAPES 8

struct PackedNodeHalf {
    float3 v;
    uint ib;
};

StructuredBuffer<float4> triangleVerticesArray : register(t0);
StructuredBuffer<int> triangleIndicesArray : register(t1);
StructuredBuffer<PackedNodeHalf> bvhNodeLowersArray : register(t2);
StructuredBuffer<PackedNodeHalf> bvhNodeUppersArray : register(t3);
StructuredBuffer<int> bvhRootNodeArray : register(t4);
StructuredBuffer<float4> sortedPositions : register(t5);
StructuredBuffer<float4> sortedNewPositions : register(t6);
StructuredBuffer<int> sortedPhases : register(t7);
StructuredBuffer<float4> shapePrevPositions : register(t8);
StructuredBuffer<float4> shapePrevRotations : register(t9);
StructuredBuffer<int> shapeFlags : register(t10);
StructuredBuffer<float4> shapePositions : register(t11);
StructuredBuffer<float4> shapeRotations : register(t12);
StructuredBuffer<float4> shapeGeometry : register(t13);
StructuredBuffer<int> shapeBvhRootNode : register(t14);
StructuredBuffer<PackedNodeHalf> shapeBvhNodeLowers : register(t15);
StructuredBuffer<PackedNodeHalf> shapeBvhNodeUppers : register(t16);
StructuredBuffer<FlexTriangleMeshDevice> meshOffsets : register(t17);

RWStructuredBuffer<int> collisionCounts : register(u0);
RWStructuredBuffer<float4> collisionPlanes : register(u1);
RWStructuredBuffer<float4> collisionVelocities : register(u2);

groupshared int triangleShapeCandidates[BLOCK_DIM_X * MAX_TRIANGLE_SHAPES];
groupshared int stackOverflow[BLOCK_DIM_X][BVH_OVERFLOW_SIZE];

// FXC compiles cross() to one three-wide mul plus one three-wide mad, fusing the
// same factor in every lane. The shipped DXBC computes the components
// separately, which fuses a different product per lane and so rounds differently.
float3 CrossExplicit(float3 a, float3 b) {
    float3 r;
    r.x = a.y * b.z - a.z * b.y;
    r.y = a.z * b.x - a.x * b.z;
    r.z = a.x * b.y - a.y * b.x;
    return r;
}

float3 Rotate(float4 q, float3 v) {
    return v * (2.0f * q.w * q.w - 1.0f) + CrossExplicit(q.xyz, v) * q.w * 2.0f + q.xyz * dot(q.xyz, v) * 2.0f;
}

float3 RotateInv(float4 q, float3 v) {
    return v * (2.0f * q.w * q.w - 1.0f) - CrossExplicit(q.xyz, v) * q.w * 2.0f + q.xyz * dot(q.xyz, v) * 2.0f;
}

float4 NormalizeQuat(float4 q) {
    return q * rsqrt(dot(q, q));
}

bool OverlapAabb(float3 lowerA, float3 upperA, float3 lowerB, float3 upperB) {
    bool3 a = upperA < lowerB;
    bool anyA = a.x || a.y || a.z;
    bool3 b = upperB < lowerA;
    return !(anyA || b.x || b.y || b.z);
}

float3 ClosestPointOnTriangle(float3 p, float3 a, float3 b, float3 c) {
    float3 ab = b - a;
    float3 ac = c - a;
    float3 ap = p - a;
    float d1 = dot(ab, ap);
    float d2 = dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
        return a;

    float3 bp = p - b;
    float d3 = dot(ab, bp);
    float d4 = dot(ac, bp);
    if (d3 >= 0.0f && d3 >= d4)
        return b;

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        return a + v * ab;
    }

    float3 cp = p - c;
    float d5 = dot(ab, cp);
    float d6 = dot(ac, cp);
    if (d6 >= 0.0f && d6 >= d5)
        return c;

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        return a + w * ac;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);
    }

    float invDenom = 1.0f / (va + vb + vc);
    float v = vb * invDenom;
    float w = vc * invDenom;
    return a + ab * v + ac * w;
}

bool SegmentIntersectsTriangle(float3 d, float3 start, float3 a, float3 b, float3 c) {
    float3 av = a - start;
    float3 bv = b - start;
    float3 cv = c - start;
    // cross(d, cv), not cross(cv, d): the DXBC puts the segment delta first.
    // Reversed, s0 and s1 invert while s2 does not, so the three signed volumes
    // no longer share a sign and the test can never fire -- tunneling contacts
    // are then silently dropped.
    float3 n0 = CrossExplicit(d, cv);
    bool s0 = dot(bv, n0) >= 0.0f;
    bool s1 = -dot(av, n0) >= 0.0f;
    float3 n1 = CrossExplicit(d, bv);
    bool s2 = dot(n1, av) >= 0.0f;
    return s0 && (s2 && s1);
}

void StoreTriangleContact(
    int particle,
    inout int count,
    int shape,
    int maxContacts,
    float3 shapeEnd,
    float4 qEnd,
    float4 velocity,
    float3 localNormal,
    float localPlaneW) {
    if (count < maxContacts) {
        int contactIndex = particle * maxContacts + count;
        float3 normal = Rotate(qEnd, localNormal);
        float planeW = -(localPlaneW + dot(normal, shapeEnd));
        collisionPlanes[contactIndex] = float4(normal, planeW);
        collisionVelocities[contactIndex] = velocity;
        count++;
    }
}

void TraverseShapeBvh(
    int particle,
    int localIdx,
    float3 particleStart,
    float3 particleEnd,
    inout int stack[BVH_STACK_SIZE],
    out int numTriangleShapes) {
    uint particleChannels = uint(sortedPhases[particle]) & 0xff000000u;
    float3 lower = min(particleStart, particleEnd);
    float3 upper = max(particleStart, particleEnd);
    float3 queryLower = lower - gParams.kCollisionMargin.xxx;
    float3 queryUpper = upper + gParams.kCollisionMargin.xxx;
    stack[0] = shapeBvhRootNode[0];
    int candidateBase = localIdx * MAX_TRIANGLE_SHAPES;
    int stackSize = 1;
    numTriangleShapes = 0;

    while (stackSize != 0) {
        int node;
        if (stackSize > BVH_STACK_SIZE) {
            stackSize--;
            node = stackOverflow[localIdx][stackSize - BVH_STACK_SIZE];
        } else {
            stackSize--;
            node = stack[stackSize];
        }

        PackedNodeHalf lowerNode = shapeBvhNodeLowers[node];
        PackedNodeHalf upperNode = shapeBvhNodeUppers[node];

        if (OverlapAabb(queryLower, queryUpper, lowerNode.v, upperNode.v)) {

            uint lowerBits = lowerNode.ib & 0x7fffffffu;
            if (lowerNode.ib & 0x80000000u) {
                int flags = shapeFlags[lowerBits];
                // Three nested gates, as the DXBC has them: one `and`/`if_nz` per test.
                if (uint(flags) & particleChannels) {
                    if ((uint(flags) & 7u) == eNvFlexShapeTriangleMesh) {
                        if (numTriangleShapes < MAX_TRIANGLE_SHAPES) {
                            triangleShapeCandidates[candidateBase + numTriangleShapes] = int(lowerBits);
                            numTriangleShapes++;
                        }
                    }
                }
            } else {
                int upperBits = int(upperNode.ib & 0x7fffffffu);
                int nextSize;
                if (stackSize >= BVH_STACK_SIZE) {
                    nextSize = stackSize + 1;
                    stackOverflow[localIdx][stackSize - BVH_STACK_SIZE] = int(lowerBits);
                } else {
                    nextSize = stackSize + 1;
                    stack[stackSize] = int(lowerBits);
                }

                if (nextSize >= BVH_STACK_SIZE) {
                    stackSize = nextSize + 1;
                    stackOverflow[localIdx][nextSize - BVH_STACK_SIZE] = upperBits;
                } else {
                    stackSize = nextSize + 1;
                    stack[nextSize] = upperBits;
                }
            }
        }
    }
}

void TraverseMeshBvh(
    int particle,
    int localIdx,
    int shape,
    float3 queryLower,
    float3 queryUpper,
    float3 localStart,
    float3 localEnd,
    float3 shapeEnd,
    float4 qEnd,
    float4 geometry,
    float3 shapeStart,
    float4 qStart,
    int meshId,
    inout int stack[BVH_STACK_SIZE],
    inout int count) {
    int stackSize = 1;
    stack[0] = bvhRootNodeArray[meshId];
    FlexTriangleMeshDevice mesh = meshOffsets[meshId];
    float3 segmentDelta = localEnd - localStart;
    float3 localEndAtStart = Rotate(qStart, localEnd) + shapeStart;
    float3 localEndAtEnd = Rotate(qEnd, localEnd) + shapeEnd;
    float4 velocity = float4((localEndAtEnd - localEndAtStart) * geometry.xyz, float(shape));

    while (stackSize != 0) {
        int node;
        if (stackSize > BVH_STACK_SIZE) {
            stackSize--;
            node = stackOverflow[localIdx][stackSize - BVH_STACK_SIZE];
        } else {
            stackSize--;
            node = stack[stackSize];
        }

        int nodeIndex = mesh.mNodeStart + node;
        PackedNodeHalf lowerNode = bvhNodeLowersArray[nodeIndex];
        PackedNodeHalf upperNode = bvhNodeUppersArray[nodeIndex];

        if (OverlapAabb(queryLower, queryUpper, lowerNode.v, upperNode.v)) {

            uint lowerBits = lowerNode.ib & 0x7fffffffu;
            if (lowerNode.ib & 0x80000000u) {
                int triIndex = int(lowerBits);
                int indexBase = mesh.mIndexStart + triIndex * 3;
                int t0 = triangleIndicesArray[indexBase];
                int2 nextIndex = indexBase + int2(1, 2);
                int t1 = triangleIndicesArray[nextIndex.x];
                int t2 = triangleIndicesArray[nextIndex.y];
                int i1 = mesh.mVertexStart + t1;
                int i0 = mesh.mVertexStart + t0;

                float3 va = triangleVerticesArray[i0].xyz * geometry.xyz;
                float3 vb = triangleVerticesArray[i1].xyz * geometry.xyz;
                int i2 = mesh.mVertexStart + t2;
                float3 vc = triangleVerticesArray[i2].xyz * geometry.xyz;
                float3 ab = vb - va;
                float3 ac = vc - va;
                float3 normal = normalize(CrossExplicit(ab, ac));
                float planeW = dot(va, normal);
                float startDot = dot(localStart, normal);
                float endDot = dot(localEnd, normal);
                float startDist = startDot - planeW;
                float endDist = endDot - planeW;

                bool hit = false;
                float4 plane = float4(normal, planeW);

                if (endDist >= 0.0f && endDist < gParams.kCollisionMargin) {
                    float3 closest = ClosestPointOnTriangle(localEnd, va, vb, vc);
                    // The shipped blob computes the separation twice with opposite
                    // operand order: `add -localEnd + closest` for the margin test,
                    // `add localEnd + -closest` for the contact normal.
                    float3 toClosest = closest - localEnd;
                    float distSq = dot(toClosest, toClosest);
                    hit = distSq <= gParams.kCollisionMarginSq;

                    float3 delta = localEnd - closest;
                    float lenSq = dot(delta, delta);
                    float3 contactNormal = lenSq > 0.0f ? delta * rsqrt(lenSq) : normal;
                    plane = hit ? float4(contactNormal, dot(contactNormal, closest)) : plane;
                }

                [branch] if (!hit && startDist > 0.0f && endDist < 0.0f) {
                    hit = SegmentIntersectsTriangle(segmentDelta, localStart, va, vb, vc);
                }

                if (hit) {
                    StoreTriangleContact(
                        particle,
                        count,
                        shape,
                        gParams.kMaxContactsPerParticle,
                        shapeEnd,
                        qEnd,
                        velocity,
                        plane.xyz,
                        plane.w);
                }
            } else {
                int upperBits = int(upperNode.ib & 0x7fffffffu);
                int nextSize;
                if (stackSize >= BVH_STACK_SIZE) {
                    nextSize = stackSize + 1;
                    stackOverflow[localIdx][stackSize - BVH_STACK_SIZE] = int(lowerBits);
                } else {
                    nextSize = stackSize + 1;
                    stack[stackSize] = int(lowerBits);
                }

                if (nextSize >= BVH_STACK_SIZE) {
                    stackSize = nextSize + 1;
                    stackOverflow[localIdx][nextSize - BVH_STACK_SIZE] = upperBits;
                } else {
                    stackSize = nextSize + 1;
                    stack[nextSize] = upperBits;
                }
            }
        }
    }
}


[numthreads(BLOCK_DIM_X, 1, 1)]
void CollideTriangles(int particle : SV_DispatchThreadID, int localIdx : SV_GroupThreadID) {
    if (particle < gParams.kNumParticles) {
        float3 particleStart = sortedPositions[particle].xyz;
        float3 particleEnd = sortedNewPositions[particle].xyz;
        int stack[BVH_STACK_SIZE];
        int numTriangleShapes = 0;

        if (gParams.kNumShapes != 0) {
            TraverseShapeBvh(particle, localIdx, particleStart, particleEnd, stack, numTriangleShapes);
        }

        int count = 0;
        for (int i = 0; i < numTriangleShapes; ++i) {
            int shape = triangleShapeCandidates[localIdx * MAX_TRIANGLE_SHAPES + i];
            float3 shapePosition = shapePositions[shape].xyz;
            float4 shapeRotation = shapeRotations[shape];
            float3 shapePrevPosition = shapePrevPositions[shape].xyz;
            float4 shapePrevRotation = shapePrevRotations[shape];

            float3 shapeDelta = shapePosition - shapePrevPosition;
            float3 shapeStart = shapePrevPosition + shapeDelta * gSubParams.kSubstepStart;
            float4 rotationDelta = shapeRotation - shapePrevRotation;
            float4 qStart = NormalizeQuat(shapePrevRotation + rotationDelta * gSubParams.kSubstepStart);
            float3 shapeEnd = shapePrevPosition + shapeDelta * gSubParams.kSubstepEnd;
            float4 qEnd = NormalizeQuat(shapePrevRotation + rotationDelta * gSubParams.kSubstepEnd);
            float4 geometry = shapeGeometry[shape];
            int meshId = asint(geometry.w) - 1;

            float3 localStart = RotateInv(qStart, particleStart - shapeStart);
            float3 localEnd = RotateInv(qEnd, particleEnd - shapeEnd);
            float3 invScale = 1.0f.xxx / geometry.xyz;
            float3 lower = min(localStart, localEnd);
            float3 upper = max(localStart, localEnd);
            float3 queryLower = lower * invScale - gParams.kCollisionMargin * invScale;
            float3 queryUpper = upper * invScale + gParams.kCollisionMargin * invScale;
            TraverseMeshBvh(particle, localIdx, shape, queryLower, queryUpper, localStart, localEnd,
                            shapeEnd, qEnd, geometry, shapeStart, qStart, meshId, stack, count);
        }

        collisionCounts[particle] = count;
    }
}
