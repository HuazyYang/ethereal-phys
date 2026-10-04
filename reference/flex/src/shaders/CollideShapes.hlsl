#include "KernelParams.hlsli"

#define BLOCK_DIM_X 128
#define BVH_STACK_SIZE 64

static const uint eNvFlexShapeFlagTrigger = 0x10;

struct PackedNodeHalf {
    float3 v;
    uint ib;
};

SamplerState sdfSampler : register(s0);

StructuredBuffer<float4> shapeGeometry : register(t0);
StructuredBuffer<float4> shapePositions : register(t1);
StructuredBuffer<float4> shapeRotations : register(t2);
StructuredBuffer<float4> shapePrevPositions : register(t3);
StructuredBuffer<float4> shapePrevRotations : register(t4);
StructuredBuffer<int> shapeFlags : register(t5);
StructuredBuffer<float4> sortedPositions : register(t6);
StructuredBuffer<float4> sortedNewPositions : register(t7);
StructuredBuffer<int> sortedPhases : register(t8);
StructuredBuffer<int> shapeBvhRootNode : register(t9);
StructuredBuffer<PackedNodeHalf> shapeBvhNodeLowers : register(t10);
StructuredBuffer<PackedNodeHalf> shapeBvhNodeUppers : register(t11);
StructuredBuffer<FlexConvexMeshDevice> convexes : register(t12);
StructuredBuffer<float4> convexPlanes : register(t13);
StructuredBuffer<FlexSDFDevice> sdfs : register(t14);

Texture3D<float> sdfTex0 : register(t15);
Texture3D<float> sdfTex1 : register(t16);
Texture3D<float> sdfTex2 : register(t17);
Texture3D<float> sdfTex3 : register(t18);
Texture3D<float> sdfTex4 : register(t19);
Texture3D<float> sdfTex5 : register(t20);
Texture3D<float> sdfTex6 : register(t21);
Texture3D<float> sdfTex7 : register(t22);
Texture3D<float> sdfTex8 : register(t23);
Texture3D<float> sdfTex9 : register(t24);
Texture3D<float> sdfTex10 : register(t25);
Texture3D<float> sdfTex11 : register(t26);
Texture3D<float> sdfTex12 : register(t27);
Texture3D<float> sdfTex13 : register(t28);
Texture3D<float> sdfTex14 : register(t29);
Texture3D<float> sdfTex15 : register(t30);

RWStructuredBuffer<int> collisionCounts : register(u0);
RWStructuredBuffer<float4> collisionPlanes : register(u1);
RWStructuredBuffer<float4> collisionVelocities : register(u2);

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

float3 SafeNormalize(float3 v) {
    float lenSq = dot(v, v);
    return lenSq > 0.0f ? v * rsqrt(lenSq) : float3(0.0f, 0.0f, 0.0f);
}

bool OverlapAabb(float3 lowerA, float3 upperA, float3 lowerB, float3 upperB) {
    bool3 a = upperA < lowerB;
    bool anyA = a.x || a.y || a.z;
    bool3 b = upperB < lowerA;
    return !(anyA || b.x || b.y || b.z);
}

bool IntersectSegmentAabb(float3 invD, float3 a, float3 halfExtents) {
    float3 t1 = (halfExtents - a) * invD;
    float3 t0 = (-halfExtents - a) * invD;
    float3 tMin3 = min(t1, t0);
    float tMin = max(0.0f, tMin3.x);
    float3 tMax3 = max(t1, t0);
    float tMax = min(1.0f, tMax3.x);
    tMin = max(tMin, tMin3.y);
    tMax = min(tMax, tMax3.y);
    tMin = max(tMin, tMin3.z);
    tMax = min(tMax, tMax3.z);
    return tMin < tMax;
}

float SampleSDF(int index, float3 uvw) {
    switch (index) {
    case 0:
        return sdfTex0.SampleLevel(sdfSampler, uvw, 0.0f);
    case 1:
        return sdfTex1.SampleLevel(sdfSampler, uvw, 0.0f);
    case 2:
        return sdfTex2.SampleLevel(sdfSampler, uvw, 0.0f);
    case 3:
        return sdfTex3.SampleLevel(sdfSampler, uvw, 0.0f);
    case 4:
        return sdfTex4.SampleLevel(sdfSampler, uvw, 0.0f);
    case 5:
        return sdfTex5.SampleLevel(sdfSampler, uvw, 0.0f);
    case 6:
        return sdfTex6.SampleLevel(sdfSampler, uvw, 0.0f);
    case 7:
        return sdfTex7.SampleLevel(sdfSampler, uvw, 0.0f);
    case 8:
        return sdfTex8.SampleLevel(sdfSampler, uvw, 0.0f);
    case 9:
        return sdfTex9.SampleLevel(sdfSampler, uvw, 0.0f);
    case 10:
        return sdfTex10.SampleLevel(sdfSampler, uvw, 0.0f);
    case 11:
        return sdfTex11.SampleLevel(sdfSampler, uvw, 0.0f);
    case 12:
        return sdfTex12.SampleLevel(sdfSampler, uvw, 0.0f);
    case 13:
        return sdfTex13.SampleLevel(sdfSampler, uvw, 0.0f);
    case 14:
        return sdfTex14.SampleLevel(sdfSampler, uvw, 0.0f);
    case 15:
        return sdfTex15.SampleLevel(sdfSampler, uvw, 0.0f);
    default:
        return 0.0f;
    }
}

void StoreContact(
    int particle,
    inout int count,
    int shape,
    int flags,
    float4 qStart,
    float4 qEnd,
    float3 xStart,
    float3 xEnd,
    float3 particleEnd,
    float3 localParticleEnd,
    float3 localNormal,
    float localPlaneW) {

    if (count < gParams.kMaxContactsPerParticle) {
        float3 shapeStartPoint = Rotate(qStart, localParticleEnd) + xStart;
        float3 velocity = particleEnd - shapeStartPoint;
        int contactIndex = particle * gParams.kMaxContactsPerParticle + count;

        float3 normal = Rotate(qEnd, localNormal);
        float4 plane = float4(normal, localPlaneW - dot(normal, xEnd));

        if (flags & eNvFlexShapeFlagTrigger)
            plane = 0.0f.xxxx;

        collisionPlanes[contactIndex] = plane;
        collisionVelocities[contactIndex] = float4(velocity, float(shape));
        count++;
    }
}

bool SphereContact(float3 localEnd, float radius, out float3 normal, out float planeW) {
    float lenSq = dot(localEnd, localEnd);
    float len = sqrt(lenSq);
    normal = SafeNormalize(localEnd);
    planeW = -radius;
    return len - radius - gParams.kCollisionDistance < gParams.kCollisionThreshold;
}

bool CapsuleContact(
    float3 localStart,
    float3 localEnd,
    float radius,
    float halfHeight,
    out float3 normal,
    out float planeW) {

    float axisX = (localStart.x > halfHeight)
                ? halfHeight
                : ((localStart.x < -halfHeight) ? -halfHeight : localStart.x);
    float3 axisPoint = float3(axisX, 0.0f, 0.0f);
    float3 delta = localStart - axisPoint;
    float distSq = dot(delta, delta);
    float radialDistance = sqrt(distSq) - radius - gParams.kCollisionDistance;

    // Closest point between the particle sweep (localStart -> localEnd) and the
    // capsule axis segment, expanded by the radius along x.
    float axisLower = -halfHeight - radius;
    float axisUpper = halfHeight + radius;
    float3 seg = localEnd - localStart;
    float extent = axisUpper - axisLower;
    float3 offset = localStart - float3(axisLower, 0.0f, 0.0f);

    float segLengthSq = dot(seg, seg);
    float segDotOffset = dot(seg, offset);
    float axisLengthSq = extent * extent;
    float axisDotOffset = extent * offset.x;
    float axisDotSeg = seg.x * extent;

    float denominator = segLengthSq * axisLengthSq - axisDotSeg * axisDotSeg;
    float segParam = (denominator != 0.0f)
                   ? saturate((axisDotSeg * axisDotOffset - segDotOffset * axisLengthSq) / denominator)
                   : 0.0f;
    float axisParam = (axisDotSeg * segParam + axisDotOffset) / axisLengthSq;

    bool axisBelow = axisParam < 0.0f;
    float segParamBelow = saturate(-segDotOffset / segLengthSq);
    if (axisParam > 1.0f) {
        segParam = saturate((axisDotSeg - segDotOffset) / segLengthSq);
        axisParam = 1.0f;
    }
    if (axisBelow) {
        segParam = segParamBelow;
        axisParam = 0.0f;
    }

    float3 closestOnSeg = seg * segParam + localStart;
    float3 closestOnAxis = float3(extent * axisParam + axisLower, 0.0f, 0.0f);
    float3 separation = closestOnSeg - closestOnAxis;

    float separationSq = dot(separation, separation);
    float reach = gParams.kCollisionDistance + gParams.kCollisionThreshold + radius;
    bool axisContact = reach * reach >= separationSq;

    normal = SafeNormalize(delta);
    float3 surfacePoint = normal * radius + axisPoint;
    planeW = -dot(normal, surfacePoint);
    return axisContact || radialDistance < gParams.kCollisionThreshold;
}

bool BoxContact(
    float3 localStart,
    float3 localEnd,
    float3 halfExtents,
    out float3 normal,
    out float planeW) {

    // `normal` and `planeW` are deliberately left untouched on the miss path: the DXBC
    // writes them only once a contact is found, and the caller keeps its own defaults.
    float3 invD = 1.0f / (localEnd - localStart);
    float3 expanded = halfExtents + gParams.kCollisionDistance;
    expanded = expanded + gParams.kCollisionThreshold;

    // The DXBC bases the sweep at localStart and marches toward localEnd; the
    // reversed parametrisation is the same segment but mirrors t.
    if (!IntersectSegmentAabb(invD, localStart, expanded))
        return false;

    float3 toMax = localStart - expanded;

    float best = 3.402823466e+38f;
    float4 plane = float4(1.0f, 0.0f, 0.0f, -halfExtents.x);

    best = min(abs(toMax.x), best);
    float3 toMin = localStart + expanded;
    if (abs(toMin.x) < best) { plane = float4(-1.0f,  0.0f,  0.0f, -halfExtents.x); }
    best = min(abs(toMin.x), best);
    if (abs(toMax.y) < best) { plane = float4( 0.0f,  1.0f,  0.0f, -halfExtents.y); }
    best = min(best, abs(toMax.y));
    if (abs(toMin.y) < best) { plane = float4( 0.0f, -1.0f,  0.0f, -halfExtents.y); }
    best = min(best, abs(toMin.y));
    if (abs(toMax.z) < best) { plane = float4( 0.0f,  0.0f,  1.0f, -halfExtents.z); }
    best = min(best, abs(toMax.z));
    if (abs(toMin.z) < best) { plane = float4( 0.0f,  0.0f, -1.0f, -halfExtents.z); }

    normal = plane.xyz;
    planeW = plane.w;
    return true;
}

bool ConvexContact(
    float3 localStart,
    float3 localEnd,
    float3 scale,
    int meshId,
    out float3 normal,
    out float planeW) {

    FlexConvexMeshDevice convex = convexes[meshId - 1];
    int planeBegin = convex.mPlaneOffset;
    float3 segment = localEnd - localStart;
    int planeEnd = convex.mPlaneCount + planeBegin;
    float3 invScale = 1.0f / scale;

    normal = float3(0.0f, 0.0f, 0.0f);
    float enter = 0.0f;
    float exit = 1.0f;
    float2 closestPair = float2(3.402823466e+38f, 0.0f);

    for (int i = planeBegin; i < planeEnd; ++i) {
        float4 p = convexPlanes[i];
        float3 n = invScale * p.xyz;
        float invLength = 1.0f / sqrt(dot(n, n));
        // Written as four scalars in this order so the scaled plane offset lands
        // in the same lane pair as `closest` below; the shipped blob fuses them
        // into one `mul r28.xyzw, r27.xwyz, r6.wwww`.
        float nx = invLength * n.x;
        float wScaled = invLength * p.w;
        float ny = invLength * n.y;
        float nz = invLength * n.z;
        n = float3(nx, ny, nz);
        // The DXBC keeps the unadjusted offset for the stored plane and uses the
        // margin-adjusted one only for the ray clip, so both are needed.
        float wRaw = wScaled;
        float w = invLength * p.w - gParams.kCollisionDistance - gParams.kCollisionThreshold;

        // Measured at localStart, the ray origin: this is what picks the
        // "closest" plane, so basing it at the other endpoint selects a
        // different plane whenever the particle has moved.
        float distance = dot(float4(n, w), float4(localStart, 1.0f));
        float denom = dot(n, segment);

        if (denom != 0.0f) {
            float t = -distance / denom;
            if (denom < 0.0f) {
                if (enter < t)
                    enter = t;
            } else {
                if (t < exit)
                    exit = t;
            }
            if (enter >= exit)
                break;
        } else if (distance > 0.0f) {
            enter = exit + 1.0f;
            break;
        }

        float2 candidate = float2(abs(distance), wRaw);
        if (candidate.x < closestPair.x) {
            normal = n;
            closestPair = candidate;
        }
    }

    planeW = closestPair.y;
    return enter < exit;
}

// The DXBC nests the contact store inside the gradient gate rather than returning
// a hit flag, so the store is performed here instead of at the call site.
void SdfContact(
    float3 localEnd,
    float scale,
    int fieldId,
    int particle,
    inout int count,
    int shape,
    int flags,
    float4 qStart,
    float4 qEnd,
    float3 xStart,
    float3 xEnd,
    float3 particleEnd) {

    int sdfIndex = fieldId - 1;
    float invScale = 1.0f / scale;
    float3 uvw = localEnd * invScale;

    if (uvw.x > 0.0f && uvw.x < 1.0f && uvw.y > 0.0f && uvw.y < 1.0f && uvw.z > 0.0f && uvw.z < 1.0f) {

        float dist = SampleSDF(sdfIndex, uvw);
        if (dist < (gParams.kCollisionDistance + gParams.kCollisionThreshold) * invScale) {
            float3 h = 1.0f / float3(sdfs[sdfIndex].mDim.xyz);
            // The shipped blob offsets all three components at once
            // (`mad r10.xyz` / `mad r18.yzw`) and then rebuilds each sample
            // coordinate from one perturbed component plus two unperturbed ones,
            // which is where its per-case `mov r16.x, r10.x` / `mov r16.yz`
            // pairs come from -- 155 instructions that `uvw + float3(h.x,0,0)`
            // folds away.
            float3 uvwLo = uvw - h;
            float3 uvwHi = uvw + h;
            float dx = SampleSDF(sdfIndex, float3(uvwHi.x, uvw.y, uvw.z)) -
                SampleSDF(sdfIndex, float3(uvwLo.x, uvw.y, uvw.z));
            float dy = SampleSDF(sdfIndex, float3(uvw.x, uvwHi.y, uvw.z)) -
                SampleSDF(sdfIndex, float3(uvw.x, uvwLo.y, uvw.z));
            float dz = SampleSDF(sdfIndex, float3(uvw.x, uvw.y, uvwHi.z)) -
                SampleSDF(sdfIndex, float3(uvw.x, uvw.y, uvwLo.z));

            float3 gradient = float3(dx, dy, dz);
            float gradientSq = dot(gradient, gradient);

            if (gradientSq > 0.0f) {
                float3 scaledUvw = scale * uvw;
                float3 normal = gradient * rsqrt(gradientSq);
                float planeW = scale * dist - dot(normal, scaledUvw);
                StoreContact(
                    particle,
                    count,
                    shape,
                    flags,
                    qStart,
                    qEnd,
                    xStart,
                    xEnd,
                    particleEnd,
                    localEnd,
                    normal,
                    planeW);
            }
        }
    }
}

[numthreads(BLOCK_DIM_X, 1, 1)]
void CollideShapes(int particle : SV_DispatchThreadID) {
    if (particle < gParams.kNumParticles) {
    float3 particleEnd = sortedNewPositions[particle].xyz;
    int count = collisionCounts[particle];

    for (int i = 0; i < gParams.kNumPlanes; ++i) {
        float4 plane = gParams.kPlanes[i];
        float d = dot(particleEnd, plane.xyz) + plane.w - gParams.kCollisionDistance;

        if (d <= gParams.kCollisionThreshold && count < gParams.kMaxContactsPerParticle) {
            int contactIndex = particle * gParams.kMaxContactsPerParticle + count;
            collisionPlanes[contactIndex] = plane;
            collisionVelocities[contactIndex] = float4(0.0f, 0.0f, 0.0f, -1.0f);
            count++;
        }
    }

    if (gParams.kNumShapes != 0) {
        float3 particleStart = sortedPositions[particle].xyz;
        uint particleChannels = uint(sortedPhases[particle]) & 0xff000000u;
        float3 lower = min(particleStart, particleEnd);
        float3 upper = max(particleStart, particleEnd);
        float3 queryLower = lower - gParams.kCollisionMargin.xxx;
        float3 queryUpper = upper + gParams.kCollisionMargin.xxx;
        uint stack[BVH_STACK_SIZE];
        int stackSize = 1;
        stack[0] = uint(shapeBvhRootNode[0]);

        while (stackSize != 0) {
            uint node = stack[stackSize - 1];
            PackedNodeHalf lowerNode = shapeBvhNodeLowers[node];
            PackedNodeHalf upperNode = shapeBvhNodeUppers[node];

            if (OverlapAabb(queryLower, queryUpper, lowerNode.v, upperNode.v)) {
                uint lowerBits = lowerNode.ib & 0x7fffffffu;
                uint leafBit = lowerNode.ib & 0x80000000u;
                if (leafBit) {
                    stackSize = stackSize - 1;

                    int shape = int(lowerBits);
                    int flags = shapeFlags[shape];
                    if (particleChannels & uint(flags)) {

                    uint type = uint(flags) & 7u;
                    float3 shapePos = shapePositions[shape].xyz;
                    float4 shapeRot = shapeRotations[shape];
                    float3 shapePrevPos = shapePrevPositions[shape].xyz;
                    float4 shapePrevRot = shapePrevRotations[shape];
                    float3 positionDelta = shapePos - shapePrevPos;
                    float3 xStart = positionDelta * gSubParams.kSubstepStart + shapePrevPos;
                    float4 rotationDelta = shapeRot - shapePrevRot;
                    float4 qStart = NormalizeQuat(rotationDelta * gSubParams.kSubstepStart + shapePrevRot);
                    float3 xEnd = positionDelta * gSubParams.kSubstepEnd + shapePrevPos;
                    float4 qEnd = NormalizeQuat(rotationDelta * gSubParams.kSubstepEnd + shapePrevRot);

                    float3 localEnd = RotateInv(qEnd, particleEnd - xEnd);
                    float3 localStart = RotateInv(qStart, particleStart - xStart);
                    float4 geometry = shapeGeometry[shape];
                    float3 localNormal = float3(0.0f, 1.0f, 0.0f);
                    float localPlaneW = 0.0f;

                    if (type == eNvFlexShapeBox) {
                        if (BoxContact(localStart, localEnd, geometry.xyz, localNormal, localPlaneW))
                            StoreContact(
                            particle,
                            count,
                            shape,
                            flags,
                            qStart,
                            qEnd,
                            xStart,
                            xEnd,
                            particleEnd,
                            localEnd,
                            localNormal,
                            localPlaneW);
                    } else if (type == eNvFlexShapeConvexMesh) {
                        if (ConvexContact(localStart, localEnd, geometry.xyz, asint(geometry.w), localNormal, localPlaneW))
                            StoreContact(
                            particle,
                            count,
                            shape,
                            flags,
                            qStart,
                            qEnd,
                            xStart,
                            xEnd,
                            particleEnd,
                            localEnd,
                            localNormal,
                            localPlaneW);
                    } else if (type == eNvFlexShapeSphere) {
                        if (SphereContact(localEnd, geometry.x, localNormal, localPlaneW))
                            StoreContact(
                            particle,
                            count,
                            shape,
                            flags,
                            qStart,
                            qEnd,
                            xStart,
                            xEnd,
                            particleEnd,
                            localEnd,
                            localNormal,
                            localPlaneW);
                    } else if (type == eNvFlexShapeCapsule) {
                        if (CapsuleContact(localStart, localEnd, geometry.x, geometry.y, localNormal, localPlaneW))
                            StoreContact(
                            particle,
                            count,
                            shape,
                            flags,
                            qStart,
                            qEnd,
                            xStart,
                            xEnd,
                            particleEnd,
                            localEnd,
                            localNormal,
                            localPlaneW);
                    } else if (type == eNvFlexShapeSDF) {
                        SdfContact(
                            localEnd,
                            geometry.x,
                            asint(geometry.y),
                            particle,
                            count,
                            shape,
                            flags,
                            qStart,
                            qEnd,
                            xStart,
                            xEnd,
                            particleEnd);
                    }
                    }
                } else {
                    uint upperBits = upperNode.ib & 0x7fffffffu;
                    stack[stackSize - 1] = lowerBits;
                    int nextSize = stackSize + 1;
                    stack[stackSize] = upperBits;
                    stackSize = nextSize;
                }
            } else {
                stackSize = stackSize - 1;
            }
        }
    }

    collisionCounts[particle] = count;
    }
}
