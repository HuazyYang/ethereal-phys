#include "KernelParams.hlsli"

StructuredBuffer<int> numDiffuseParticles : register(t0);
StructuredBuffer<int> cellBucketStarts : register(t1);
StructuredBuffer<int> cellBucketEnds : register(t2);
StructuredBuffer<int> contactCounts : register(t3);
StructuredBuffer<float4> sortedNewPositionsTex : register(t4);
StructuredBuffer<float4> sortedNewVelocitiesTex : register(t5);
StructuredBuffer<float3> gBounds : register(t7);
RWStructuredBuffer<float4> diffusePositions : register(u0);
RWStructuredBuffer<float4> diffuseVelocities : register(u1);

uint DiffuseGridIndex(float3 position) {
    int3 cell = int3((position - gBounds[0]) * gParams.kInvCellEdge);
    return uint((cell.x & 0x7f) | (((cell.z & 0x7f) << 14) | ((cell.y & 0x7f) << 7)));
}

float3 LimitDiffuseVelocity(float3 velocity) {
    float speedSq = dot(velocity, velocity);
    float invSpeed = rsqrt(speedSq);
    float maxSpeedSq = gParams.kDiffuseMaxVelocity * gParams.kDiffuseMaxVelocity;
    return (speedSq > maxSpeedSq) ? velocity * (invSpeed * gParams.kDiffuseMaxVelocity) : velocity;
}

[numthreads(256, 1, 1)]
void UpdateDiffuseParticles(uint idx : SV_DispatchThreadID) {
    if (int(idx) < numDiffuseParticles[0]) {
        float4 diffusePosition = diffusePositions[idx];
        float3 diffuseVelocity = diffuseVelocities[idx].xyz;

        uint cell = DiffuseGridIndex(diffusePosition.xyz);
        int start = cellBucketStarts[cell];
        int end = cellBucketEnds[cell];
        float3 weightedVelocity = 0.0;
        int firstContactCount = 0;
        float weightSum = 0.0;

        [loop]
        for (int i = start; i < end; ++i) {
            float3 delta = diffusePosition.xyz - sortedNewPositionsTex[i].xyz;
            float distSq = dot(delta, delta);
            if (distSq < gParams.kRadiusSq) {
                float3 neighborVelocity = sortedNewVelocitiesTex[i].xyz;
                float weight = 1.0 - distSq * gParams.kInvRadius;
                weightSum += weight;
                weightedVelocity += neighborVelocity * weight;
                if (firstContactCount == 0)
                    firstContactCount = contactCounts[i];
            }
        }

        float3 fluidVelocity = weightedVelocity / weightSum;
        bool coupledToFluid = firstContactCount >= gParams.kDiffuseBallistic;

        float3 draggedVelocity = diffuseVelocity -
                                 gParams.kDiffuseBuoyancy * gParams.kGravity * gParams.kDiffuseDt +
                                 gParams.kDiffuseDrag * (fluidVelocity - diffuseVelocity);
        draggedVelocity = LimitDiffuseVelocity(draggedVelocity);
        float3 draggedPosition = diffusePosition.xyz + draggedVelocity * gParams.kDiffuseDt;

        float3 ballisticVelocity = diffuseVelocity * (1.0 - gParams.kDiffuseDt) +
                                   gParams.kGravity * gParams.kDiffuseDt;
        ballisticVelocity = LimitDiffuseVelocity(ballisticVelocity);
        float3 ballisticPosition = diffusePosition.xyz + ballisticVelocity * gParams.kDiffuseDt;

        // The DXBC selects velocity.xyz together with position.x in one movc and
        // position.yz in a second; keep the pairing so the movc widths match.
        float4 velocityAndPositionX = coupledToFluid ? float4(draggedVelocity, draggedPosition.x)
                                                     : float4(ballisticVelocity, ballisticPosition.x);
        float2 positionYZ = coupledToFluid ? draggedPosition.yz : ballisticPosition.yz;
        float3 velocity = velocityAndPositionX.xyz;
        float3 position = float3(velocityAndPositionX.w, positionYZ);

        [loop]
        for (int planeIndex = 0; planeIndex < gParams.kNumPlanes; ++planeIndex) {
            float4 plane = gParams.kPlanes[planeIndex];
            float distance = dot(position, plane.xyz) + plane.w - gParams.kCollisionDistance;
            if (distance <= 0.0) {
                position -= distance * plane.xyz;
                // The DXBC takes this dot against the velocity loaded at entry, not
                // the running `velocity` the loop updates: `dp3 r1.w, r1.xyzx, ...`
                // where r1.xyz is diffuseVelocities[idx].xyz and is never rewritten.
                float normalVelocity = dot(diffuseVelocity, plane.xyz);
                if (normalVelocity < 0.0)
                    velocity -= 1.5 * normalVelocity * plane.xyz;
            }
        }

        diffuseVelocities[idx] = float4(velocity, 0.0);
        diffusePositions[idx] = float4(position, diffusePosition.w - gParams.kDiffuseDt);
    }
}
