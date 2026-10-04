#include "KernelParams.hlsli"

StructuredBuffer<uint> contacts : register(t0);
StructuredBuffer<int> contactCounts : register(t1);
Buffer<uint> indices : register(t2);
StructuredBuffer<uint> phases : register(t3);
StructuredBuffer<float4> sortedPositionsTex : register(t4);
RWStructuredBuffer<float4> q1 : register(u0);
RWStructuredBuffer<float4> q2 : register(u1);
RWStructuredBuffer<float4> q3 : register(u2);

void JacobiRotate(inout float3x3 a, inout float3x3 v, int p, int q) {
    float apq = a[p][q];
    if (apq != 0.0) {
        float diagonalDelta = a[p][p] - a[q][q];
        float theta = diagonalDelta * rcp(apq + apq);
        float t = rcp(abs(theta) + sqrt(theta * theta + 1.0)) * sign(theta);
        float c = rsqrt(t * t + 1.0);
        float s = t * c;

        float app = a[p][p];
        float aqq = a[q][q];
        a[p][p] = app + t * apq;
        a[q][q] = -t * apq + aqq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;

        [unroll]
        for (int k = 0; k < 3; ++k) {
            if (k != p && k != q) {
                float akp = a[k][p];
                float akq = a[k][q];
                a[k][p] = c * akp + s * akq;
                a[p][k] = a[k][p];
                a[k][q] = -s * akp + c * akq;
                a[q][k] = a[k][q];
            }
        }

        float3 vp = v[p];
        float3 vq = v[q];
        float3 sq = s * vq;
        float3 cq = c * vq;
        v[p] = c * vp + sq;
        v[q] = -s * vp + cq;
    }
}

void EigenSymmetric3x3(inout float3x3 a, out float3x3 v) {
    v = float3x3(1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0);

    [loop]
    for (int sweep = 0; sweep < 4; ++sweep) {
        int axis = 0;
        float largest = abs(a[0][1]);
        if (largest < abs(a[0][2])) {
            axis = 1;
            largest = abs(a[0][2]);
        }
        if (largest < abs(a[1][2])) {
            axis = 2;
            largest = abs(a[1][2]);
        }
        if (largest < 1.0e-15)
            break;

        if (axis == 0)
            JacobiRotate(a, v, 0, 1);
        else if (axis == 1)
            JacobiRotate(a, v, 0, 2);
        else
            JacobiRotate(a, v, 1, 2);
    }
}

float3 AnisotropyScale(float3 eigenValue) {
    return clamp(sqrt(eigenValue) * gParams.kAnisotropy,
                 gParams.kAnisotropyMin,
                 gParams.kAnisotropyMax);
}

[numthreads(256, 1, 1)]
void CalculateAnisotropy(uint idx : SV_DispatchThreadID) {
    if (int(idx) < gParams.kNumParticles) {
        float3 position = sortedPositionsTex[idx].xyz;
        int contactCount = contactCounts[idx];
        uint contactIndex = idx;
        float4 weighted = 0.0;

        [loop]
        for (int i = 0; i < contactCount; ++i) {
            uint neighbor = contacts[contactIndex];
            contactIndex += uint(gParams.kNumParticlesAligned);

            if (phases[neighbor] & eNvFlexPhaseFluid) {
                float3 neighborPos = sortedPositionsTex[neighbor].xyz;
                float3 delta = position - neighborPos;
                float distSq = dot(delta, delta);
                if (distSq < gParams.kRadiusSq && distSq > 0.0) {
                    // The DXBC spells sqrt(distSq) as distSq * rsqrt(distSq).
                    float q = (distSq * rsqrt(distSq)) * gParams.kInvRadius;
                    float weight = 1.0 - q * q * q;
                    weighted.xyz = neighborPos * weight + weighted.xyz;
                    weighted.w = weighted.w + weight;
                }
            }
        }

        uint originalIndex = indices[idx];
        if (weighted.w == 0.0 || contactCount < 0) {
            q1[originalIndex] = float4(1.0, 0.0, 0.0, gParams.kAnisotropyMin);
            q2[originalIndex] = float4(0.0, 1.0, 0.0, gParams.kAnisotropyMin);
            q3[originalIndex] = float4(0.0, 0.0, 1.0, gParams.kAnisotropyMin);
            return;
        }

        float invWeightSum = 1.0 / weighted.w;

        // The DXBC accumulates the nine covariance entries as 4 + 4 + 1 lanes,
        // not as three rows: acc0 = (xx, xy, xz, yx), acc1 = (yy, yz, zx, zy),
        // acc2 = zz. Matching that packing matters -- with refactoringAllowed the
        // driver re-rounds differently per packing, and the Jacobi sweep below
        // turns a one-ulp difference into a different eigenvector basis.
        float4 acc0 = 0.0;
        float4 acc1 = 0.0;
        float acc2 = 0.0;

        contactIndex = idx;
        [loop]
        for (int j = 0; j < contactCount; ++j) {
            uint neighbor = contacts[contactIndex];
            contactIndex += uint(gParams.kNumParticlesAligned);

            [branch] if (phases[neighbor] & eNvFlexPhaseFluid) {
                float3 neighborPos = sortedPositionsTex[neighbor].xyz;
                float3 radial = position - neighborPos;
                float distSq = dot(radial, radial);
                [branch] if (distSq < gParams.kRadiusSq && distSq > 0.0) {
                    float q = sqrt(distSq) * gParams.kInvRadius;
                    float weight = 1.0 - q * q * q;
                    float3 delta = neighborPos - weighted.xyz * invWeightSum;
                    float3 dx = delta * delta.x;        // xx xy xz
                    float3 dy = delta.yzx * delta.y;    // yy yz yx
                    float3 dz = delta * delta.z;        // zx zy zz
                    // Binding both weighted products before accumulating them makes
                    // FXC materialise the two operand packs back to back; written
                    // inline it interleaves the second pack with the first mad.
                    float4 w0 = weight * float4(dx, dy.z);
                    float4 w1 = weight * float4(dy.xy, dz.xy);
                    acc0 += w0;
                    acc1 += w1;
                    acc2 += weight * dz.z;
                }
            }
        }

        float4 c0 = invWeightSum * acc0;
        float4 c1 = invWeightSum * acc1;
        float c2 = invWeightSum * acc2;
        float3x3 covariance = float3x3(c0.x, c0.y, c0.z,
                                       c0.w, c1.x, c1.y,
                                       c1.z, c1.w, c2);

        float3x3 eigenVectors;
        EigenSymmetric3x3(covariance, eigenVectors);

        // The DXBC clamps all three eigenvalues with one vector sqrt/mul/max/min group.
        float3 scales = AnisotropyScale(float3(covariance[0][0], covariance[1][1], covariance[2][2]));
        q1[originalIndex] = float4(eigenVectors[0], scales.x);
        q2[originalIndex] = float4(eigenVectors[1], scales.y);
        q3[originalIndex] = float4(eigenVectors[2], scales.z);
    }
}
