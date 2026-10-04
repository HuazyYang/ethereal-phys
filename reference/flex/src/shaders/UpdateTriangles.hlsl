#include "KernelParams.hlsli"
#include "Utils.hlsli"

StructuredBuffer<float4> sortedNewPositions : register(t0);
StructuredBuffer<float4> sortedVelocities : register(t1);
StructuredBuffer<int> reverseLookup : register(t2);
StructuredBuffer<int> indices : register(t3);
RWByteAddressBuffer accum : register(u0);
RWStructuredBuffer<float3> triNormals : register(u1);

#if USE_NV_SHADER_EXT
#include <nvHLSLExtns.h>
#define InterlockedAddFp32 NvInterlockedAddFp32
#endif

#define BLOCK_DIM_X 256

[numthreads(BLOCK_DIM_X, 1, 1)]
void UpdateTriangles(int globalIdx: SV_DispatchThreadID) {
    if (globalIdx < gParams.kNumTriangles) {
        int idx1 = reverseLookup[indices[globalIdx * 3]];
        int idx2 = reverseLookup[indices[globalIdx * 3 + 1]];
        int idx3 = reverseLookup[indices[globalIdx * 3 + 2]];

        float4 pos1 = sortedNewPositions[idx1];
        float4 pos2 = sortedNewPositions[idx2];
        float4 pos3 = sortedNewPositions[idx3];

        float3 pos21 = pos2.xyz - pos1.xyz;
        float3 pos31 = pos3.xyz - pos1.xyz;
        float4 c0 = pos31.yzzx * pos21.zyxz;
        float2 nxy = c0.xz - c0.yw;
        float nz = pos31.x * pos21.y - pos31.y * pos21.x;
        float3 n = float3(nxy, nz);
        float A = dot(n, n);
        n = A > 0.0 ? n * rsqrt(A) : 0.0.xxx;
        triNormals[globalIdx] = n;

        if (gParams.kDrag != 0 || gParams.kLift != 0) {
            float3 v1 = sortedVelocities[idx1].xyz;
            float3 v2 = sortedVelocities[idx2].xyz;
            float3 v3 = sortedVelocities[idx3].xyz;

            // The shipped blob packs the relative wind into four lanes: a scalar
            // copy of w.z plus the triple (w.z, w.y, w.x). `wz` is the separately
            // spelled scalar copy consumed by windN.x (mad rN.x, .., r5.x, ..);
            // spelling it apart from `wr` is what keeps FXC from folding the lane.
            // The cross with the normal is stated as one four-wide product `t` read
            // back stride-two, matching the shipped `mul r1.yz` / `mad r6.xyzw` pair.
            float3 vs = v1 + v2;
            vs = v3 + vs;
            float wz = gParams.kWind.z - vs.z * 0.3333;
            // `wr` holds the wind reversed (w.z, w.y, w.x), matching the shipped lanes.
            float3 wsum = v1.zyx + v2.zyx;
            wsum = v3.zyx + wsum;
            float3 wr = gParams.kWind.zyx - wsum * 0.3333;
            float windMag = dot(wr, wr);
            float3 windDir = windMag > 0 ? wr.zyx * rsqrt(windMag) : 0.0.xxx;
            float4 n4 = float4(n, dot(windDir, n));
            n4 = n4.w < 0 ? -n4 : n4;
            float2 forceMag = float2(gParams.kDrag, gParams.kLift) * gParams.kDt * n4.w;
            windMag = sqrt(windMag);
            float cosTheta = n4.w;
            float sinTheta = sqrt(max(1.0 - cosTheta * cosTheta, 0.0));
            forceMag.y = forceMag.y * sinTheta;
            forceMag.y = max(min(forceMag.y, 1.0), -1.0);
            float liftMag = windMag * forceMag.y;

            float4 t = wr.xxyz * n4.xyzz;
            float4 wt = t.xzzx - t.wyyw;
            float wtz = wr.z * n4.y - wr.y * n4.x;

            float2 tq = wt.yw * wr.xz;
            float tr = wtz * wr.y;
            float3 windN;
            windN.x = wt.x * wz - tr;
            windN.y = wtz * wr.z - tq.x;
            windN.z = wt.z * wr.y - tq.y;
            float windNMag = dot(windN, windN);
            float3 liftDir = windNMag > 0.0 ? windN * rsqrt(windNMag) : 0.0.xxx;

            float3 windForce = forceMag.x * wr.zyx + liftMag * liftDir;

            float3 windAccel;

            int addr1 = idx1 << 4;
            windAccel = windForce * pos1.w;

            InterlockedAddFp32(accum, addr1, windAccel.x);
            InterlockedAddFp32(accum, addr1 + 4, windAccel.y);
            InterlockedAddFp32(accum, addr1 + 8, windAccel.z);
            InterlockedAddFp32(accum, addr1 + 12, 1.0);

            int addr2 = idx2 << 4;
            windAccel = windForce * pos2.w;
            InterlockedAddFp32(accum, addr2, windAccel.x);
            InterlockedAddFp32(accum, addr2 + 4, windAccel.y);
            InterlockedAddFp32(accum, addr2 + 8, windAccel.z);
            InterlockedAddFp32(accum, addr2 + 12, 1.0);

            int addr3 = idx3 << 4;
            windAccel = windForce * pos3.w;
            InterlockedAddFp32(accum, addr3, windAccel.x);
            InterlockedAddFp32(accum, addr3 + 4, windAccel.y);
            InterlockedAddFp32(accum, addr3 + 8, windAccel.z);
            InterlockedAddFp32(accum, addr3 + 12, 1.0);
        }
    }
}
