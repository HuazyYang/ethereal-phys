#define BLOCK_DIM 256

cbuffer params : register(b0)
{
	uint3 gridDim;
	uint passStart;
	uint numCounters;
	uint numKeys;
};

Buffer<uint4> keySrc4 : register(t0);
Buffer<uint4> valSrc4 : register(t1);
Buffer<uint> counters : register(t2);

RWBuffer<uint> keyDst : register(u0);
RWBuffer<uint> valDst : register(u1);

groupshared uint skey[4 * BLOCK_DIM];
groupshared uint sval[4 * BLOCK_DIM];
groupshared uint sdata0[BLOCK_DIM];
groupshared uint sdata1[BLOCK_DIM];
groupshared uint globalCounters[16];
groupshared uint localCounters[16];

uint4 blockScan(uint3 threadIdx, uint4 val, out uint totalCount)
{
	uint localVal = val.x + val.y + val.z + val.w;
	sdata0[threadIdx.x] = localVal;

	GroupMemoryBarrierWithGroupSync();

	if (threadIdx.x >= 1) localVal += sdata0[threadIdx.x - 1];
	if (threadIdx.x >= 2) localVal += sdata0[threadIdx.x - 2];
	if (threadIdx.x >= 3) localVal += sdata0[threadIdx.x - 3];
	sdata1[threadIdx.x] = localVal;

	GroupMemoryBarrierWithGroupSync();

	if (threadIdx.x >= 4) localVal += sdata1[threadIdx.x - 4];
	if (threadIdx.x >= 8) localVal += sdata1[threadIdx.x - 8];
	if (threadIdx.x >= 12) localVal += sdata1[threadIdx.x - 12];
	sdata0[threadIdx.x] = localVal;

	GroupMemoryBarrierWithGroupSync();

	if (threadIdx.x >= 16) localVal += sdata0[threadIdx.x - 16];
	if (threadIdx.x >= 32) localVal += sdata0[threadIdx.x - 32];
	if (threadIdx.x >= 48) localVal += sdata0[threadIdx.x - 48];
	sdata1[threadIdx.x] = localVal;

	GroupMemoryBarrierWithGroupSync();

	if (threadIdx.x >= 64) localVal += sdata1[threadIdx.x - 64];
	if (threadIdx.x >= 128) localVal += sdata1[threadIdx.x - 128];
	if (threadIdx.x >= 192) localVal += sdata1[threadIdx.x - 192];

	uint4 retVal;
	retVal.w = localVal;
	retVal.z = retVal.w - val.w;
	retVal.y = retVal.z - val.z;
	retVal.x = retVal.y - val.y;

	// compute totalCount
	totalCount = sdata1[63] + sdata1[127] + sdata1[191] + sdata1[255];

	return retVal;

	return val;
}

// where pred==1 indicates a zero allocation, pred==0 indicates a one allocation
uint4 split4(uint3 threadIdx, uint4 pred)
{
	uint totalCount;
	uint4 scanVal = blockScan(threadIdx, pred, totalCount);

	uint4 rank;
	rank.x = pred.x ? scanVal.x - 1 : 4 * threadIdx.x + 0 - scanVal.x + totalCount;
	rank.y = pred.y ? scanVal.y - 1 : 4 * threadIdx.x + 1 - scanVal.y + totalCount;
	rank.z = pred.z ? scanVal.z - 1 : 4 * threadIdx.x + 2 - scanVal.z + totalCount;
	rank.w = pred.w ? scanVal.w - 1 : 4 * threadIdx.x + 3 - scanVal.w + totalCount;

	return rank;
}

uint4 loadKey4(uint idx) {
    uint4 keyLocal = keySrc4[idx];
    uint4 linearIdx = idx * 4u + uint4(0, 1, 2, 3);
    keyLocal.x = linearIdx.x < numKeys ? keyLocal.x : ~0u;
    keyLocal.y = linearIdx.y < numKeys ? keyLocal.y : ~0u;
    keyLocal.z = linearIdx.z < numKeys ? keyLocal.z : ~0u;
    keyLocal.w = linearIdx.w < numKeys ? keyLocal.w : ~0u;
    return keyLocal;
}

[numthreads(BLOCK_DIM, 1, 1)]
void radixSort3CS(uint3 tidx : SV_DispatchThreadID, uint3 blockIdx : SV_GroupID, uint3 threadIdx : SV_GroupThreadID)
{
	// preload counter values
	if (threadIdx.x < 16)
	{
		globalCounters[threadIdx.x] = counters[threadIdx.x * gridDim.x + blockIdx.x];
		localCounters[threadIdx.x] = counters[threadIdx.x + 16 * (gridDim.x + blockIdx.x)];
	}

    // uint4 keyLocal = keySrc4[tidx.x];
    uint4 keyLocal = loadKey4(tidx.x);
	uint4 valLocal = valSrc4[tidx.x];

	for (uint passID = passStart; passID < passStart + 4; passID++)
	{
		uint4 allocVal;
		allocVal.x = ((keyLocal.x >> passID) & 1) ^ 1u;
		allocVal.y = ((keyLocal.y >> passID) & 1) ^ 1u;
		allocVal.z = ((keyLocal.z >> passID) & 1) ^ 1u;
		allocVal.w = ((keyLocal.w >> passID) & 1) ^ 1u;

		uint4 allocIdx = split4(threadIdx, allocVal);

		skey[allocIdx.x] = keyLocal.x;
		skey[allocIdx.y] = keyLocal.y;
		skey[allocIdx.z] = keyLocal.z;
		skey[allocIdx.w] = keyLocal.w;
		sval[allocIdx.x] = valLocal.x;
		sval[allocIdx.y] = valLocal.y;
		sval[allocIdx.z] = valLocal.z;
		sval[allocIdx.w] = valLocal.w;

		GroupMemoryBarrierWithGroupSync();

		keyLocal.x = skey[4 * threadIdx.x + 0];
		keyLocal.y = skey[4 * threadIdx.x + 1];
		keyLocal.z = skey[4 * threadIdx.x + 2];
		keyLocal.w = skey[4 * threadIdx.x + 3];
		valLocal.x = sval[4 * threadIdx.x + 0];
		valLocal.y = sval[4 * threadIdx.x + 1];
		valLocal.z = sval[4 * threadIdx.x + 2];
		valLocal.w = sval[4 * threadIdx.x + 3];
	}

	GroupMemoryBarrierWithGroupSync();

	for (uint idx = threadIdx.x; idx < 4 * BLOCK_DIM; idx += BLOCK_DIM)
	{
		uint bucketIdx = (skey[idx] >> passStart) & 0xF;
		uint dstIdx = idx - localCounters[bucketIdx] + globalCounters[bucketIdx];
		keyDst[dstIdx] = skey[idx];
		valDst[dstIdx] = sval[idx];
	}
}
