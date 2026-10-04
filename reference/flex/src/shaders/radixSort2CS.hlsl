#define BLOCK_DIM 256

cbuffer params : register(b0)
{
	uint3 gridDim;
	uint passStart;
    uint numCounters;
};

Buffer<uint4> countersSrc : register(t0);

RWBuffer<uint4> countersDst : register(u0);

groupshared uint sdata0[BLOCK_DIM];
groupshared uint sdata1[BLOCK_DIM];

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

[numthreads(BLOCK_DIM, 1, 1)]
void radixSort2CS(uint3 blockIdx : SV_GroupID, uint3 threadIdx : SV_GroupThreadID)
{
	if (blockIdx.x == 0)
	{
		uint globalOffset = 0u;

		uint numPasses = (numCounters / 4 + BLOCK_DIM - 1) / (BLOCK_DIM);
		uint idx = threadIdx.x;
		for (uint passID = 0; passID < numPasses; passID++)
		{
			uint blockOffset = 0u;

			uint4 countLocal = (idx < numCounters / 4) ? countersSrc[idx] : uint4(0, 0, 0, 0);

			uint4 countGlobal = blockScan(threadIdx, countLocal, blockOffset);

			// make inclusive
			countGlobal.x -= countLocal.x;
			countGlobal.y -= countLocal.y;
			countGlobal.z -= countLocal.z;
			countGlobal.w -= countLocal.w;

			countGlobal.x += globalOffset;
			countGlobal.y += globalOffset;
			countGlobal.z += globalOffset;
			countGlobal.w += globalOffset;

			if (idx < numCounters / 4)
			{
				countersDst[idx] = countGlobal;
			}

			globalOffset += blockOffset;

			idx += BLOCK_DIM;
		}
	}
	else
	{
		uint numPasses = (numCounters / 4 + BLOCK_DIM - 1) / (BLOCK_DIM);
		uint idx = threadIdx.x;
		for (uint passID = 0; passID < numPasses; passID++)
		{
			uint4 countLocal = (idx < numCounters / 4) ? countersSrc[idx + numCounters / 4] : uint4(0, 0, 0, 0);

			sdata0[threadIdx.x] = countLocal.x + countLocal.y + countLocal.z + countLocal.w;

			GroupMemoryBarrierWithGroupSync();

			uint scanTotal = 0;
			if ((threadIdx.x & 3) >= 1) scanTotal += sdata0[4 * (threadIdx.x / 4) + 0];
			if ((threadIdx.x & 3) >= 2) scanTotal += sdata0[4 * (threadIdx.x / 4) + 1];
			if ((threadIdx.x & 3) >= 3) scanTotal += sdata0[4 * (threadIdx.x / 4) + 2];

			// make final scan exclusive
			countLocal.w = countLocal.z + countLocal.y + countLocal.x + scanTotal;
			countLocal.z = countLocal.y + countLocal.x + scanTotal;
			countLocal.y = countLocal.x + scanTotal;
			countLocal.x = scanTotal;

			if (idx < numCounters / 4)
			{
				countersDst[idx + numCounters / 4] = countLocal;
			}

			idx += BLOCK_DIM;
		}
	}
}
