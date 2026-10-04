#define BLOCK_DIM 256

cbuffer params : register(b0)
{
	uint3 gridDim;
	uint passStart;
	uint numCounters;
	uint numKeys;
};

Buffer<uint4> srcKey : register(t0);

RWBuffer<uint> counters : register(u0);

groupshared uint4 scount0[BLOCK_DIM];
groupshared uint4 scount1[BLOCK_DIM / 4];

void count(inout uint4 counter, uint bucket)
{
	if (bucket == 0) counter.x += (1 << 0);
	if (bucket == 1) counter.x += (1 << 16);
	if (bucket == 2) counter.y += (1 << 0);
	if (bucket == 3) counter.y += (1 << 16);
	if (bucket == 4) counter.z += (1 << 0);
	if (bucket == 5) counter.z += (1 << 16);
	if (bucket == 6) counter.w += (1 << 0);
	if (bucket == 7) counter.w += (1 << 16);
	if (bucket == 8) counter.x += (1 << 8);
	if (bucket == 9) counter.x += (1 << 24);
	if (bucket == 10) counter.y += (1 << 8);
	if (bucket == 11) counter.y += (1 << 24);
	if (bucket == 12) counter.z += (1 << 8);
	if (bucket == 13) counter.z += (1 << 24);
	if (bucket == 14) counter.w += (1 << 8);
	if (bucket == 15) counter.w += (1 << 24);
}

uint4 expand8to16L(uint4 counter)
{
	uint4 counterL;
	counterL.x = counter.x & 0x00FF00FF;
	counterL.y = counter.y & 0x00FF00FF;
	counterL.z = counter.z & 0x00FF00FF;
	counterL.w = counter.w & 0x00FF00FF;
	return counterL;
}

uint4 expand8to16H(uint4 counter)
{
	uint4 counterH;
	counterH.x = (counter.x & 0xFF00FF00) >> 8;
	counterH.y = (counter.y & 0xFF00FF00) >> 8;
	counterH.z = (counter.z & 0xFF00FF00) >> 8;
	counterH.w = (counter.w & 0xFF00FF00) >> 8;
	return counterH;
}

uint4 loadKey4(uint idx) {
    uint4 keyLocal = srcKey[idx];
    uint4 linearIdx = idx * 4u + uint4(0, 1, 2, 3);
    keyLocal.x = linearIdx.x < numKeys ? keyLocal.x : ~0u;
    keyLocal.y = linearIdx.y < numKeys ? keyLocal.y : ~0u;
    keyLocal.z = linearIdx.z < numKeys ? keyLocal.z : ~0u;
    keyLocal.w = linearIdx.w < numKeys ? keyLocal.w : ~0u;
    return keyLocal;
}

[numthreads(BLOCK_DIM, 1, 1)]
void radixSort1CS(uint3 blockIdx : SV_GroupID, uint3 threadIdx : SV_GroupThreadID)
{
	uint globalOffset = blockIdx.x * BLOCK_DIM;

	uint4 localCount = uint4(0, 0, 0, 0);

    // uint4 keyLocal = srcKey[threadIdx.x + globalOffset];
    uint4 keyLocal = loadKey4(threadIdx.x + globalOffset);
	keyLocal.x = (keyLocal.x >> passStart) & 0xF;
	keyLocal.y = (keyLocal.y >> passStart) & 0xF;
	keyLocal.z = (keyLocal.z >> passStart) & 0xF;
	keyLocal.w = (keyLocal.w >> passStart) & 0xF;
	count(localCount, keyLocal.x);
	count(localCount, keyLocal.y);
	count(localCount, keyLocal.z);
	count(localCount, keyLocal.w);

	scount0[threadIdx.x] = localCount;
	GroupMemoryBarrierWithGroupSync();

	if (threadIdx.x < BLOCK_DIM / 4)
	{
		localCount = scount0[threadIdx.x];
		localCount = localCount + scount0[threadIdx.x + 1 * BLOCK_DIM / 4];
		localCount = localCount + scount0[threadIdx.x + 2 * BLOCK_DIM / 4];
		localCount = localCount + scount0[threadIdx.x + 3 * BLOCK_DIM / 4];
		scount1[threadIdx.x] = localCount;
	}
	GroupMemoryBarrierWithGroupSync();

	// expand to 16-bit from 8-bit
	if (threadIdx.x < BLOCK_DIM / 16)
	{
		uint4 localCountH;
		localCount = expand8to16L(scount1[threadIdx.x]);
		localCountH = expand8to16H(scount1[threadIdx.x]);
		localCount = localCount + expand8to16L(scount1[threadIdx.x + 1 * BLOCK_DIM / 16]);
		localCountH = localCountH + expand8to16H(scount1[threadIdx.x + 1 * BLOCK_DIM / 16]);
		localCount = localCount + expand8to16L(scount1[threadIdx.x + 2 * BLOCK_DIM / 16]);
		localCountH = localCountH + expand8to16H(scount1[threadIdx.x + 2 * BLOCK_DIM / 16]);
		localCount = localCount + expand8to16L(scount1[threadIdx.x + 3 * BLOCK_DIM / 16]);
		localCountH = localCountH + expand8to16H(scount1[threadIdx.x + 3 * BLOCK_DIM / 16]);
		scount0[threadIdx.x] = localCount;
		scount0[threadIdx.x + BLOCK_DIM / 16] = localCountH;
	}
	GroupMemoryBarrierWithGroupSync();

	// two sets of 16 uint4 left to be reduced
	uint setID = threadIdx.x / (BLOCK_DIM / 2);
	uint setLaneID = threadIdx.x & (BLOCK_DIM / 2 - 1);
	if (setLaneID < BLOCK_DIM / 64)
	{
		uint offset = setID * BLOCK_DIM / 16;
		localCount = scount0[setLaneID + offset];
		localCount = localCount + scount0[setLaneID + 1 * BLOCK_DIM / 64 + offset];
		localCount = localCount + scount0[setLaneID + 2 * BLOCK_DIM / 64 + offset];
		localCount = localCount + scount0[setLaneID + 3 * BLOCK_DIM / 64 + offset];
		scount1[setLaneID + setID * BLOCK_DIM / 64] = localCount;
	}
	GroupMemoryBarrierWithGroupSync();

	// two sets of 4 uint4 left to be reduced
	if (setLaneID == 0)
	{
		uint offset = setID * BLOCK_DIM / 64;
		localCount = scount1[0 + offset];
		localCount = localCount + scount1[1 + offset];
		localCount = localCount + scount1[2 + offset];
		localCount = localCount + scount1[3 + offset];

		uint bucketOffset = 8 * setID;

		// output counter values for global scan
		counters[(bucketOffset + 0) * gridDim.x + blockIdx.x] = (localCount.x & 0x0000FFFF) >> 0;
		counters[(bucketOffset + 1) * gridDim.x + blockIdx.x] = (localCount.x & 0xFFFF0000) >> 16;
		counters[(bucketOffset + 2) * gridDim.x + blockIdx.x] = (localCount.y & 0x0000FFFF) >> 0;
		counters[(bucketOffset + 3) * gridDim.x + blockIdx.x] = (localCount.y & 0xFFFF0000) >> 16;
		counters[(bucketOffset + 4) * gridDim.x + blockIdx.x] = (localCount.z & 0x0000FFFF) >> 0;
		counters[(bucketOffset + 5) * gridDim.x + blockIdx.x] = (localCount.z & 0xFFFF0000) >> 16;
		counters[(bucketOffset + 6) * gridDim.x + blockIdx.x] = (localCount.w & 0x0000FFFF) >> 0;
		counters[(bucketOffset + 7) * gridDim.x + blockIdx.x] = (localCount.w & 0xFFFF0000) >> 16;

		// output counter values for local scan
		counters[(bucketOffset + 0) + 16 * (gridDim.x + blockIdx.x)] = (localCount.x & 0x0000FFFF) >> 0;
		counters[(bucketOffset + 1) + 16 * (gridDim.x + blockIdx.x)] = (localCount.x & 0xFFFF0000) >> 16;
		counters[(bucketOffset + 2) + 16 * (gridDim.x + blockIdx.x)] = (localCount.y & 0x0000FFFF) >> 0;
		counters[(bucketOffset + 3) + 16 * (gridDim.x + blockIdx.x)] = (localCount.y & 0xFFFF0000) >> 16;
		counters[(bucketOffset + 4) + 16 * (gridDim.x + blockIdx.x)] = (localCount.z & 0x0000FFFF) >> 0;
		counters[(bucketOffset + 5) + 16 * (gridDim.x + blockIdx.x)] = (localCount.z & 0xFFFF0000) >> 16;
		counters[(bucketOffset + 6) + 16 * (gridDim.x + blockIdx.x)] = (localCount.w & 0x0000FFFF) >> 0;
		counters[(bucketOffset + 7) + 16 * (gridDim.x + blockIdx.x)] = (localCount.w & 0xFFFF0000) >> 16;
	}
}
