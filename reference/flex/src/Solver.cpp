#include "Solver.h"
#include "Library.h"
#include "Data.h"
#include "RadixSort.h"
#include "Random.h"
#include "SPH.h"

namespace NvFlex {

struct SubstepParams {
    float kSubstepStart;
    float kSubstepEnd;
};

NvFlexLibrary *Solver::GetLibrary() {
    return mLib;
}

NvFlexSolverCallback Solver::RegisterCallback(NvFlexSolverCallbackStage stage,
                                              const NvFlexSolverCallback *callback) {
    NvFlexSolverCallback prev = {};
    if(stage >= 0 && stage < eNvFlexStageCount) {
        prev = mCallbacks[stage];
        if(callback)
            mCallbacks[stage] = *callback;
    }
    return prev;
}

void Solver::Update(float dt, int numSubsteps, bool enableTimers) {
    auto context = mLib->mContext;

    NvFlexContextPush(context);

    mTimerPool->enable(enableTimers);
    mTimerPool->clear();
    if(enableTimers)
        mTimerPool->reserve(mParams.numIterations * 10 * numSubsteps + 50 * numSubsteps);
    if(mLatencyTimerEnable) {
        mLatencyTimerIndex = mTimerPool->begin();
        mLatencyTimerRecordStatus = 1;
    }

    mLib->mTriangleMeshData->Rebuild(context, mTimerPool);

    mShapes->RebuildShapeData(mLib, mTimerPool);

    if(mNumParticles) {
        IterationState state = {};
        state.dt = dt;
        state.dta = dt / (float)numSubsteps;
        state.substepIdx = 0;
        state.numSubsteps = numSubsteps;
        state.kNumBlocks = divCeil<256>(mNumParticles);
        state.kNumBlocksHalf = divCeil<128>(mNumParticles);
        InitParams(state);
        for (int i = 0; i < numSubsteps; ++i) {
            state.substepIdx = i;
            UpdateSubstep(state);
        }
    }

    if (mLatencyTimerEnable) {
        mTimerPool->end(mLatencyTimerIndex);
        mLatencyTimerRecordStatus = 2;
    }

    NvFlexContextPop(context);
}

void Solver::SetParams(const NvFlexParams *params) {
    if(params->fluidRestDistance <= 0.f) {
        mFluidRestDensity = 0.f;
        mFluidDensityConstraintScale = 0.f;
        mFluidSurfaceConstraintScale = 0.f;
    } else if(params->radius != mParams.radius || params->fluidRestDistance != mParams.fluidRestDistance) {
        SPHCalculateRestDensity(params->fluidRestDistance, params->radius,
                                &mFluidRestDensity, &mFluidDensityConstraintScale,
                                &mFluidSurfaceConstraintScale);
    }
    mParams = *params;
}

const NvFlexParams &Solver::GetParams() {
    return mParams;
}

void Solver::SetActive(NvFlexBuffer *srcBuffer, const NvFlexCopyDesc *copyDesc) {
    CopyBufferImpl(mActiveIndices, srcBuffer, copyDesc);
}

void Solver::GetActive(NvFlexBuffer *dstBuffer, const NvFlexCopyDesc *copyDesc) {
    CopyBufferImpl(dstBuffer, mActiveIndices, copyDesc);
}

void Solver::SetActiveCount(int n) {
    mNumParticles = n;
    mNumParticlesAligned = n;
}

int Solver::GetActiveCount() {
    return mNumParticles;
}

void Solver::SetParticles(NvFlexBuffer *srcBuffer, const NvFlexCopyDesc *copyDesc) {
    CopyBufferImpl(mPositions, srcBuffer, copyDesc);
}

void Solver::GetParticles(NvFlexBuffer *dstBuffer, const NvFlexCopyDesc *copyDesc) {
    CopyBufferImpl(dstBuffer, mPositions, copyDesc);
}

void Solver::SetRestParticles(NvFlexBuffer *srcBuffer, const NvFlexCopyDesc *copyDesc) {
    CopyBufferImpl(mRestPositions, srcBuffer, copyDesc);
}

void Solver::GetRestParticles(NvFlexBuffer *dstBuffer, const NvFlexCopyDesc *copyDesc) {
    CopyBufferImpl(dstBuffer, mRestPositions, copyDesc);
}

void Solver::GetSmoothParticles(NvFlexBuffer *dstBuffer, const NvFlexCopyDesc *copyDesc) {
    CopyBufferImpl(dstBuffer, mStepCounter ? mSmoothPositionsOriginal : mPositions,
                   copyDesc);
}

void Solver::SetVelocities(NvFlexBuffer *srcBuffer, const NvFlexCopyDesc *copyDesc) {
    CopyBufferImpl(mVelocities, srcBuffer, copyDesc);
}

void Solver::GetVelocities(NvFlexBuffer *dstBuffer, const NvFlexCopyDesc *copyDesc) {
    CopyBufferImpl(dstBuffer, mVelocities, copyDesc);
}

void Solver::SetPhases(NvFlexBuffer *srcBuffer, const NvFlexCopyDesc *copyDesc) {
    CopyBufferImpl(mPhases, srcBuffer, copyDesc);
}

void Solver::GetPhases(NvFlexBuffer *dstBuffer, const NvFlexCopyDesc *copyDesc) {
    CopyBufferImpl(dstBuffer, mPhases, copyDesc);
}

void Solver::SetNormals(NvFlexBuffer *srcBuffer, const NvFlexCopyDesc *copyDesc) {
    CopyBufferImpl(mNormals, srcBuffer, copyDesc);
}

void Solver::GetNormals(NvFlexBuffer *dstBuffer, const NvFlexCopyDesc *copyDesc) {
    CopyBufferImpl(dstBuffer, mNormals, copyDesc);
}

void Solver::SetSprings(NvFlexBuffer *springIndices, NvFlexBuffer *springLengths,
                        NvFlexBuffer *springStiffness, int numSprings) {
    auto context = mLib->mContext;

    if (numSprings > mNumSprings) {
        NvFlexUint numToAlloc = CalculateSlack(numSprings);

        mSpringIndices.Create(context, 2 * numToAlloc, 
                                      "NvFlexSolver::mSpringIndices");

        mSpringLengths.Create(context, numToAlloc, 
                                      "NvFlexSolver::mSpringLengths");

        mSpringStiffness.Create(context, numToAlloc, 
                                        "NvFlexSolver::mSpringStiffness");
        mMaxSprings = numToAlloc;
    }

    if (numSprings) {
        CopyBufferImpl(mSpringIndices, 0, springIndices, 0, 2 * numSprings);
        CopyBufferImpl(mSpringLengths, 0, springLengths, 0, numSprings);
        CopyBufferImpl(mSpringStiffness, 0, springStiffness, 0, numSprings);
    }

    if (2 * numSprings > mMaxHalfSprings) {
        NvFlexUint elementCount = CalculateSlack(2 * numSprings);

        mHalfSpringIndices.Create(context, elementCount, 
                                          "NvFlexSolver::mHalfSpringIndices");

        mHalfSpringOpposites.Create(context, elementCount, 
                                            "NvFlexSolver::mHalfSpringOpposites");

        mHalfSpringLengths.Create(context, elementCount, 
                                          "NvFlexSolver::mHalfSpringLengths");

        mHalfSpringStiffness.Create(context, elementCount, 
                                            "NvFlexSolver::mHalfSpringStiffness");

        mHalfSpringParticleBegin.Create(context, mMaxParticles, 
                                                "NvFlexSolver::mHalfSpringParticleBegin");

        mHalfSpringParticleEnd.Create(context, mMaxParticles, 
                                              "NvFlexSolver::mHalfSpringParticleEnd");

        mHalfSpringLambdas.CreateWithZero(context, elementCount,
                                          "NvFlexSolver::mHalfSpringLambdas");

        mHalfSpringConstantBuffer.Create(context, sizeof(int), 1);
        mMaxHalfSprings = elementCount;
    }

    if (numSprings) {
        if (mRadixSortSize < 2 * numSprings) {
            RadixSortDesc sortDesc = {};
            sortDesc.maxSortBlocks = divCeil<1024>(2 * numSprings);
            SafeRelease(mRadixSort);
            mRadixSort = createRadixSort(context, &sortDesc);
            mRadixSortSize = 2 * numSprings;
        }

        auto vptr = mHalfSpringConstantBuffer.Map(context);
        *(int *)vptr = numSprings;
        mHalfSpringConstantBuffer.Unmap(context);

        auto input = mRadixSort->getBuffer();
        NvFlexContextCopyBuffer(context, input.key, 0, springIndices, 0,
                                2 * sizeof(int) * numSprings);

        SpringGenerateIndices(input.val, numSprings);

        RadixSortParams sortParams = {};
        sortParams.numKeys = 2 * numSprings;
        sortParams.numSortBlocks = divCeil<1024>(2 * numSprings);
        sortParams.bits = 32;
        mRadixSort->sort(context, &sortParams);

        auto output = mRadixSort->getBuffer();

        mLib->ClearBufferInt(mHalfSpringParticleBegin, sizeof(int) * mMaxParticles, 0);
        mLib->ClearBufferInt(mHalfSpringParticleEnd, sizeof(int) * mMaxParticles, 0);

        SpringFindParticleRange(output.key, numSprings);
        SpringRecordData(output.val, numSprings);
    }

    mNumSprings = numSprings;
}

void Solver::SetRigids(NvFlexBuffer *offsets, NvFlexBuffer *indices,
                       NvFlexBuffer *restPositions, NvFlexBuffer *restNormals,
                       NvFlexBuffer *stiffness, NvFlexBuffer *thresholds,
                       NvFlexBuffer *creeps, NvFlexBuffer *rotations,
                       NvFlexBuffer *translations, int numRigids, int numIndices) {
    auto context = mLib->mContext;

    if (numRigids) {
        if (numRigids > mMaxRigids) {
            NvFlexUint numToAlloc = CalculateSlack(numRigids);

            mRigidOffsets.Create(context, numToAlloc + 1, 
                                         "NvFlexSolver::mRigidOffsets");

            mRigidRotations.Create(context, numToAlloc, 
                                   "NvFlexSolver::mRigidRotations");

            mRigidTranslations.Create(context, numToAlloc, 
                                      "NvFlexSolver::mRigidTranslations");

            mRigidCoefficients.Create(context, numToAlloc, 
                                      "NvFlexSolver::mRigidCoefficients");

            mMaxRigids = numToAlloc;
        }

        if (numIndices > mMaxRigidIndices) {
            NvFlexUint elementCount = CalculateSlack(numIndices);

            mRigidIndices.Create(context, elementCount, 
                                 "NvFlexSolver::mRigidIndices");

            mRigidLocalPositions.Create(context, elementCount, 
                                        "NvFlexSolver::mRigidLocalPositions");

            mRigidLocalNormals.Create(context, elementCount, 
                                      "NvFlexSolver::mRigidLocalNormals");

            mMaxRigidIndices = elementCount;
        }

        CopyBufferImpl(mRigidRotations, 0, rotations, 0, numRigids);
        CopyBufferImpl(mRigidTranslations, 0, translations, 0, numRigids);

        CopyBufferImpl(mRigidOffsets, 0, offsets, 0, numRigids + 1);
        CopyBufferImpl(mRigidIndices, 0, indices, 0, numIndices);
        CopyBufferImpl(mRigidCoefficients, 0, stiffness, 0, numRigids);
        CopyBufferImpl(mRigidLocalPositions, 0, restPositions, 0, numIndices);
        if (restNormals)
            CopyBufferImpl(mRigidLocalNormals, 0, restNormals, 0, numIndices);
        else {
            mLib->ClearBufferFloat4(mRigidLocalNormals, numIndices * sizeof(float4),
                                    make_float4(0.f));
        }

        mNumRigids = numRigids;
        mNumRigidIndices = numIndices;

        if (thresholds && creeps) {
            if (mMaxPlasticEntries < mNumRigids) {
                mRigidPlasticThresholds.Create(context, mMaxRigids, 
                                                       "NvFlexSolver::mRigidPlasticThresholds");

                mRigidPlasticCreeps.Create(context, mMaxRigids, 
                                                   "NvFlexSolver::mRigidPlasticCreeps");

                mMaxPlasticEntries = mMaxRigids;
            }

            CopyBufferImpl(mRigidPlasticThresholds, 0, thresholds, 0, numRigids);
            CopyBufferImpl(mRigidPlasticCreeps, 0, creeps, 0, numRigids);
        } else {
            mRigidPlasticThresholds = nullptr;
            mRigidPlasticCreeps = nullptr;
            mMaxPlasticEntries = 0;
        }
    } else {
        mNumRigids = 0;
        mNumRigidIndices = 0;
    }
}

void Solver::GetRigids(NvFlexBuffer *offsets, NvFlexBuffer *indices,
                       NvFlexBuffer *restPositions, NvFlexBuffer *restNormals,
                       NvFlexBuffer *stiffness, NvFlexBuffer *thresholds,
                       NvFlexBuffer *creeps, NvFlexBuffer *rotations,
                       NvFlexBuffer *translations) {
    if (mNumRigids) {
        NvFlexUint numRigids = mNumRigids;
        NvFlexUint numRigidIndices = mNumRigidIndices;
        CopyBufferImpl(rotations, 0, mRigidRotations, 0, numRigids);
        CopyBufferImpl(translations, 0, mRigidTranslations, 0, numRigids);
        CopyBufferImpl(offsets, 0, mRigidOffsets, 0, numRigids + 1);
        CopyBufferImpl(indices, 0, mRigidIndices, 0, mNumRigidIndices);
        CopyBufferImpl(stiffness, 0, mRigidCoefficients, 0, numRigids);
        CopyBufferImpl(restPositions, 0, mRigidLocalPositions, 0, numRigidIndices);
        CopyBufferImpl(restNormals, 0, mRigidLocalNormals, 0, numRigidIndices);

        if (thresholds)
            CopyBufferImpl(thresholds, 0, mRigidPlasticThresholds, 0, numRigids);
        if (creeps)
            CopyBufferImpl(creeps, 0, mRigidPlasticCreeps, 0, numRigids);
    }
}

void Solver::SetShapes(NvFlexBuffer *geometry, NvFlexBuffer *shapePositions,
                       NvFlexBuffer *shapeRotations, NvFlexBuffer *prevShapePositions,
                       NvFlexBuffer *prevShapeRotations, NvFlexBuffer *shapeFlags,
                       int numShapes) {
    auto context = mLib->mContext;
    if (numShapes) {
        if (numShapes > mShapes->mMaxShapes) {
            NvFlexUint numToAlloc = CalculateSlack(numShapes);

            mShapes->mAabbMin.Create(context, numToAlloc, 
                                             "NvFlexSolver::mShapes::mAabbMin");

            mShapes->mAabbMax.Create(context, numToAlloc, 
                                             "NvFlexSolver::mShapes::mAabbMax");

            mShapes->mPositions.Create(context, numToAlloc, 
                                               "NvFlexSolver::mShapes::mPositions");

            mShapes->mRotations.Create(context, numToAlloc, 
                                               "NvFlexSolver::mShapes::mRotations");

            mShapes->mPrevPositions.Create(context, numToAlloc, 
                                                   "NvFlexSolver::mShapes::mPrevPositions");

            mShapes->mPrevRotations.Create(context, numToAlloc, 
                                                   "NvFlexSolver::mShapes::mPrevRotations");

            mShapes->mFlags.Create(context, numToAlloc, 
                                           "NvFlexSolver::mShapes::mFlags");

            mShapes->mGeometry.Create(context, numToAlloc, 
                                              "NvFlexSolver::mShapes::mGeometry");

            mShapes->mMaxShapes = numToAlloc;
        }

        CopyBufferImpl(mShapes->mGeometry, 0, geometry, 0, numShapes);
        CopyBufferImpl(mShapes->mPositions, 0, shapePositions, 0, numShapes);
        CopyBufferImpl(mShapes->mRotations, 0, shapeRotations, 0, numShapes);

        if (!prevShapePositions)
            prevShapePositions = shapePositions;
        if (!prevShapeRotations)
            prevShapeRotations = shapeRotations;
        CopyBufferImpl(mShapes->mPrevPositions, 0, prevShapePositions, 0, numShapes);
        CopyBufferImpl(mShapes->mPrevRotations, 0, prevShapeRotations, 0, numShapes);

        CopyBufferImpl(mShapes->mFlags, 0, shapeFlags, 0, numShapes);
        mShapes->mNumShapes = numShapes;
        mShapes->mRebuildShapeData = 1;
    } else {
        mShapes->mNumShapes = 0;
    }
}

void Solver::SetDynamicTriangles(NvFlexBuffer *triangles, NvFlexBuffer *normals,
                                 NvFlexUint numTris) {
    auto context = mLib->mContext;
    if (numTris) {
        if (numTris > mDynamicMaxTris) {
            NvFlexUint numToAlloc = CalculateSlack(numTris);

            mDynamicTriangles.Create(context, 3 * numToAlloc, 
                                     "NvFlexSolver::mDynamicTriangles");

            mDynamicTriangleNormals.Create(context, numToAlloc, 
                                           "NvFlexSolver::mDynamicTriangleNormals");

            mDynamicMaxTris = numToAlloc;
        }

        CopyBufferImpl(mDynamicTriangles, 0, triangles, 0, 3 * numTris);
        CopyBufferImpl(mDynamicTriangleNormals, 0, normals, 0, numTris);

        mDynamicNumTris = numTris;
    } else {
        mDynamicNumTris = 0;
    }
}

void Solver::GetDynamicTriangles(NvFlexBuffer *triangles, NvFlexBuffer *normals,
                                 NvFlexUint numTris) {
    if (numTris > mDynamicNumTris)
        NVFLEX_ASSERT(0);

    CopyBufferImpl(triangles, 0, mDynamicTriangles, 0, numTris * 3);
    CopyBufferImpl(normals, 0, mDynamicTriangleNormals, 0, numTris);
}

void Solver::SetInflatables(NvFlexBuffer *startTris, NvFlexBuffer *numTris,
                            NvFlexBuffer *restVolumes, NvFlexBuffer *overPressures,
                            NvFlexBuffer *constraintScales, NvFlexUint n) {
    auto context = mLib->mContext;
    if (n > mMaxInflatables) {
        NvFlexUint numToAlloc = CalculateSlack(n);

        mInflatables.Create(context, numToAlloc,  "NvFlexSolver::mInflatables");

        mPressures.CreateWithZero(context, numToAlloc,  "NvFlexSolver::mPressures");

        mInflatableLambdas.CreateWithZero(context, numToAlloc,
                                          "NvFlexSolver::mInflatableLambdas");

        mMaxInflatables = numToAlloc;
    }

    if (n) {
        VectorCached<InflatableDevice> inflatables((size_t)n);

        auto pStartTris = (const int *)NvFlexContextMapBuffer(context, startTris, eNvFlexStagingCpuAccess_read, false);
        auto pNumTris = (const int *)NvFlexContextMapBuffer(
            context, numTris, eNvFlexStagingCpuAccess_read, false);
        auto pRestVolumes = (const float *)NvFlexContextMapBuffer(
            context, restVolumes, eNvFlexStagingCpuAccess_read, false);
        auto pOverPressures = (const float *)NvFlexContextMapBuffer(
            context, overPressures, eNvFlexStagingCpuAccess_read, false);
        auto pConstraintScales = (const float *)NvFlexContextMapBuffer(
            context, constraintScales, eNvFlexStagingCpuAccess_read, false);

        for (NvFlexUint i = 0; i < n; ++i) {
            auto inf = &inflatables[i];
            inf->mStartTri = pStartTris[i];
            inf->mNumTris = pNumTris[i];
            inf->mRestVolume = pRestVolumes[i] * pOverPressures[i];
            inf->mConstraintScale = pConstraintScales[i];
        }

        NvFlexContextUnmapBuffer(context, startTris);
        NvFlexContextUnmapBuffer(context, numTris);
        NvFlexContextUnmapBuffer(context, restVolumes);
        NvFlexContextUnmapBuffer(context, overPressures);
        NvFlexContextUnmapBuffer(context, constraintScales);

        mInflatables.Write(context, 0, n, inflatables.data());
    }

    mNumInflatables = n;
}

void Solver::GetDensities(NvFlexBuffer *d, const NvFlexCopyDesc *desc) {
    if(d)
        CopyBufferImpl(d, mDensities, desc);
}

void Solver::GetAnisotropy(NvFlexBuffer *q1, NvFlexBuffer *q2, NvFlexBuffer *q3,
                           const NvFlexCopyDesc *desc) {
    CopyBufferImpl(q1, mAnisotropy1, desc);
    CopyBufferImpl(q2, mAnisotropy2, desc);
    CopyBufferImpl(q3, mAnisotropy3, desc);
}

void Solver::GetDiffuseParticles(NvFlexBuffer *p, NvFlexBuffer *v, NvFlexBuffer *count) {
    if(mMaxDiffuseParticles) {
        CopyBufferImpl(p, 0, mDiffusePositions, 0, mMaxDiffuseParticles);
        CopyBufferImpl(v, 0, mDiffuseVelocities, 0, mMaxDiffuseParticles);
        CopyBufferImpl(count, 0, mNumDiffuseParticles, 0, 1);
    }
}

void Solver::SetDiffuseParticles(NvFlexBuffer *p, NvFlexBuffer *v, NvFlexUint n) {
    auto context = mLib->mContext;
    if(n > mMaxDiffuseParticles)
        NVFLEX_ASSERT(0);

    CopyBufferImpl(mDiffusePositions, 0, p, 0, n);
    CopyBufferImpl(mDiffuseVelocities, 0, v, 0, n);

    mNumDiffuseParticles.Write(context, 0, 1, (const int *)&n);
}

void Solver::GetContacts(NvFlexBuffer *planes, NvFlexBuffer *velocities,
                         NvFlexBuffer *indices, NvFlexBuffer *counts) {
    auto context = mLib->mContext;
    CopyBufferImpl(planes, 0, mStaticContactPlanes, 0,
                   mMaxParticles * mMaxContactsPerParticle);
    CopyBufferImpl(velocities, 0, mStaticContactVelocities, 0,
                   mMaxParticles * mMaxContactsPerParticle);
    CopyBufferImpl(indices, 0, mReverseLookup, 0, mMaxParticles);
    CopyBufferImpl(counts, 0, mStaticContactCounts, 0, mMaxParticles);
}

void Solver::GetNeighbors(NvFlexBuffer *neighbors, NvFlexBuffer *counts,
                          NvFlexBuffer *apiToInternal, NvFlexBuffer *internalToApi) {
    auto context = mLib->mContext;
    CopyBufferImpl(neighbors, 0, mParticleNeighbors, 0,
                   mMaxParticles * mMaxContactsPerParticle);
    CopyBufferImpl(counts, 0, mParticleNeighborCounts, 0, mMaxParticles);
    CopyBufferImpl(apiToInternal, 0, mReverseLookup, 0, mMaxParticles);
    auto sortedCellHash = mRadixSort->getBuffer();
    CopyBufferImpl(internalToApi, 0, sortedCellHash.val, 0, mMaxParticles);
}

void Solver::GetBounds(NvFlexBuffer *lower, NvFlexBuffer *upper) {
    auto context = mLib->mContext;
    CopyBufferImpl(lower, 0, mParticleBounds, 0, 1);
    CopyBufferImpl(upper, 0, mParticleBounds, 1, 1);
}

float Solver::GetDeviceLatency(NvFlexUint64 *gpuStartStamp, NvFlexUint64 *gpuEndStamp,
                              NvFlexUint64 *gpuFreq) {
    float latencyTimeVal = 0.f;
    if (mLatencyTimerEnable && mLatencyTimerRecordStatus == 2) {
        mTimerPool->get(mLatencyTimerIndex, &latencyTimeVal, nullptr, gpuStartStamp,
                        gpuEndStamp, gpuFreq);
    } else
        mLatencyTimerEnable = 1;
    return latencyTimeVal;
}

void Solver::GetTimers(NvFlexTimers *timers) {
    struct TimeQueryEntry {
        const wchar_t *label;
        float *time;
    } timeQueryTable[] = {{L"Predicate", &timers->predict},
                          {L"CalculateParticleHash", &timers->createCellIndices},
                          {L"SortParticleHash", &timers->sortCellIndices},
                          {L"CreateGrid", &timers->createGrid},
                          {L"RecordParticles", &timers->reorder},
                          {L"CollideParticles", &timers->collideParticles},
                          {L"CollideShapes", &timers->collideShapes},
                          {L"CollideTriangles", &timers->collideTriangles},
                          {L"CollideFields", &timers->collideFields},
                          {L"CalculateDensity", &timers->calculateDensity},
                          {L"SolveDensity", &timers->solveDensities},
                          {L"UpdateVelocities", &timers->solveVelocities},
                          {L"CalculateVorticity", &timers->solveVelocities},
                          {L"SolveVelocities", &timers->solveVelocities},
                          {L"SolveShapes", &timers->solveShapes},
                          {L"SolveSprings", &timers->solveSprings},
                          {L"SolveContact", &timers->solveContacts},
                          {L"SolveInflatables", &timers->solveInflatables},
                          {L"ApplyDeltas", &timers->applyDeltas},
                          {L"SmoothPositions", &timers->calculateAnisotropy},
                          {L"CalculateAnisotropy", &timers->calculateAnisotropy},
                          {L"CreateDiffuseParticles", &timers->updateDiffuse},
                          {L"UpdateDiffuseParticles", &timers->updateDiffuse},
                          {L"ClampDiffuseParticleCount", &timers->updateDiffuse},
                          {L"CompactDiffuseParticles", &timers->updateDiffuse},
                          {L"UpdateTriangles", &timers->updateTriangles},
                          {L"UpdateNormals", &timers->updateNormals},
                          {L"NormalizeNormals", &timers->updateNormals},
                          {L"Finalize", &timers->finalize},
                          {L"ComputeBounds", &timers->updateBounds}};

    memset(timers, 0, sizeof(*timers));
    float currTime;
    float totalTime = 0.f;
    NvFlexResult rc;
    for(const auto &entry : timeQueryTable) {
        rc = mTimerPool->get(entry.label, &currTime);
        NVFLEX_ASSERT(rc == eNvFlexSuccess);
        *entry.time += currTime;
        totalTime += currTime;
    }

    timers->total = totalTime;
}

NvFlexUint Solver::GetDetailTimers(NvFlexDetailTimer **timers) {
    NvFlexUint numTimers = 0;
    *timers = mDetailTimers;

    numTimers = mTimerPool->getStatistics(mDetailTimers, mNumDetailTimers);

    float latencyTimeVal = 0.f;
    if(mLatencyTimerEnable) {
        if(mLatencyTimerRecordStatus == 2) {
            mTimerPool->get(mLatencyTimerIndex, &latencyTimeVal, nullptr, nullptr, nullptr,
                            nullptr);
        }
    } else {
        mLatencyTimerEnable = 1;
    }

    if(numTimers < mNumDetailTimers) {
        strcpy(mDetailTimers[numTimers].name, "Total");
        mDetailTimers[numTimers].time = latencyTimeVal;
        ++numTimers;
    }

    return numTimers;
}

Solver::Solver(NvFlexLibrary *lib, const NvFlexSolverDesc *desc)
    : mLib{implCast<Library>(lib)},
      mKernelParamsHost{},
      mKernelParams{},
      mPositions{},
      mNewPositions{},
      mVelocities{},
      mRestPositions{},
      mDeltas{},
      mPhases{},
      mActiveIndices{},
      mDensities{},
      mPotentials{},
      mCurl{},
      mNormals{},
      mSortedNormals{},
      mNormalsTemp{},
      mAnisotropy1{},
      mAnisotropy2{},
      mAnisotropy3{},
      mSortedPositions{},
      mSortedVelocities{},
      mSortedNewPositions{},
      mSortedNewVelocities{},
      mSortedPhases{},
      mSortedDensities{},
      mStaticContactPlanes{},
      mStaticContactVelocities{},
      mStaticContactCounts{},
      mMaxNeighborsPerParticle{},
      mParticleNeighbors{},
      mParticleNeighborCounts{},
      mMaxContactsPerParticle{},
      mPlanes{},
      mSpringIndices{},
      mSpringLengths{},
      mSpringStiffness{},
      mNumSprings{},
      mMaxSprings{},
      mHalfSpringIndices{},
      mHalfSpringOpposites{},
      mHalfSpringLengths{},
      mHalfSpringStiffness{},
      mHalfSpringParticleBegin{},
      mHalfSpringParticleEnd{},
      mHalfSpringLambdas{},
      mHalfSpringConstantBuffer{},
      mMaxHalfSprings{},
      mRigidOffsets{},
      mRigidIndices{},
      mRigidLocalPositions{},
      mRigidLocalNormals{},
      mRigidCoefficients{},
      mRigidPlasticThresholds{},
      mRigidPlasticCreeps{},
      mRigidRotations{},
      mRigidTranslations{},
      mNumRigids{},
      mMaxRigids{},
      mNumRigidIndices{},
      mMaxRigidIndices{},
      mMaxPlasticEntries{},
      mDynamicTriangles{},
      mDynamicTriangleNormals{},
      mDynamicNumTris{},
      mDynamicMaxTris{},
      mDynamicTriangleEdges{},
      mDynamicNumTriEdges{},
      mPressures{},
      mInflatableLambdas{},
      mInflatables{},
      mNumInflatables{},
      mMaxInflatables{},
      mDiffusePositions{},
      mDiffuseVelocities{},
      mDiffusePositionsNew{},
      mDiffuseVelocitiesNew{},
      mNumDiffuseParticles{},
      mNumDiffuseParticlesNew{},
      mMaxDiffuseParticles{},
      mNumActiveDiffuseParticles{},
      mDiffuseSortSize{},
      mDiffuseLaunchArg{},
      mDiffuseRate{},
      mDiffuseTimer{},
      mCellIds{},
      mReverseLookup{},
      mNumParticles{},
      mNumParticlesAligned{},
      mMaxParticles{},
      mParticleBounds{},
      mBoundsLower{},
      mBoundsUpper{},
      mGroupBoundsLower{},
      mGroupBoundsUpper{},
      mDesc{*desc},
      mParams{},
      mSDFShapeDX{},
      mSDFDeviceShapes{},
      mShapes{},
      mDebug{},
      mRandom{},
      mFluidRestDensity{},
      mFluidDensityConstraintScale{},
      mFluidSurfaceConstraintScale{},
      mStepCounter{},
      mCallbacks{},
      mSmoothPositionsOriginal{},
      mRadixSortSize{},
      mRadixSort{},
      mSubstepParams{},
      mTestFloatResult{},
      mTestFloat4Result{},
      mTestIntResult{},
      mTimerPool{},
      mNumDetailTimers{},
      mDetailTimers{},
      mLatencyTimerRecordStatus{},
      mLatencyTimerEnable{},
      mLatencyTimerIndex{},
      mIterState{} {}

Solver::~Solver() {
    for(auto &sdfShape : mSDFDeviceShapes)
        delete sdfShape;
    delete mShapes;
    
    SafeRelease(mRadixSort);
    SafeRelease(mTimerPool);
    Allocable::deallocate(mDetailTimers);

    mLib->GetResourceTracker()->remove(static_cast<NvFlexSolver *>(this),
                                       NvResourceTracker::eSolver);
}

bool Solver::Init() {
    auto context = mLib->GetContext();
    NvFlexConstantBufferDesc cbDesc = {};
    NvFlexBufferDesc bufDesc = {};
    void *vptr;

    mLib->ResetContext(true);

    mKernelParams.Create(context, 1, 1);

    constexpr int kTestLength = 64;
    mTestFloatResult.CreateWithZero(context, kTestLength,  "NvFlexSolver::mTestFloatResult");
    mTestFloat4Result.CreateWithZero(context, kTestLength, 
                                     "NvFlexSolver::mTestFloat4Result");
    mTestIntResult.CreateWithZero(context, kTestLength, 
                                  "NvFlexSolver::mTestIntResult");

    mDebug = 0;
    mNumParticles = 0;
    mNumParticlesAligned = 0;
    mMaxParticles = mDesc.maxParticles;
    mMaxNeighborsPerParticle = mDesc.maxNeighborsPerParticle;
    mMaxContactsPerParticle = mDesc.maxContactsPerParticle;

    const NvFlexUint maxParticle = mDesc.maxParticles;
    const NvFlexUint maxParticlesAligned = alignUp<256>(maxParticle);
    const NvFlexUint maxDiffuseParticles = mDesc.maxDiffuseParticles;

    auto initialActiveIndices = (int *)Allocable::allocate(sizeof(int) * maxParticle);
    for (NvFlexUint i = 0; i < maxParticle; ++i)
        initialActiveIndices[i] = i;
    mActiveIndices.Create(context, maxParticle, 
                          "NvFlexSolver::mActiveIndices");
    mActiveIndices.Write(context, 0, maxParticle, initialActiveIndices);
    Allocable::deallocate(initialActiveIndices);

    mPositions.CreateWithZero(context, maxParticle, 
                              "NvFlexSolver::mPositions");

    mNewPositions.CreateWithZero(context, maxParticle, 
                                 "NvFlexSolver::mNewPositions");

    mRestPositions.CreateWithZero(context, maxParticle, 
                                  "NvFlexSolver::mRestPositions");

    mVelocities.CreateWithZero(context, maxParticle, 
                               "NvFlexSolver::mVelocities");

    mNormals.CreateWithZero(context, maxParticle,  "NvFlexSolver::mNormals");

    mSortedNormals.CreateWithZero(context, maxParticle, 
                                  "NvFlexSolver::mSortNormals");

    mNormalsTemp.CreateWithZero(context, 4 * maxParticle, 
                                "NvFlexSolver::mNormalsTemp");

    mDeltas.CreateWithZero(context, 4 * maxParticle, 
                           "NvFlexSolver::mDeltas");

    mParticleNeighbors.CreateWithZero(context,
                                      maxParticlesAligned * mMaxNeighborsPerParticle,
                                       "NvFlexSolver::mParticleNeighbors");

    mParticleNeighborCounts.CreateWithZero(context, maxParticle, 
                                           "NvFlexSolver::mParticleNeighborCounts");

    mSortedPositions.CreateWithZero(context, maxParticle, 
                                    "NvFlexSolver::mSortedPositions");

    mSortedNewPositions.CreateWithZero(context, maxParticle, 
                                       "NvFlexSolver::mSortedNewPositions");

    mSortedNewVelocities.CreateWithZero(context, maxParticle, 
                                        "NvFlexSolver::mSortedNewVelocities");

    mStaticContactPlanes.CreateWithZero(context, mMaxContactsPerParticle * maxParticle,
                                        
                                        "NvFlexSolver::mStaticContactPlanes");

    mStaticContactVelocities.CreateWithZero(context, mMaxContactsPerParticle * maxParticle,
                                            
                                            "NvFlexSolver::mStaticContactVelocities");

    mStaticContactCounts.CreateWithZero(context, maxParticle, 
                                        "NvFlexSolver::mStaticContactCounts");

    mDensities.CreateWithZero(context, maxParticle, 
                              "NvFlexSolver::mDensities");

    mPhases.CreateWithZero(context, maxParticle,  "NvFlexSolver::mPhases");

    mSortedPhases.CreateWithZero(context, maxParticle, 
                                 "NvFlexSolver::mSortedPhases");

    mPotentials.CreateWithZero(context, maxParticle, 
                               "NvFlexSolver::mPotentials");

    mCurl.CreateWithZero(context, maxParticle,  "NvFlexSolver::mCurl");

    {
        VectorCached<float4> anisoData1(maxParticle, make_float4(1.f, 0.f, 0.f, 0.1f)),
            anisoData2(maxParticle, make_float4(0.f, 1.f, 0.f, 0.1f)),
            anisoData3(maxParticle, make_float4(0.f, 0.f, 1.f, 0.1f));

        mAnisotropy1.Create(context, maxParticle, 
                                    "NvFlexSolver::mAnisotropy1");
        mAnisotropy1.Write(context, 0, maxParticle, anisoData1.data());

        mAnisotropy2.Create(context, maxParticle, 
                                    "NvFlexSolver::mAnisotropy2");
        mAnisotropy2.Write(context, 0, maxParticle, anisoData2.data());

        mAnisotropy3.Create(context, maxParticle, 
                                    "NvFlexSolver::mAnisotropy3");
        mAnisotropy3.Write(context, 0, maxParticle, anisoData3.data());
    }

    mSortedDensities.CreateWithZero(context, maxParticle, 
                                    "NvFlexSolver::mSortedDensities");

    mParticleBounds.CreateWithZero(context, 2, 
                                   "NvFlexSolver::mParticleBounds");

    NvFlexUint maxGroupBounds = divCeil<128>(divCeil<128>(maxParticle));

    mBoundsLower.CreateWithZero(context, divCeil<128>(maxParticle), 
                                "NvFlexSolver::mBoundsLower");

    mBoundsUpper.CreateWithZero(context, divCeil<128>(maxParticle), 
                                "NvFlexSolver::mBoundsUpper");

    mGroupBoundsLower.CreateWithZero(context, maxGroupBounds, 
                                     "NvFlexSolver::mGroupBoundsLower");

    mGroupBoundsUpper.CreateWithZero(context, maxGroupBounds, 
                                     "NvFlexSolver::mGroupBoundsUpper");

    RadixSortDesc radixSortDesc = {};
    radixSortDesc.maxSortBlocks = divCeil<1024>(maxParticle);
    mRadixSort = createRadixSort(context, &radixSortDesc);
    mRadixSortSize = maxParticle;

    mReverseLookup.CreateWithZero(context, maxParticle, 
                                  "NvFlexSolver::mReverseLookup");

    {
        VectorCached<float> random((size_t)0x100);
        for (auto &v : random)
            v = Randf(-1.f, 1.f);
        mRandom.Create(context, 0x100,  "NvFlexSolver::mRandom");
        mRandom.Write(context, 0, random.size(), random.data());
    }

    if (maxDiffuseParticles) {
        mDiffusePositions.CreateWithZero(context, maxDiffuseParticles, 
                                         "NvFlexSolver::mDiffusePositions");

        mDiffuseVelocities.CreateWithZero(context, maxDiffuseParticles, 
                                          "NvFlexSolver::mDiffuseVelocities");

        mDiffusePositionsNew.CreateWithZero(context, maxDiffuseParticles, 
                                            "NvFlexSolver::mDiffusePositionsNew");

        mDiffuseVelocitiesNew.CreateWithZero(context, maxDiffuseParticles, 
                                             "NvFlexSolver::mDiffuseVelocitiesNew");

        mDiffuseSortSize.CreateWithZero(context, 1, 
                                        "NvFlexSolver::mDiffuseSortSize");

        mNumDiffuseParticles.CreateWithZero(context, 1, 
                                            "NvFlexBuffer::mNumDiffuseParticles");

        mNumDiffuseParticlesNew.CreateWithZero(context, 1, 
                                               "NvFlexBuffer::mNumDiffuseParticlesNew");

        mMaxDiffuseParticles = maxDiffuseParticles;
        mDiffuseRate = 0.05f / 3.f;
        mDiffuseTimer = 0.f;
    }

    memset(&mParams, 0, sizeof(mParams));
    mParams.relaxationMode = eNvFlexRelaxationLocal;
    mParams.relaxationFactor = 1.f;
    mParams.maxSpeed = FLT_MAX;
    mParams.maxAcceleration = FLT_MAX;
    mParams.numIterations = 1;
    mParams.solverMode = eNvFlexSolverPBD;
    mParams.stiffnessMin = 1.0e3f;
    mParams.stiffnessMax = 1.0e9f;
    mParams.springDamping = 0.f;
    mParams.volumeCompliance = 0.f;

    mShapes = new ShapeData;

    mSortedVelocities.CreateWithZero(context, maxParticle, 
                                     "NvFlexSolver::mSortedVelocities");

    mSmoothPositionsOriginal.CreateWithZero(context, maxParticle, 
                                            "NvFlexSolver::mSmoothPositionsOriginal");

    mSubstepParams.Create(context, sizeof(SubstepParams), 1);

    mTimerPool = new TimerPool(context);
    mLatencyTimerEnable = 0;
    mNumDetailTimers = 256;

    {
        constexpr int byteBufferLen = 256;
        constexpr int detailTimerElemWidth =
            sizeof(NvFlexDetailTimer) + byteBufferLen * sizeof(char);
        constexpr int detailTimerElemStride = detailTimerElemWidth;
        mDetailTimers = (NvFlexDetailTimer *)Allocable::allocate(mNumDetailTimers *
                                                                 detailTimerElemWidth);
        memset(mDetailTimers, 0, mNumDetailTimers * detailTimerElemWidth);
        char *bptr = (char *)(mDetailTimers + mNumDetailTimers);
        for (int k = 0; k < mNumDetailTimers; ++k, bptr += byteBufferLen) {
            mDetailTimers[k].name = bptr;
        }
    }

    mLib->ExecuteContext();

    mLib->GetResourceTracker()->add(static_cast<NvFlexSolver *>(this),
                                    NvResourceTracker::eSolver);

    return true;
}

void Solver::InitParams(const IterationState &state) {
    float dt = state.dta;
    int numSubsteps = state.numSubsteps;
    KernelParams p;

    memcpy(p.kPlanes, mParams.planes, sizeof(mParams.planes));
    float h = mParams.radius;
    p.kNumPlanes = mParams.numPlanes;
    h = mParams.radius;
    p.kRadius = h;
    p.kRadiusSq = h * h;
    p.kInvRadius = 1.f / h;
    p.kRestDensity = mFluidRestDensity;
    p.kInvRestDensity = 1.f / mFluidRestDensity;
    p.kLambdaScale = 1.f / mFluidDensityConstraintScale;
    p.kFluidRestDistance = mParams.fluidRestDistance;
    p.kSolidRestDistance = mParams.solidRestDistance;
    p.kVorticityConfinement = mParams.vorticityConfinement;
    p.kSolidPressure = mParams.solidPressure;
    p.kBuoyancy = mParams.buoyancy;
    p.kViscosity = mParams.viscosity * p.kInvRestDensity;
    p.kFreeSurfaceDrag = mParams.freeSurfaceDrag;
    p.kPoly6 = 315.f / (pi() * 64.f * h * h * h * h * h * h * h * h * h);
    p.kSpiky1 = 15.f / (pi() * h * h * h);
    p.kSpiky2 = 30.f / (pi() * h * h * h * h);
    float rest = mParams.fluidRestDistance / mParams.radius;
    p.kCohesion1 = -(rest + 1.0) / sqr(rest);
    p.kCohesion2 = ((sqr(rest) + rest) + 1.0) / sqr(rest);
    p.kSurfaceTension =
        (float)(p.kInvRestDensity * mParams.surfaceTension) / mFluidSurfaceConstraintScale;
    p.kCohesion = mParams.cohesion * h;
    p.kAdhesion = mParams.adhesion;
    p.kGravity = *(float3 *)mParams.gravity;
    p.kDynamicFriction = mParams.dynamicFriction;
    p.kStaticFriction = mParams.staticFriction;
    p.kParticleFriction = mParams.particleFriction;
    p.kDiffuseThreshold = mMaxDiffuseParticles ? mParams.diffuseThreshold : 0.f;
    p.kDiffuseBuoyancy = mParams.diffuseBuoyancy;
    p.kDiffuseDrag = mParams.diffuseDrag;
    p.kDiffuseBallistic = mParams.diffuseBallistic;
    p.kDiffuseLifetime = mParams.diffuseLifetime;
    p.kAnisotropy = mParams.anisotropyScale;
    p.kAnisotropyMin = mParams.anisotropyMin * mParams.radius;
    p.kAnisotropyMax = mParams.anisotropyMax * mParams.radius;
    p.kSmoothing = mParams.smoothing;
    p.kDissipation = mParams.dissipation;
    p.kDamping = mParams.damping;
    p.kDrag = mParams.drag * 60.0;
    p.kLift = mParams.lift * 60.0;
    p.kRestitution = mParams.restitution;
    p.kCollisionDistance = mParams.collisionDistance;
    p.kCollisionThreshold = mParams.shapeCollisionMargin;
    p.kCollisionMargin = p.kCollisionDistance + p.kCollisionThreshold;
    p.kCollisionMarginSq = sqr(p.kCollisionDistance + p.kCollisionThreshold);
    p.kSleepThreshold = mParams.sleepThreshold;
    p.kSleepThresholdSq = sqr(mParams.sleepThreshold);
    p.kShockPropagation = mParams.shockPropagation;
    p.kSOR = 1.0 / (float)(mParams.relaxationFactor + 1.0);
    p.kRelaxationFactor = mParams.relaxationFactor;
    p.kRelaxationMode = mParams.relaxationMode;
    p.kNumParticles = mNumParticles;
    p.kNumParticlesAligned = mNumParticlesAligned;
    p.kMaxParticles = mMaxParticles;
    p.kDt = dt;
    p.kInvDt = 1.0 / dt;
    p.kMaxSpeed = mParams.maxSpeed;
    p.kMaxAcceleration = mParams.maxAcceleration;
    p.kMaxVelocityDelta = mParams.maxAcceleration * dt;
    p.kSubStepLength = 1.0 / (float)numSubsteps;
    p.kMaxNeighborsPerParticle = mMaxNeighborsPerParticle;
    p.kNumShapes = mShapes->mNumShapes;
    p.kShapesCount = mSDFDeviceShapes.size();
    p.kInvCellEdge = 1.0 / (float)(mParams.radius + mParams.particleCollisionMargin);
    p.kCollideParticlesRadiusSq = sqr(mParams.radius + mParams.particleCollisionMargin);
    p.kNumRigids = mNumRigids;
    p.kNumSprings = mNumSprings;
    p.kNumTriangles = mDynamicNumTris;
    p.kWind = *(float3 *)mParams.wind;
    p.kMaxDiffuseParticles = mMaxDiffuseParticles;
    p.kDiffuseDt = dt * (float)numSubsteps;
    p.kDiffuseMaxVelocity = FLT_MAX;
    p.kMaxContactsPerParticle = mMaxContactsPerParticle;
    p.kSolverMode = mParams.solverMode;
    {
        // [0,1] stiffness coefficient -> [stiffnessMin, stiffnessMax] N/m, geometric,
        // compliance = 1 / stiffness (blog.mmacklin.com, "XPBD slides and stiffness")
        float stiffnessMin = std::max(mParams.stiffnessMin, 1.0e-6f);
        float stiffnessMax = std::max(mParams.stiffnessMax, stiffnessMin);
        p.kInvStiffnessMin = 1.f / stiffnessMin;
        p.kLogStiffnessRange = log2f(stiffnessMax / stiffnessMin);
    }
    p.kSpringDamping = mParams.springDamping;
    p.kVolumeCompliance = mParams.volumeCompliance;
    p._padXpbd[0] = p._padXpbd[1] = p._padXpbd[2] = 0.f;

    auto vptr = mKernelParams.Map(mLib->mContext);
    memcpy(vptr, &p, sizeof(p));
    mKernelParams.Unmap(mLib->mContext);
    mKernelParamsHost = p;
}

void Solver::UpdateSubstep(const IterationState &state) {
    Predict(state);
    ComputeBounds(state);
    CalculateParticleHash(state);
    SortParticleHash(state);
    CreateGrid(state);
    RecorderParticles(state);
    CollideParticles(state);
    ExecuteCallback(eNvFlexStageSubstepBegin, state.dta);
    mLib->ClearBufferInt(mStaticContactCounts, sizeof(int) * mMaxParticles, 0);
    CollideTriangles(state);
    CollideShapes(state);
    ContinuousShockPropagation(state);

    if (mParams.solverMode == eNvFlexSolverXPBD) {
        // XPBD: Lagrange multipliers persist across iterations and reset per substep
        if (mNumSprings)
            mLib->ClearBufferInt(mHalfSpringLambdas, sizeof(float) * mMaxHalfSprings, 0);
        if (mNumInflatables)
            mLib->ClearBufferInt(mInflatableLambdas, sizeof(float) * mMaxInflatables, 0);
    }

    for (int i = 0; i < mParams.numIterations; ++i) {
        ExecuteCallback(eNvFlexStageIterationStart, state.dta);
        CalculateDensity(state);
        SolveDensity(state);
        SolveSprings(state);
        CalculateAndSolveInflatables(state);
        SolveShapes(state);
        ApplyDeltas(state);
        SolveContact(state);
        ExecuteCallback(eNvFlexStageIterationEnd, state.dta);
    }

    UpdateVelocities(state);
    ExecuteCallback(eNvFlexStageSubstepEnd, state.dta);

    if (state.substepIdx == state.numSubsteps - 1)
        ExecuteCallback(eNvFlexStageUpdateEnd, state.dt);

    CalculateVorticity(state);
    SolveVelocities(state);

    if(mDynamicNumTris) {
        UpdateTriangles(state);
        UpdateNormals(state);
        NormalizeNormals(state);
    }
    Finalize(state);

    mDiffuseTimer += state.dta;

    if(state.substepIdx == state.numSubsteps - 1) {
        if(mMaxDiffuseParticles) {
            CreateDiffuse(state);
            UpdateDiffuse(state);
            ClampDiffuse(state);
            CompactDiffuse(state);
            swap(mDiffusePositions, mDiffusePositionsNew);
            swap(mDiffuseVelocities, mDiffuseVelocitiesNew);
            swap(mNumDiffuseParticles, mNumDiffuseParticlesNew);
        }

        SmoothPositions(state);
        CalculateAnisotropy(state);
    }

    LazyClearGrid(state);

    if (mLib->mGpuVendorId == VENDOR_ID_NVIDIA)
        NvFlexContextClearState(mLib->mContext);

    ++mStepCounter;
}

void Solver::Predict(const IterationState &state) {
    NVFLEX_PROFILE_SECTION("Predicate", mTimerPool);
    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderPredict;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mNewPositions);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mVelocities);
    params.readOnly[0] = NvFlexBufferGetResource(mActiveIndices);
    params.readOnly[1] = NvFlexBufferGetResource(mPositions);
    params.readOnly[2] = NvFlexBufferGetResource(mPhases);
    params.gridDim = make_dim(state.kNumBlocks, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::ComputeBounds(const IterationState &state) {
    NVFLEX_PROFILE_SECTION("ComputeBounds", mTimerPool);
    NvFlexUint kNumGroupBoundsBlocks = divCeil<128>(state.kNumBlocksHalf);
    if(kNumGroupBoundsBlocks > 0x80)
        NVFLEX_ASSERT(0 && "kNumGroupBoundsBlocks <= kNumThreadsPerBlockHalf");

    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderCalculateBounds;
    params.rootConstantBuffer = mKernelParams;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mBoundsLower);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mBoundsUpper);
    params.readWrite[2] = NvFlexBufferGetResourceRW(mParticleBounds);
    params.readOnly[0] = NvFlexBufferGetResource(mActiveIndices);
    params.readOnly[1] = NvFlexBufferGetResource(mNewPositions);
    params.gridDim = make_dim(state.kNumBlocksHalf, 1, 1);
    NvFlexContextDispatch(mLib->mContext, &params);

    memset(&params, 0, sizeof(params));
    params.shader = mLib->mShaderCalculateBoundsGroup;
    params.rootConstantBuffer = mKernelParams;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mGroupBoundsLower);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mGroupBoundsUpper);
    params.readWrite[2] = NvFlexBufferGetResourceRW(mParticleBounds);
    params.readOnly[2] = NvFlexBufferGetResource(mBoundsLower);
    params.readOnly[3] = NvFlexBufferGetResource(mBoundsUpper);
    params.gridDim = make_dim(kNumGroupBoundsBlocks, 1, 1);
    NvFlexContextDispatch(mLib->mContext, &params);

    memset(&params, 0, sizeof(params));
    params.shader = mLib->mShaderCalculateBoundsFinalize;
    params.rootConstantBuffer = mKernelParams;
    params.readWrite[2] = NvFlexBufferGetResourceRW(mParticleBounds);
    params.readOnly[2] = NvFlexBufferGetResource(mGroupBoundsLower);
    params.readOnly[3] = NvFlexBufferGetResource(mGroupBoundsUpper);
    params.gridDim = make_dim(1);
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::CalculateParticleHash(const IterationState &) {
    NVFLEX_PROFILE_SECTION("CalculateParticleHash", mTimerPool);
    NvFlexUint kRadixSortNumBlocks = divCeil<256>(alignUp<1024>(mNumParticles));
    auto cellHash = mRadixSort->getBuffer();

    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderCalculateParticleHash;
    params.readWrite[0] = NvFlexBufferGetResourceRW(cellHash.key);
    params.readWrite[1] = NvFlexBufferGetResourceRW(cellHash.val);
    params.readOnly[0] = NvFlexBufferGetResource(mActiveIndices);
    params.readOnly[1] = NvFlexBufferGetResource(mNewPositions);
    params.readOnly[2] = NvFlexBufferGetResource(mParticleBounds);
    params.gridDim = make_dim(kRadixSortNumBlocks, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::SortParticleHash(const IterationState &) {
    if(mNumParticles) {
        NVFLEX_PROFILE_SECTION("SortParticleHash", mTimerPool);
        RadixSortParams sortParams = {};
        sortParams.numKeys = mNumParticles;
        sortParams.numSortBlocks = divCeil<1024>(mNumParticles);
        sortParams.bits = 24;
        mRadixSort->sort(mLib->mContext, &sortParams);
    }
}

void Solver::CreateGrid(const IterationState &state) {
    NVFLEX_PROFILE_SECTION("CreateGrid", mTimerPool);
    NvFlexDispatchParams params = {};
    auto sortedCellHash = mRadixSort->getBuffer();
    params.shader = mLib->mShaderCreateGrid;
    params.readWrite[0] = mLib->mCellBucketStarts;
    params.readWrite[1] = mLib->mCellBucketEnds;
    params.readOnly[0] = NvFlexBufferGetResource(sortedCellHash.key);
    params.gridDim = make_dim(state.kNumBlocks, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::RecorderParticles(const IterationState &state) {
    NVFLEX_PROFILE_SECTION("RecordParticles", mTimerPool);
    auto sortedCellHash = mRadixSort->getBuffer();
    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderReorderParticles;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mSortedPositions);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mSortedNewPositions);
    params.readWrite[2] = NvFlexBufferGetResourceRW(mSortedVelocities);
    params.readWrite[3] = NvFlexBufferGetResourceRW(mSortedPhases);
    params.readWrite[4] = NvFlexBufferGetResourceRW(mReverseLookup);
    params.readWrite[5] = NvFlexBufferGetResourceRW(mSortedNormals);
    params.readOnly[0] = NvFlexBufferGetResource(mPositions);
    params.readOnly[1] = NvFlexBufferGetResource(mNewPositions);
    params.readOnly[2] = NvFlexBufferGetResource(mVelocities);
    params.readOnly[3] = NvFlexBufferGetResource(mPhases);
    params.readOnly[4] = NvFlexBufferGetResource(sortedCellHash.val);
    params.readOnly[5] = NvFlexBufferGetResource(mNormals);
    params.gridDim = make_dim(state.kNumBlocks, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::CollideParticles(const IterationState &state) {
    NVFLEX_PROFILE_SECTION("CollideParticles", mTimerPool);
    auto sortedCellHash = mRadixSort->getBuffer();
    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderCollideParticles;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mParticleNeighbors);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mParticleNeighborCounts);
    params.readOnly[0] = NvFlexBufferGetResource(mLib->mCellBucketStarts);
    params.readOnly[1] = NvFlexBufferGetResource(mLib->mCellBucketEnds);
    params.readOnly[2] = NvFlexBufferGetResource(sortedCellHash.val);
    params.readOnly[3] = NvFlexBufferGetResource(mSortedNewPositions);
    params.readOnly[4] = NvFlexBufferGetResource(mRestPositions);
    params.readOnly[5] = NvFlexBufferGetResource(mSortedPhases);
    params.readOnly[6] = NvFlexBufferGetResource(mParticleBounds);
    params.gridDim = make_dim(state.kNumBlocks, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::CollideTriangles(const IterationState &state) {
    NVFLEX_PROFILE_SECTION("CollideTriangles", mTimerPool);

    auto context = mLib->mContext;
    SubstepParams substepParams = {};
    substepParams.kSubstepStart = (float)state.substepIdx / (float)state.numSubsteps;
    substepParams.kSubstepEnd = (float)(state.substepIdx + 1) / (float)state.numSubsteps;
    auto vptr = mSubstepParams.Map(context);
    memcpy(vptr, &substepParams, sizeof(substepParams));
    mSubstepParams.Unmap(context);

    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderCollideTriangles;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mStaticContactCounts);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mStaticContactPlanes);
    params.readWrite[2] = NvFlexBufferGetResourceRW(mStaticContactVelocities);

    auto meshDevice = mLib->mTriangleMeshData;
    if(meshDevice->mVerticesArray) {
        params.readOnly[0] = NvFlexBufferGetResource(meshDevice->mVerticesArray);
        params.readOnly[1] = NvFlexBufferGetResource(meshDevice->mIndicesArray);
        params.readOnly[2] = NvFlexBufferGetResource(meshDevice->mBvhData.mNodeLowersArray);
        params.readOnly[3] = NvFlexBufferGetResource(meshDevice->mBvhData.mNodeUppersArray);
        params.readOnly[4] = NvFlexBufferGetResource(meshDevice->mBvhData.mRootNodeArray);
    }

    params.readOnly[5] = NvFlexBufferGetResource(mSortedPositions);
    params.readOnly[6] = NvFlexBufferGetResource(mSortedNewPositions);
    params.readOnly[7] = NvFlexBufferGetResource(mSortedPhases);

    if(mShapes->mNumShapes > 0) {
        params.readOnly[8] = NvFlexBufferGetResource(mShapes->mPrevPositions);
        params.readOnly[9] = NvFlexBufferGetResource(mShapes->mPrevRotations);
        params.readOnly[10] = NvFlexBufferGetResource(mShapes->mFlags);
        params.readOnly[11] = NvFlexBufferGetResource(mShapes->mPositions);
        params.readOnly[12] = NvFlexBufferGetResource(mShapes->mRotations);
        params.readOnly[13] = NvFlexBufferGetResource(mShapes->mGeometry);
        params.readOnly[14] = NvFlexBufferGetResource(mShapes->mBVH.mRootNode);
        params.readOnly[15] = NvFlexBufferGetResource(mShapes->mBVH.mNodeLowers);
        params.readOnly[16] = NvFlexBufferGetResource(mShapes->mBVH.mNodeUppers);
    }

    if(meshDevice->mVerticesArray) {
        params.readOnly[17] = NvFlexBufferGetResource(meshDevice->mTriMeshTableDevice);
    }

    params.gridDim = make_dim(state.kNumBlocks, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    params.secondConstantBuffer = mSubstepParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::CollideShapes(const IterationState &state) {
    NVFLEX_PROFILE_SECTION("CollideShapes", mTimerPool);

    auto context = mLib->mContext;
    SubstepParams substepParams = {};
    substepParams.kSubstepStart = (float)state.substepIdx / (float)state.numSubsteps;
    substepParams.kSubstepEnd = (float)(state.substepIdx + 1) / (float)state.numSubsteps;
    auto vptr = mSubstepParams.Map(context);
    memcpy(vptr, &substepParams, sizeof(substepParams));
    mSubstepParams.Unmap(context);

    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderCollideShapes;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mStaticContactCounts);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mStaticContactPlanes);
    params.readWrite[2] = NvFlexBufferGetResourceRW(mStaticContactVelocities);

    if(mShapes->mNumShapes > 0) {
        params.readOnly[0] = NvFlexBufferGetResource(mShapes->mGeometry);
        params.readOnly[1] = NvFlexBufferGetResource(mShapes->mPositions);
        params.readOnly[2] = NvFlexBufferGetResource(mShapes->mRotations);
        params.readOnly[3] = NvFlexBufferGetResource(mShapes->mPrevPositions);
        params.readOnly[4] = NvFlexBufferGetResource(mShapes->mPrevRotations);
        params.readOnly[5] = NvFlexBufferGetResource(mShapes->mFlags);
    }
    params.readOnly[6] = NvFlexBufferGetResource(mSortedPositions);
    params.readOnly[7] = NvFlexBufferGetResource(mSortedNewPositions);
    params.readOnly[8] = NvFlexBufferGetResource(mSortedPhases);

    if(mShapes->mNumShapes > 0) {
        params.readOnly[9] = NvFlexBufferGetResource(mShapes->mBVH.mRootNode);
        params.readOnly[10] = NvFlexBufferGetResource(mShapes->mBVH.mNodeLowers);
        params.readOnly[11] = NvFlexBufferGetResource(mShapes->mBVH.mNodeUppers);
    }

    if(mLib->mConvexMeshData->mNumConvexes) {
        params.readOnly[12] = NvFlexBufferGetResource(mLib->mConvexMeshData->mConvexes);
        params.readOnly[13] = NvFlexBufferGetResource(mLib->mConvexMeshData->mPlanes);
    }

    if(mLib->mSDFData->mNumSDFs) {
        params.readOnly[14] = NvFlexBufferGetResource(mLib->mSDFData->mSDFs);
        for (int i = 0; i < 16; ++i) {
            if(mLib->mSDFData->mTextures[i]) {
                params.readOnly[i + 15] =
                    NvFlexTexture3DGetResource(mLib->mSDFData->mTextures[i]);
            }
        }
    }

    params.gridDim = make_dim(state.kNumBlocksHalf, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    params.secondConstantBuffer = mSubstepParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::ContinuousShockPropagation(const IterationState &state) {
    if(mParams.shockPropagation > 0.f) {
        NVFLEX_PROFILE_SECTION("ContinuousShockPropagation", mTimerPool);
        NvFlexDispatchParams params = {};
        params.shader = mLib->mShaderContinuousShockPropagation;
        params.readWrite[0] = NvFlexBufferGetResourceRW(mSortedNewPositions);
        params.readOnly[0] = NvFlexBufferGetResource(mParticleBounds);
        params.gridDim = make_dim(state.kNumBlocks, 1, 1);
        params.rootConstantBuffer = mKernelParams;
        NvFlexContextDispatch(mLib->mContext, &params);
    }
}

void Solver::CalculateDensity(const IterationState &state) {
    if(mParams.fluidRestDistance > 0.f) {
        NVFLEX_PROFILE_SECTION("CalculateDensity", mTimerPool);
        auto sortedCellHash = mRadixSort->getBuffer();
        NvFlexDispatchParams params = {};
        params.readWrite[0] = NvFlexBufferGetResourceRW(mDensities);
        params.readWrite[1] = NvFlexBufferGetResourceRW(mNormals);
        params.readWrite[2] = NvFlexBufferGetResourceRW(mSortedNormals);
        params.readOnly[0] = NvFlexBufferGetResource(mParticleNeighbors);
        params.readOnly[1] = NvFlexBufferGetResource(mParticleNeighborCounts);
        params.readOnly[2] = NvFlexBufferGetResource(mSortedNewPositions);
        params.readOnly[3] = NvFlexBufferGetResource(mSortedPhases);
        params.readOnly[4] = NvFlexBufferGetResource(sortedCellHash.val);
        params.gridDim = make_dim(state.kNumBlocks, 1, 1);
        if(mParams.surfaceTension <= 0.f) {
            params.shader = mLib->mShaderCalculateDensity;
        } else {
            params.shader = mLib->mShaderCalculateDensitySurfaceTension;
        }
        NvFlexContextDispatch(mLib->mContext, &params);
    }
}

void Solver::SolveDensity(const IterationState &state) {
    NVFLEX_PROFILE_SECTION("SolveDensity", mTimerPool);
    NvFlexDispatchParams params = {};
    params.readWrite[0] = NvFlexBufferGetResourceRW(mDeltas);
    params.readOnly[0] = NvFlexBufferGetResource(mParticleNeighbors);
    params.readOnly[1] = NvFlexBufferGetResource(mParticleNeighborCounts);
    params.readOnly[2] = NvFlexBufferGetResource(mSortedNewPositions);
    params.readOnly[3] = NvFlexBufferGetResource(mSortedPhases);
    params.readOnly[4] = NvFlexBufferGetResource(mDensities);
    params.readOnly[5] = NvFlexBufferGetResource(mNormals);
    params.readOnly[6] = NvFlexBufferGetResource(mSortedPositions);
    params.readOnly[7] = NvFlexBufferGetResource(mSortedNormals);
    params.gridDim = make_dim(state.kNumBlocks, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    if(mDesc.featureMode == eNvFlexFeatureModeSimpleSolids) {
        params.shader = mLib->mShaderSolveDensitiesNonFluid;
    } else if(mParams.surfaceTension <= 0.f) {
        params.shader = mLib->mShaderSolveDensities;
    } else {
        params.shader = mLib->mShaderSolveDensitiesSurfaceTension;
    }
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::SolveSprings(const IterationState &) {
    if(mNumSprings) {
        NVFLEX_PROFILE_SECTION("SolveSprings", mTimerPool);
        const NvFlexUint kNumParticleBlocks = divCeil<512>(8 * mMaxParticles);
        NvFlexDispatchParams params = {};
        params.shader = mLib->mShaderSolveSprings;
        params.readWrite[0] = NvFlexBufferGetResourceRW(mDeltas);
        params.readOnly[0] = NvFlexBufferGetResource(mHalfSpringIndices);
        params.readOnly[1] = NvFlexBufferGetResource(mHalfSpringOpposites);
        params.readOnly[2] = NvFlexBufferGetResource(mHalfSpringLengths);
        params.readOnly[3] = NvFlexBufferGetResource(mHalfSpringStiffness);
        params.readOnly[4] = NvFlexBufferGetResource(mHalfSpringParticleBegin);
        params.readOnly[5] = NvFlexBufferGetResource(mHalfSpringParticleEnd);
        params.readOnly[6] = NvFlexBufferGetResource(mReverseLookup);
        params.readOnly[7] = NvFlexBufferGetResource(mSortedNewPositions);
        if (mParams.solverMode == eNvFlexSolverXPBD) {
            params.shader = mLib->mShaderSolveSpringsXPBD;
            params.readWrite[1] = NvFlexBufferGetResourceRW(mHalfSpringLambdas);
            params.readOnly[8] = NvFlexBufferGetResource(mSortedPositions);
        }
        params.gridDim = make_dim(kNumParticleBlocks, 1, 1);
        params.rootConstantBuffer = mKernelParams;
        NvFlexContextDispatch(mLib->mContext, &params);
    }
}

void Solver::CalculateAndSolveInflatables(const IterationState &) {
    if(mNumInflatables) {
        NVFLEX_PROFILE_SECTION("SolveInflatables", mTimerPool);
        NvFlexUint kNumInflatables = mNumInflatables;
        NvFlexDispatchParams params = {};
        params.shader = mLib->mShaderCalculateInflatableVolume;
        params.readWrite[0] = NvFlexBufferGetResourceRW(mPressures);
        params.readOnly[0] = NvFlexBufferGetResource(mInflatables);
        params.readOnly[1] = NvFlexBufferGetResource(mSortedNewPositions);
        params.readOnly[2] = NvFlexBufferGetResource(mReverseLookup);
        params.readOnly[3] = NvFlexBufferGetResource(mDynamicTriangles);
        if (mParams.solverMode == eNvFlexSolverXPBD) {
            params.shader = mLib->mShaderCalculateInflatableVolumeXPBD;
            params.readWrite[1] = NvFlexBufferGetResourceRW(mInflatableLambdas);
        }
        params.gridDim = make_dim(kNumInflatables, 1, 1);
        params.rootConstantBuffer = mKernelParams;
        NvFlexContextDispatch(mLib->mContext, &params);

        memset(&params, 0, sizeof(params));
        params.shader = mLib->mShaderSolveInflatableVolume;
        params.readWrite[0] = NvFlexBufferGetResourceRW(mDeltas);
        params.readOnly[0] = NvFlexBufferGetResource(mInflatables);
        params.readOnly[1] = NvFlexBufferGetResource(mSortedNewPositions);
        params.readOnly[2] = NvFlexBufferGetResource(mReverseLookup);
        params.readOnly[3] = NvFlexBufferGetResource(mDynamicTriangles);
        params.readOnly[4] = NvFlexBufferGetResource(mPressures);
        params.gridDim = make_dim(kNumInflatables, 1, 1);
        params.rootConstantBuffer = mKernelParams;
        NvFlexContextDispatch(mLib->mContext, &params);
    }
}

void Solver::SolveShapes(const IterationState &) {
    if(mNumRigids) {
        NVFLEX_PROFILE_SECTION("SolveShapes", mTimerPool);
        const NvFlexUint kNumRidgeBlocks = mNumRigids;
        NvFlexDispatchParams params = {};
        params.readWrite[0] = NvFlexBufferGetResourceRW(mDeltas);
        params.readWrite[1] = NvFlexBufferGetResourceRW(mRigidLocalPositions);
        params.readWrite[2] = NvFlexBufferGetResourceRW(mSortedNewPositions);
        params.readWrite[3] = NvFlexBufferGetResourceRW(mRigidRotations);
        params.readWrite[4] = NvFlexBufferGetResourceRW(mRigidTranslations);
        params.readWrite[5] = NvFlexBufferGetResourceRW(mNormals);
        params.readOnly[0] = NvFlexBufferGetResource(mRigidOffsets);
        params.readOnly[1] = NvFlexBufferGetResource(mRigidIndices);
        params.readOnly[2] = NvFlexBufferGetResource(mRigidCoefficients);
        if(mRigidPlasticThresholds && mRigidPlasticCreeps) {
            params.readOnly[3] = NvFlexBufferGetResource(mRigidPlasticThresholds);
            params.readOnly[4] = NvFlexBufferGetResource(mRigidPlasticCreeps);
        }
        params.readOnly[5] = NvFlexBufferGetResource(mReverseLookup);
        params.readOnly[6] = NvFlexBufferGetResource(mSortedPositions);
        params.readOnly[7] = NvFlexBufferGetResource(mRigidLocalNormals);
        params.gridDim = make_dim(kNumRidgeBlocks, 1, 1);
        params.rootConstantBuffer = mKernelParams;

        NvFlexUint avgWorkload;
        if(mParams.solverMode == eNvFlexSolverXPBD) {
            params.shader = (mRigidPlasticThresholds && mRigidPlasticCreeps)
                                ? mLib->mShaderSolveShapesPlasticDeformationXPBD
                                : mLib->mShaderSolveShapesXPBD;
        } else if(mRigidPlasticThresholds && mRigidPlasticCreeps) {
            if(mLib->mIsSHFLSupported && mLib->mIsFP32ATOMICSupported) {
                avgWorkload = mNumRigidIndices / mNumRigids;
                if(avgWorkload > 32) {
                    if(avgWorkload < 128)
                        params.shader = mLib->mShaderSolveShapesPlasticDeformation;
                    else
                        params.shader = mLib->mShaderSolveShapesPlasticDeformation128;
                } else
                    params.shader = mLib->mShaderSolveShapesPlasticDeformation32;
            } else
                params.shader = mLib->mShaderSolveShapesPlasticDeformation;
        } else if(mLib->mIsSHFLSupported && mLib->mIsFP32ATOMICSupported) {
            avgWorkload = mNumRigidIndices / mNumRigids;
            if (avgWorkload > 32) {
                if (avgWorkload < 128)
                    params.shader = mLib->mShaderSolveShapes;
                else
                    params.shader = mLib->mShaderSolveShapes128;
            } else
                params.shader = mLib->mShaderSolveShapes32;
        } else {
            params.shader = mLib->mShaderSolveShapes;
        }

        NvFlexContextDispatch(mLib->mContext, &params);
    }
}

void Solver::ApplyDeltas(const IterationState &state) {
    NVFLEX_PROFILE_SECTION("ApplyDeltas", mTimerPool);
    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderApplyDeltas;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mSortedNewPositions);
    params.readOnly[0] = NvFlexBufferGetResource(mDeltas);
    params.gridDim = make_dim(state.kNumBlocks, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::SolveContact(const IterationState &state) {
    NVFLEX_PROFILE_SECTION("SolveContact", mTimerPool);
    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderSolveContactsSequential;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mSortedNewPositions);
    params.readOnly[0] = NvFlexBufferGetResource(mStaticContactCounts);
    params.readOnly[1] = NvFlexBufferGetResource(mStaticContactPlanes);
    params.readOnly[2] = NvFlexBufferGetResource(mSortedPositions);
    params.readOnly[3] = NvFlexBufferGetResource(mStaticContactVelocities);
    params.gridDim = make_dim(state.kNumBlocks, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::UpdateVelocities(const IterationState &state) {
    NVFLEX_PROFILE_SECTION("UpdateVelocities", mTimerPool);
    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderUpdateVelocities;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mSortedNewVelocities);
    params.readOnly[0] = NvFlexBufferGetResource(mSortedPositions);
    params.readOnly[1] = NvFlexBufferGetResource(mSortedNewPositions);
    params.gridDim = make_dim(state.kNumBlocks, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::CalculateVorticity(const IterationState &state) {
    if(mParams.vorticityConfinement > 0.f) {
        NVFLEX_PROFILE_SECTION("CalculateVorticity", mTimerPool);
        NvFlexDispatchParams params = {};
        params.shader = mLib->mShaderCalculateVorticity;
        params.readWrite[0] = NvFlexBufferGetResourceRW(mCurl);
        params.readOnly[0] = NvFlexBufferGetResource(mSortedNewPositions);
        params.readOnly[1] = NvFlexBufferGetResource(mSortedNewVelocities);
        params.readOnly[2] = NvFlexBufferGetResource(mSortedPhases);
        params.readOnly[3] = NvFlexBufferGetResource(mParticleNeighbors);
        params.readOnly[4] = NvFlexBufferGetResource(mParticleNeighborCounts);
        params.readOnly[5] = NvFlexBufferGetResource(mStaticContactPlanes);
        params.readOnly[6] = NvFlexBufferGetResource(mStaticContactCounts);
        params.gridDim = make_dim(state.kNumBlocks, 1, 1);
        params.rootConstantBuffer = mKernelParams;
        NvFlexContextDispatch(mLib->mContext, &params);
    }
}

void Solver::SolveVelocities(const IterationState &state) {
    NVFLEX_PROFILE_SECTION("SolveVelocities", mTimerPool);
    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderSolveVelocities;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mSortedNewPositions);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mDeltas);
    params.readWrite[2] = NvFlexBufferGetResourceRW(mSortedDensities);
    params.readWrite[3] = NvFlexBufferGetResourceRW(mPotentials);
    params.readOnly[0] = NvFlexBufferGetResource(mParticleNeighbors);
    params.readOnly[1] = NvFlexBufferGetResource(mParticleNeighborCounts);
    params.readOnly[2] = NvFlexBufferGetResource(mStaticContactPlanes);
    params.readOnly[3] = NvFlexBufferGetResource(mStaticContactCounts);
    params.readOnly[4] = NvFlexBufferGetResource(mSortedPositions);
    params.readOnly[5] = NvFlexBufferGetResource(mSortedVelocities);
    params.readOnly[6] = NvFlexBufferGetResource(mSortedNewVelocities);
    params.readOnly[7] = NvFlexBufferGetResource(mSortedPhases);
    params.readOnly[8] = NvFlexBufferGetResource(mCurl);
    params.gridDim = make_dim(state.kNumBlocks, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::UpdateTriangles(const IterationState &) {
    NVFLEX_PROFILE_SECTION("UpdateTriangles", mTimerPool);
    NvFlexUint kNumTriBlocks = divCeil<256>(mDynamicNumTris);
    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderUpdateTriangles;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mDeltas);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mDynamicTriangleNormals);
    params.readOnly[0] = NvFlexBufferGetResource(mSortedNewPositions);
    params.readOnly[1] = NvFlexBufferGetResource(mSortedNewVelocities);
    params.readOnly[2] = NvFlexBufferGetResource(mReverseLookup);
    params.readOnly[3] = NvFlexBufferGetResource(mDynamicTriangles);
    params.rootConstantBuffer = mKernelParams;
    params.gridDim = make_dim(kNumTriBlocks, 1, 1);
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::UpdateNormals(const IterationState &state) {

    if(state.substepIdx == state.numSubsteps - 1) {
        NVFLEX_PROFILE_SECTION("UpdateNormals", mTimerPool);
        const NvFlexUint kNumMaxBlocks = divCeil<256>(mMaxParticles);
        const NvFlexUint kNumTriBlocks = divCeil<256>(mDynamicNumTris);
        NvFlexDispatchParams params = {};
        params.shader = mLib->mShaderUpdateVertexNormalsInit;
        params.readWrite[0] = NvFlexBufferGetResourceRW(mNormalsTemp);
        params.readOnly[0] = NvFlexBufferGetResource(mReverseLookup);
        params.readOnly[1] = NvFlexBufferGetResource(mDynamicTriangles);
        params.readOnly[2] = NvFlexBufferGetResource(mDynamicTriangleNormals);
        params.rootConstantBuffer = mKernelParams;
        params.gridDim = make_dim(kNumMaxBlocks, 1, 1);
        NvFlexContextDispatch(mLib->mContext, &params);

        params.gridDim = make_dim(kNumTriBlocks, 1, 1);
        params.shader = mLib->mShaderUpdateVertexNormals;
        NvFlexContextDispatch(mLib->mContext, &params);
    }
}

void Solver::NormalizeNormals(const IterationState &state) {
    if(state.substepIdx == state.numSubsteps - 1) {
        NVFLEX_PROFILE_SECTION("NormalizeNormals", mTimerPool);
        auto sortedCellHash = mRadixSort->getBuffer();
        NvFlexDispatchParams params = {};
        params.shader = mLib->mShaderNormalizeVertexNormals;
        params.readWrite[0] = NvFlexBufferGetResourceRW(mNormals);
        params.readOnly[0] = NvFlexBufferGetResource(sortedCellHash.val);
        params.readOnly[1] = NvFlexBufferGetResource(mNormalsTemp);
        params.gridDim = make_dim(state.kNumBlocks, 1, 1);
        params.rootConstantBuffer = mKernelParams;
        NvFlexContextDispatch(mLib->mContext, &params);
    }
}

void Solver::Finalize(const IterationState &state) {
    NVFLEX_PROFILE_SECTION("Finalize", mTimerPool);
    auto sortedCellHash = mRadixSort->getBuffer();
    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderFinalize;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mPositions);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mVelocities);
    params.readWrite[2] = NvFlexBufferGetResourceRW(mDensities);
    params.readOnly[0] = NvFlexBufferGetResource(mSortedNewPositions);
    params.readOnly[1] = NvFlexBufferGetResource(mSortedNewVelocities);
    params.readOnly[2] = NvFlexBufferGetResource(mSortedDensities);
    params.readOnly[3] = NvFlexBufferGetResource(mDeltas);
    params.readOnly[4] = NvFlexBufferGetResource(sortedCellHash.val);
    params.gridDim = make_dim(state.kNumBlocks, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::CreateDiffuse(const IterationState &state) {
    if(mDiffuseTimer >= mDiffuseRate) {
        NVFLEX_PROFILE_SECTION("CreateDiffuseParticles", mTimerPool);
        mDiffuseTimer = fmodf(mDiffuseTimer, mDiffuseRate);
        NvFlexDispatchParams params = {};
        params.shader = mLib->mShaderCreateDiffuseParticles;
        params.readWrite[0] = NvFlexBufferGetResourceRW(mNumDiffuseParticles);
        params.readWrite[1] = NvFlexBufferGetResourceRW(mDiffusePositions);
        params.readWrite[2] = NvFlexBufferGetResourceRW(mDiffuseVelocities);
        params.readOnly[0] = NvFlexBufferGetResource(mPotentials);
        params.readOnly[1] = NvFlexBufferGetResource(mSortedNewPositions);
        params.readOnly[2] = NvFlexBufferGetResource(mSortedNewVelocities);
        params.readOnly[3] = NvFlexBufferGetResource(mRandom);
        params.gridDim = make_dim(state.kNumBlocks, 1, 1);
        params.rootConstantBuffer = mKernelParams;
        NvFlexContextDispatch(mLib->mContext, &params);
    }
}

void Solver::UpdateDiffuse(const IterationState &) {
    NVFLEX_PROFILE_SECTION("UpdateDiffuseParticles", mTimerPool);
    const NvFlexUint kNumDiffuseBlocks = divCeil<256>(mMaxDiffuseParticles);
    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderUpdateDiffuseParticles;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mDiffusePositions);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mDiffuseVelocities);
    params.readOnly[0] = NvFlexBufferGetResource(mNumDiffuseParticles);
    params.readOnly[1] = NvFlexBufferGetResource(mLib->mCellBucketStarts);
    params.readOnly[2] = NvFlexBufferGetResource(mLib->mCellBucketEnds);
    params.readOnly[3] = NvFlexBufferGetResource(mParticleNeighborCounts);
    params.readOnly[4] = NvFlexBufferGetResource(mSortedNewPositions);
    params.readOnly[5] = NvFlexBufferGetResource(mSortedNewVelocities);
    params.readOnly[6] = NvFlexBufferGetResource(mSortedPhases);
    params.readOnly[7] = NvFlexBufferGetResource(mParticleBounds);
    params.gridDim = make_dim(kNumDiffuseBlocks, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::ClampDiffuse(const IterationState &) {
    NVFLEX_PROFILE_SECTION("ClampDiffuseParticleCount", mTimerPool);
    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderClampDiffuseParticleCount;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mNumDiffuseParticles);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mNumDiffuseParticlesNew);
    params.gridDim = make_dim(1);
    params.rootConstantBuffer = mKernelParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::CompactDiffuse(const IterationState &) {
    NVFLEX_PROFILE_SECTION("CompactDiffuseParticles", mTimerPool);
    const NvFlexUint kNumDiffuseBlocks = divCeil<256>(mMaxDiffuseParticles);
    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderCompactDiffuseParticles;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mNumDiffuseParticlesNew);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mDiffusePositionsNew);
    params.readWrite[2] = NvFlexBufferGetResourceRW(mDiffuseVelocitiesNew);
    params.readOnly[0] = NvFlexBufferGetResource(mNumDiffuseParticles);
    params.readOnly[1] = NvFlexBufferGetResource(mDiffusePositions);
    params.readOnly[2] = NvFlexBufferGetResource(mDiffuseVelocities);
    params.gridDim = make_dim(kNumDiffuseBlocks, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::SmoothPositions(const IterationState &state) {
    if(mParams.anisotropyScale > 0.f || mParams.smoothing > 0.f) {
        NVFLEX_PROFILE_SECTION("SmoothPositions", mTimerPool);
        auto sortedCellHash = mRadixSort->getBuffer();
        NvFlexDispatchParams params = {};
        params.shader = mLib->mShaderSmoothPositions;
        params.readWrite[0] = NvFlexBufferGetResourceRW(mSortedPositions);
        params.readWrite[1] = NvFlexBufferGetResourceRW(mSmoothPositionsOriginal);
        params.readOnly[0] = NvFlexBufferGetResource(mParticleNeighbors);
        params.readOnly[1] = NvFlexBufferGetResource(mParticleNeighborCounts);
        params.readOnly[2] = NvFlexBufferGetResource(mSortedPhases);
        params.readOnly[3] = NvFlexBufferGetResource(mSortedNewPositions);
        params.readOnly[4] = NvFlexBufferGetResource(sortedCellHash.val);
        params.gridDim = make_dim(state.kNumBlocks, 1, 1);
        params.rootConstantBuffer = mKernelParams;
        NvFlexContextDispatch(mLib->mContext, &params);
    }
}

void Solver::CalculateAnisotropy(const IterationState &state) {
    if(mParams.anisotropyScale > 0.f) {
        NVFLEX_PROFILE_SECTION("CalculateAnisotropy", mTimerPool);
        auto sortedCellHash = mRadixSort->getBuffer();
        NvFlexDispatchParams params = {};
        params.shader = mLib->mShaderCalculateAnisotropy;
        params.readWrite[0] = NvFlexBufferGetResourceRW(mAnisotropy1);
        params.readWrite[1] = NvFlexBufferGetResourceRW(mAnisotropy2);
        params.readWrite[2] = NvFlexBufferGetResourceRW(mAnisotropy3);
        params.readOnly[0] = NvFlexBufferGetResource(mParticleNeighbors);
        params.readOnly[1] = NvFlexBufferGetResource(mParticleNeighborCounts);
        params.readOnly[2] = NvFlexBufferGetResource(sortedCellHash.val);
        params.readOnly[3] = NvFlexBufferGetResource(mSortedPhases);
        params.readOnly[4] = NvFlexBufferGetResource(mSortedPositions);
        params.gridDim = make_dim(state.kNumBlocks, 1, 1);
        params.rootConstantBuffer = mKernelParams;
        NvFlexContextDispatch(mLib->mContext, &params);
    }
}

void Solver::LazyClearGrid(const IterationState &state) {
    NVFLEX_PROFILE_SECTION("LazyClearGrid", mTimerPool);
    NvFlexDispatchParams params = {};
    params.shader = mLib->mShaderClearCellBuckets;
    params.readWrite[0] = NvFlexBufferGetResourceRW(mLib->mCellBucketStarts);
    params.readWrite[1] = NvFlexBufferGetResourceRW(mLib->mCellBucketEnds);
    params.readOnly[0] = NvFlexBufferGetResource(mActiveIndices);
    params.readOnly[1] = NvFlexBufferGetResource(mNewPositions);
    params.readOnly[2] = NvFlexBufferGetResource(mParticleBounds);
    params.gridDim = make_dim(state.kNumBlocks, 1, 1);
    params.rootConstantBuffer = mKernelParams;
    NvFlexContextDispatch(mLib->mContext, &params);
}

void Solver::ExecuteCallback(NvFlexSolverCallbackStage stage, float dt) {
    if (stage >= 0 && stage < eNvFlexStageCount && mCallbacks[stage].function) {
        auto sortedCellHash = mRadixSort->getBuffer();
        auto cb = &mCallbacks[stage];
        NvFlexSolverCallbackParams params = {};
        params.particles = reinterpret_cast<float *>(
            static_cast<NvFlexBuffer *>(mSortedNewPositions));
        params.velocities = reinterpret_cast<float *>(
            static_cast<NvFlexBuffer *>(mSortedNewVelocities));
        params.phases =
            reinterpret_cast<int *>(static_cast<NvFlexBuffer *>(mSortedPhases));
        params.numActive = mNumParticles;
        params.sortedToOriginalMap = reinterpret_cast<const int *>(sortedCellHash.key);
        params.originalToSortedMap = reinterpret_cast<const int *>(sortedCellHash.val);
        params.userData = cb->userData;
        params.dt = dt;
        cb->function(params);
    }
}

void Solver::CopyBufferImpl(NvFlexBuffer *dstBuffer, NvFlexBuffer *srcBuffer,
                            const NvFlexCopyDesc *copyDesc) {

    if(!srcBuffer || !dstBuffer)
        return;

    NvFlexBufferDesc srcDesc, dstDesc;
    NvFlexBufferGetDesc(srcBuffer, &srcDesc);
    NvFlexBufferGetDesc(dstBuffer, &dstDesc);

    NvFlexUint srcElemOffset, dstElemOffset, elemCount;

    if(copyDesc) {
        srcElemOffset = copyDesc->srcOffset;
        dstElemOffset = copyDesc->dstOffset;
        elemCount = copyDesc->elementCount;
    } else {
        srcElemOffset = 0;
        dstElemOffset = 0;
        elemCount = min(srcDesc.dim, dstDesc.dim);
    }

    auto srcStride =
        srcDesc.structStride ? srcDesc.structStride : getFormatSizeInBytes(srcDesc.format);
    auto dstStride =
        dstDesc.structStride ? dstDesc.structStride : getFormatSizeInBytes(dstDesc.format);

    if (srcStride == dstStride) {
        if(dstElemOffset + elemCount <= dstDesc.dim) {
            if(srcElemOffset + elemCount <= srcDesc.dim) {
                NvFlexContextCopyBuffer(
                    mLib->mContext, dstBuffer, dstElemOffset * dstStride, srcBuffer,
                    srcElemOffset * srcStride, elemCount * srcStride);

            } else
                FLEX_LOG_ERROR(mLib,
                               "Call to an NvFlexSet*() method with source range outside "
                               "of source buffer");
        } else
            FLEX_LOG_ERROR(mLib,
                           "Call to an NvFlexSet*() method with destination range outside "
                           "of destination buffer");

    } else {
        FLEX_LOG_ERROR(mLib,
                       "Call to an NvFlexSet*() method with incompatible buffer strides");
    }
}
void Solver::CopyBufferImpl(NvFlexBuffer *dstBuffer, NvFlexUint dstElemOffset,
                            NvFlexBuffer *srcBuffer, NvFlexUint srcElemOffset,
                            NvFlexUint numElements) {
    NvFlexCopyDesc copyDesc = {};
    copyDesc.dstOffset = dstElemOffset;
    copyDesc.srcOffset = srcElemOffset;
    copyDesc.elementCount = numElements;
    CopyBufferImpl(dstBuffer, srcBuffer, &copyDesc);
}

void Solver::SpringGenerateIndices(NvFlexBuffer *indices, int numSprings) {
    if(numSprings) {
        NvFlexDispatchParams params = {};
        params.shader = mLib->mShaderSpringsGenerateIndices;
        params.readWrite[0] = NvFlexBufferGetResourceRW(indices);
        params.gridDim = make_dim(divCeil<256>(2 * numSprings), 1, 1);
        params.rootConstantBuffer = mHalfSpringConstantBuffer;
        NvFlexContextDispatch(mLib->mContext, &params);
    }
}

void Solver::SpringFindParticleRange(NvFlexBuffer *sortedIndices, int numSprings) {
    if(numSprings) {
        NvFlexDispatchParams params = {};
        params.shader = mLib->mShaderSpringsParticleRange;
        params.readOnly[0] = NvFlexBufferGetResource(sortedIndices);
        params.readWrite[0] = mHalfSpringParticleBegin;
        params.readWrite[1] = mHalfSpringParticleEnd;
        params.gridDim = make_dim(divCeil<256>(2 * numSprings), 1, 1);
        params.rootConstantBuffer = mHalfSpringConstantBuffer;
        NvFlexContextDispatch(mLib->mContext, &params);
    }
}

void Solver::SpringRecordData(NvFlexBuffer *sortedIndices, int numSprings) {
    if(numSprings) {
        NvFlexDispatchParams params = {};
        params.shader = mLib->mShaderSpringsReorder;
        params.readOnly[0] = NvFlexBufferGetResource(sortedIndices);
        params.readOnly[1] = mSpringIndices;
        params.readOnly[2] = mSpringLengths;
        params.readOnly[3] = mSpringStiffness;
        params.readWrite[0] = mHalfSpringOpposites;
        params.readWrite[1] = mHalfSpringLengths;
        params.readWrite[2] = mHalfSpringStiffness;
        params.gridDim = make_dim(divCeil<256>(2 * numSprings), 1, 1);
        params.rootConstantBuffer = mHalfSpringConstantBuffer;
        NvFlexContextDispatch(mLib->mContext, &params);
    }
}

}  // namespace NvFlex
