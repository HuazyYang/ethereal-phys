// This code contains NVIDIA Confidential Information and is disclosed to you
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and
// any modifications thereto. Any use, reproduction, disclosure, or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA Corporation is strictly prohibited.
//
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2013-2020 NVIDIA Corporation. All rights reserved.
#include "core/core.h"
#include "core/maths.h"

#include <NvFlex.h>
#include <NvFlexExt.h>

#include "flexExt_dx_common.h"

using BYTE = unsigned char;
#include "flexExt.UpdateForceFields.hlsl.h"

struct NvFlexExtForceFieldCallback {};

struct NvFlexExtForceFieldCallbackREV : NvFlexExtForceFieldCallback {
    NvFlexExtForceFieldCallbackREV(NvFlexSolver* solver)
        : mSolver(solver) {
        // force fields
        mMaxForceFields = 0;
        mNumForceFields = 0;

        mForceFieldsGpu2 = NULL;

        NvFlexLibrary* lib = NvFlexGetSolverLibrary(solver);
        NvFlexGetDeviceAndContext(lib, nullptr, (void**)&mContext2);

        {
            // force field shader
            NvFlexComputeShaderDesc desc{};
            desc.cs = (void*)g_flexExt_UpdateForceFields;
            desc.cs_length = sizeof(g_flexExt_UpdateForceFields);
            desc.label = L"NvFlexExtForceFieldCallback";
            desc.NVAPI_Slot = 0;

            mShaderUpdateForceFields2 = NvFlexCreateComputeShader(mContext2, &desc);
        }

        {
            // constant buffer
            NvFlexConstantBufferDesc desc = {};
            desc.sizeInBytes = 4 * sizeof(int);
            desc.uploadAccess = true;

            mConstantBuffer2 = NvFlexCreateConstantBuffer(mContext2, &desc);
        }
    }

    ~NvFlexExtForceFieldCallbackREV() {
        // force fields
        NvFlexReleaseBuffer(mForceFieldsGpu2);
        NvFlexReleaseConstantBuffer(mConstantBuffer2);
        NvFlexReleaseComputeShader(mShaderUpdateForceFields2);
    }

    void applyForceFields(const NvFlexSolverCallbackParams& params) {
        // callbacks always have the correct CUDA device set so we can safely launch kernels
        // without acquiring

        if (params.numActive && mNumForceFields) {
            const unsigned int numThreadsPerBlock = 256;
            const unsigned int kNumBlocks =
                (params.numActive + numThreadsPerBlock - 1) / numThreadsPerBlock;

            NvFlexBuffer* particles = (NvFlexBuffer*)params.particles;
            NvFlexBuffer* velocities = (NvFlexBuffer*)params.velocities;

            // Init constant buffer
            {
                FlexExtConstParams constBuffer;

                constBuffer.kNumParticles = params.numActive;
                constBuffer.kNumForceFields = mNumForceFields;
                constBuffer.kDt = params.dt;

                auto vptr = NvFlexConstantBufferMap(mContext2, mConstantBuffer2);
                memcpy(vptr, &constBuffer, sizeof(constBuffer));
                NvFlexConstantBufferUnmap(mContext2, mConstantBuffer2);
            }

            {
                NvFlexDispatchParams params = {};
                params.shader = mShaderUpdateForceFields2;
                params.readWrite[0] = NvFlexBufferGetResourceRW(velocities);
                params.readOnly[0] = NvFlexBufferGetResource(particles);
                params.readOnly[1] = NvFlexBufferGetResource(mForceFieldsGpu2);
                params.gridDim = {kNumBlocks, 1, 1};
                params.rootConstantBuffer = mConstantBuffer2;

                NvFlexContextDispatch(mContext2, &params);
            }
        }
    }

    void registerToSolver(const NvFlexExtForceField* forceFields, int numForceFields) {
        // re-alloc if necessary
        if (numForceFields > mMaxForceFields) {
            NvFlexReleaseBuffer(mForceFieldsGpu2);

            NvFlexBufferDesc desc{};
            desc.dim = numForceFields;
            desc.structStride = sizeof(NvFlexExtForceField);

            mForceFieldsGpu2 = NvFlexCreateBuffer(mContext2, &desc);

            mMaxForceFields = numForceFields;
        }
        mNumForceFields = numForceFields;

        if (numForceFields > 0) {
            // upload staging buffer
            NvFlexContextUploadBuffer(mContext2, mForceFieldsGpu2, 0, forceFields,
                                      numForceFields * sizeof(NvFlexExtForceField));
        }

        NvFlexSolverCallback callback;
        callback.function = [](NvFlexSolverCallbackParams c) {
            static_cast<NvFlexExtForceFieldCallbackREV*>(c.userData)->applyForceFields(c);
        };
        callback.userData = this;

        // register a callback to calculate the forces at the end of the time-step
        NvFlexRegisterSolverCallback(mSolver, callback, eNvFlexStageUpdateEnd);
    }

    NvFlexBuffer* mForceFieldsGpu2;

    // DX Specific
    NvFlexComputeShader* mShaderUpdateForceFields2;
    NvFlexConstantBuffer* mConstantBuffer2;

    int mMaxForceFields;
    int mNumForceFields;

    // D3D device and context wrappers for the solver library
    NvFlexContext* mContext2;

    NvFlexSolver* mSolver;
};

#undef NV_FLEX_DRAW_MAX_READ_TEXTURES

#include "context/Context.h"
#include "context/Device.h"

struct NvFlexExtForceFieldCallbackORG : NvFlexExtForceFieldCallback {
    NvFlexExtForceFieldCallbackORG(NvFlexSolver* solver)
        : mSolver(solver) {
        // force fields
        mMaxForceFields = 0;
        mNumForceFields = 0;

        mForceFieldsGpu = NULL;

        mDevice = NULL;
        mContext = NULL;

        NvFlexLibrary* lib = NvFlexGetSolverLibrary(solver);
        NvFlexGetDeviceAndContext(lib, (void**)&mDevice, (void**)&mContext);

        {
            // force field shader
            NvFlex::ComputeShaderDesc desc{};
            desc.cs = (void*)g_flexExt_UpdateForceFields;
            desc.cs_length = sizeof(g_flexExt_UpdateForceFields);
            desc.label = L"NvFlexExtForceFieldCallback";
            desc.NvAPI_Slot = 0;

            mShaderUpdateForceFields = mContext->createComputeShader(&desc);
        }

        {
            // constant buffer
            NvFlex::ConstantBufferDesc desc;
            desc.stride = sizeof(int);
            desc.dim = 4;
            desc.uploadAccess = true;

            mConstantBuffer = mContext->createConstantBuffer(&desc);
        }
    }

    ~NvFlexExtForceFieldCallbackORG() {
        // force fields
        delete mForceFieldsGpu;
        delete mConstantBuffer;
        delete mShaderUpdateForceFields;
    }

    void applyForceFields(const NvFlexSolverCallbackParams& params) {
        // callbacks always have the correct CUDA device set so we can safely launch kernels
        // without acquiring

        if (params.numActive && mNumForceFields) {
            const unsigned int numThreadsPerBlock = 256;
            const unsigned int kNumBlocks =
                (params.numActive + numThreadsPerBlock - 1) / numThreadsPerBlock;

            NvFlex::Buffer* particles = (NvFlex::Buffer*)params.particles;
            NvFlex::Buffer* velocities = (NvFlex::Buffer*)params.velocities;

            // Init constant buffer
            {
                FlexExtConstParams constBuffer;

                constBuffer.kNumParticles = params.numActive;
                constBuffer.kNumForceFields = mNumForceFields;
                constBuffer.kDt = params.dt;

                memcpy(mContext->map(mConstantBuffer), &constBuffer,
                       sizeof(FlexExtConstParams));
                mContext->unmap(mConstantBuffer);
            }

            {
                NvFlex::DispatchParams params = {};
                params.shader = mShaderUpdateForceFields;
                params.readWrite[0] = velocities->getResourceRW();
                params.readOnly[0] = particles->getResource();
                params.readOnly[1] = mForceFieldsGpu->getResource();
                params.gridDim = {kNumBlocks, 1, 1};
                params.rootConstantBuffer = mConstantBuffer;

                mContext->dispatch(&params);
            }
        }
    }

    void registerToSolver(const NvFlexExtForceField* forceFields, int numForceFields) {
        // re-alloc if necessary
        if (numForceFields > mMaxForceFields) {
            delete mForceFieldsGpu;

            NvFlex::BufferDesc desc{};
            desc.dim = numForceFields;
            desc.stride = sizeof(NvFlexExtForceField);
            desc.bufferType =
                NvFlex::eBuffer | NvFlex::eUAV_SRV | NvFlex::eStructured | NvFlex::eStage;
            desc.format = NvFlexFormat::eNvFlexFormat_unknown;
            desc.data = NULL;

            mForceFieldsGpu = mContext->createBuffer(&desc);

            mMaxForceFields = numForceFields;
        }
        mNumForceFields = numForceFields;

        if (numForceFields > 0) {
            // update staging buffer
            void* dstPtr = mContext->map(mForceFieldsGpu, NvFlex::eMapWrite);
            memcpy(dstPtr, forceFields, numForceFields * sizeof(NvFlexExtForceField));
            mContext->unmap(mForceFieldsGpu);

            // upload to device buffer
            mContext->upload(mForceFieldsGpu, 0,
                             numForceFields * sizeof(NvFlexExtForceField));
        }

        NvFlexSolverCallback callback;
        callback.function = [](NvFlexSolverCallbackParams c) {
            static_cast<NvFlexExtForceFieldCallbackORG*>(c.userData)->applyForceFields(c);
        };
        callback.userData = this;

        // register a callback to calculate the forces at the end of the time-step
        NvFlexRegisterSolverCallback(mSolver, callback, eNvFlexStageUpdateEnd);
    }

    union {
        NvFlex::Buffer* mForceFieldsGpu;
        NvFlexBuffer* mForceFieldsGpu2;
    };

    // DX Specific
    union {
        NvFlex::ComputeShader* mShaderUpdateForceFields;
        NvFlexComputeShader* mShaderUpdateForceFields2;
    };
    union {
        NvFlex::ConstantBuffer* mConstantBuffer;
        NvFlexConstantBuffer* mConstantBuffer2;
    };

    int mMaxForceFields;
    int mNumForceFields;

    // D3D device and context wrappers for the solver library
    NvFlex::Device* mDevice;
    union {
        NvFlex::Context* mContext;
        NvFlexContext* mContext2;
    };

    NvFlexSolver* mSolver;
};

NvFlexExtForceFieldCallback* NvFlexExtCreateForceFieldCallback(NvFlexSolver* solver) {
    if(NvFlexRuntime::Get().GetBackend() == NvFlexRuntime::eBackendPublic)
        return new NvFlexExtForceFieldCallbackORG(solver);
    else
        return new NvFlexExtForceFieldCallbackREV(solver);
}

void NvFlexExtDestroyForceFieldCallback(NvFlexExtForceFieldCallback* callback) {
    if (NvFlexRuntime::Get().GetBackend() == NvFlexRuntime::eBackendPublic)
        delete static_cast<NvFlexExtForceFieldCallbackORG *>(callback);
    else
        delete static_cast < NvFlexExtForceFieldCallbackREV*>(callback);
}

void NvFlexExtSetForceFields(NvFlexExtForceFieldCallback* c,
                             const NvFlexExtForceField* forceFields, int numForceFields) {
    if (NvFlexRuntime::Get().GetBackend() == NvFlexRuntime::eBackendPublic)
        static_cast<NvFlexExtForceFieldCallbackORG*>(c)->registerToSolver(forceFields,
                                                                          numForceFields);
    else
        static_cast<NvFlexExtForceFieldCallbackREV*>(c)->registerToSolver(forceFields,
                                                                          numForceFields);
}
