#pragma once

#include "NvFlex.h"
#include "nvflex/NvFlexContext.h"
#include "nvflex/NvFlexContextExt.h"

#include <string>

#define NVFLEX_RUNTIME_FUNCTIONS(X) \
    X(NvFlexInit)                   \
    X(NvFlexShutdown)               \
    X(NvFlexGetVersion)             \
    X(NvFlexSetSolverDescDefaults)  \
    X(NvFlexCreateSolver)           \
    X(NvFlexDestroySolver)          \
    X(NvFlexGetSolvers)             \
    X(NvFlexGetSolverLibrary)       \
    X(NvFlexGetSolverDesc)          \
    X(NvFlexRegisterSolverCallback) \
    X(NvFlexUpdateSolver)           \
    X(NvFlexSetParams)              \
    X(NvFlexGetParams)              \
    X(NvFlexSetActive)              \
    X(NvFlexGetActive)              \
    X(NvFlexSetActiveCount)         \
    X(NvFlexGetActiveCount)         \
    X(NvFlexSetParticles)           \
    X(NvFlexGetParticles)           \
    X(NvFlexSetRestParticles)       \
    X(NvFlexGetRestParticles)       \
    X(NvFlexGetSmoothParticles)     \
    X(NvFlexSetVelocities)          \
    X(NvFlexGetVelocities)          \
    X(NvFlexSetPhases)              \
    X(NvFlexGetPhases)              \
    X(NvFlexSetNormals)             \
    X(NvFlexGetNormals)             \
    X(NvFlexSetSprings)             \
    X(NvFlexGetSprings)             \
    X(NvFlexSetRigids)              \
    X(NvFlexGetRigids)              \
    X(NvFlexCreateTriangleMesh)     \
    X(NvFlexDestroyTriangleMesh)    \
    X(NvFlexGetTriangleMeshes)      \
    X(NvFlexUpdateTriangleMesh)     \
    X(NvFlexGetTriangleMeshBounds)  \
    X(NvFlexCreateDistanceField)    \
    X(NvFlexDestroyDistanceField)   \
    X(NvFlexGetDistanceFields)      \
    X(NvFlexUpdateDistanceField)    \
    X(NvFlexCreateConvexMesh)       \
    X(NvFlexDestroyConvexMesh)      \
    X(NvFlexGetConvexMeshes)        \
    X(NvFlexUpdateConvexMesh)       \
    X(NvFlexGetConvexMeshBounds)    \
    X(NvFlexSetShapes)              \
    X(NvFlexSetDynamicTriangles)    \
    X(NvFlexGetDynamicTriangles)    \
    X(NvFlexSetInflatables)         \
    X(NvFlexGetDensities)           \
    X(NvFlexGetAnisotropy)          \
    X(NvFlexGetDiffuseParticles)    \
    X(NvFlexSetDiffuseParticles)    \
    X(NvFlexGetContacts)            \
    X(NvFlexGetNeighbors)           \
    X(NvFlexGetBounds)              \
    X(NvFlexGetDeviceLatency)       \
    X(NvFlexGetTimers)              \
    X(NvFlexGetDetailTimers)        \
    X(NvFlexAllocBuffer)            \
    X(NvFlexFreeBuffer)             \
    X(NvFlexMap)                    \
    X(NvFlexUnmap)                  \
    X(NvFlexRegisterD3DBuffer)      \
    X(NvFlexUnregisterD3DBuffer)    \
    X(NvFlexAcquireContext)         \
    X(NvFlexRestoreContext)         \
    X(NvFlexGetDeviceName)          \
    X(NvFlexGetDeviceAndContext)    \
    X(NvFlexFlush)                  \
    X(NvFlexWait)                   \
    X(NvFlexComputeWaitForGraphics)

#define NVFLEX_REV_RUNTIME_FUNCTIONS(X) \
    X(NvFlexExecuteContext) \
    X(NvFlexWaitContext) \
    X(NvFlexResetContext) \
    X(NvFlexUpdateDistanceField2)

#define NVFLEX_REV_CONTEXT_FUNCTIONS(X) \
    X(NvFlexCreateTexture3D)            \
    X(NvFlexTexture3DMap)               \
    X(NvFlexTexture3DUnmap)             \
    X(NvFlexReleaseTexture3D)           \
    X(NvFlexCreateComputeShader)        \
    X(NvFlexReleaseBuffer)              \
    X(NvFlexReleaseConstantBuffer)      \
    X(NvFlexReleaseComputeShader)       \
    X(NvFlexCreateConstantBuffer)       \
    X(NvFlexConstantBufferMap)          \
    X(NvFlexConstantBufferUnmap)        \
    X(NvFlexBufferGetResourceRW)        \
    X(NvFlexBufferGetResource)          \
    X(NvFlexContextDispatch)            \
    X(NvFlexCreateBuffer)               \
    X(NvFlexContextUploadBuffer)

class __declspec(dllexport) NvFlexRuntime
{
public:
    enum Backend
    {
        eBackendPublic = 0,
        eBackendReversed = 1
    };

    static NvFlexRuntime &Get();

    bool Load(const std::wstring &path, Backend backend, std::string &error);
    bool Load(const std::wstring &path, std::string &error);
    void Unload();
    bool IsLoaded() const { return m_Module != nullptr; }
    Backend GetBackend() const { return m_Backend; }

#define DECLARE_NVFLEX_PROC(name) decltype(&::name) p_##name = nullptr;
    NVFLEX_RUNTIME_FUNCTIONS(DECLARE_NVFLEX_PROC)
    NVFLEX_REV_RUNTIME_FUNCTIONS(DECLARE_NVFLEX_PROC)
    NVFLEX_REV_CONTEXT_FUNCTIONS(DECLARE_NVFLEX_PROC)
#undef DECLARE_NVFLEX_PROC

private:
    void *m_Module = nullptr;
    Backend m_Backend = eBackendPublic;
};

#define NvFlexInit (*NvFlexRuntime::Get().p_NvFlexInit)
#define NvFlexShutdown (*NvFlexRuntime::Get().p_NvFlexShutdown)
#define NvFlexGetVersion (*NvFlexRuntime::Get().p_NvFlexGetVersion)
#define NvFlexSetSolverDescDefaults (*NvFlexRuntime::Get().p_NvFlexSetSolverDescDefaults)
#define NvFlexCreateSolver (*NvFlexRuntime::Get().p_NvFlexCreateSolver)
#define NvFlexDestroySolver (*NvFlexRuntime::Get().p_NvFlexDestroySolver)
#define NvFlexGetSolvers (*NvFlexRuntime::Get().p_NvFlexGetSolvers)
#define NvFlexGetSolverLibrary (*NvFlexRuntime::Get().p_NvFlexGetSolverLibrary)
#define NvFlexGetSolverDesc (*NvFlexRuntime::Get().p_NvFlexGetSolverDesc)
#define NvFlexRegisterSolverCallback (*NvFlexRuntime::Get().p_NvFlexRegisterSolverCallback)
#define NvFlexUpdateSolver (*NvFlexRuntime::Get().p_NvFlexUpdateSolver)
#define NvFlexSetParams (*NvFlexRuntime::Get().p_NvFlexSetParams)
#define NvFlexGetParams (*NvFlexRuntime::Get().p_NvFlexGetParams)
#define NvFlexSetActive (*NvFlexRuntime::Get().p_NvFlexSetActive)
#define NvFlexGetActive (*NvFlexRuntime::Get().p_NvFlexGetActive)
#define NvFlexSetActiveCount (*NvFlexRuntime::Get().p_NvFlexSetActiveCount)
#define NvFlexGetActiveCount (*NvFlexRuntime::Get().p_NvFlexGetActiveCount)
#define NvFlexSetParticles (*NvFlexRuntime::Get().p_NvFlexSetParticles)
#define NvFlexGetParticles (*NvFlexRuntime::Get().p_NvFlexGetParticles)
#define NvFlexSetRestParticles (*NvFlexRuntime::Get().p_NvFlexSetRestParticles)
#define NvFlexGetRestParticles (*NvFlexRuntime::Get().p_NvFlexGetRestParticles)
#define NvFlexGetSmoothParticles (*NvFlexRuntime::Get().p_NvFlexGetSmoothParticles)
#define NvFlexSetVelocities (*NvFlexRuntime::Get().p_NvFlexSetVelocities)
#define NvFlexGetVelocities (*NvFlexRuntime::Get().p_NvFlexGetVelocities)
#define NvFlexSetPhases (*NvFlexRuntime::Get().p_NvFlexSetPhases)
#define NvFlexGetPhases (*NvFlexRuntime::Get().p_NvFlexGetPhases)
#define NvFlexSetNormals (*NvFlexRuntime::Get().p_NvFlexSetNormals)
#define NvFlexGetNormals (*NvFlexRuntime::Get().p_NvFlexGetNormals)
#define NvFlexSetSprings (*NvFlexRuntime::Get().p_NvFlexSetSprings)
#define NvFlexGetSprings (*NvFlexRuntime::Get().p_NvFlexGetSprings)
#define NvFlexSetRigids (*NvFlexRuntime::Get().p_NvFlexSetRigids)
#define NvFlexGetRigids (*NvFlexRuntime::Get().p_NvFlexGetRigids)
#define NvFlexCreateTriangleMesh (*NvFlexRuntime::Get().p_NvFlexCreateTriangleMesh)
#define NvFlexDestroyTriangleMesh (*NvFlexRuntime::Get().p_NvFlexDestroyTriangleMesh)
#define NvFlexGetTriangleMeshes (*NvFlexRuntime::Get().p_NvFlexGetTriangleMeshes)
#define NvFlexUpdateTriangleMesh (*NvFlexRuntime::Get().p_NvFlexUpdateTriangleMesh)
#define NvFlexGetTriangleMeshBounds (*NvFlexRuntime::Get().p_NvFlexGetTriangleMeshBounds)
#define NvFlexCreateDistanceField (*NvFlexRuntime::Get().p_NvFlexCreateDistanceField)
#define NvFlexDestroyDistanceField (*NvFlexRuntime::Get().p_NvFlexDestroyDistanceField)
#define NvFlexGetDistanceFields (*NvFlexRuntime::Get().p_NvFlexGetDistanceFields)
#define NvFlexUpdateDistanceField (*NvFlexRuntime::Get().p_NvFlexUpdateDistanceField)
#define NvFlexCreateConvexMesh (*NvFlexRuntime::Get().p_NvFlexCreateConvexMesh)
#define NvFlexDestroyConvexMesh (*NvFlexRuntime::Get().p_NvFlexDestroyConvexMesh)
#define NvFlexGetConvexMeshes (*NvFlexRuntime::Get().p_NvFlexGetConvexMeshes)
#define NvFlexUpdateConvexMesh (*NvFlexRuntime::Get().p_NvFlexUpdateConvexMesh)
#define NvFlexGetConvexMeshBounds (*NvFlexRuntime::Get().p_NvFlexGetConvexMeshBounds)
#define NvFlexSetShapes (*NvFlexRuntime::Get().p_NvFlexSetShapes)
#define NvFlexSetDynamicTriangles (*NvFlexRuntime::Get().p_NvFlexSetDynamicTriangles)
#define NvFlexGetDynamicTriangles (*NvFlexRuntime::Get().p_NvFlexGetDynamicTriangles)
#define NvFlexSetInflatables (*NvFlexRuntime::Get().p_NvFlexSetInflatables)
#define NvFlexGetDensities (*NvFlexRuntime::Get().p_NvFlexGetDensities)
#define NvFlexGetAnisotropy (*NvFlexRuntime::Get().p_NvFlexGetAnisotropy)
#define NvFlexGetDiffuseParticles (*NvFlexRuntime::Get().p_NvFlexGetDiffuseParticles)
#define NvFlexSetDiffuseParticles (*NvFlexRuntime::Get().p_NvFlexSetDiffuseParticles)
#define NvFlexGetContacts (*NvFlexRuntime::Get().p_NvFlexGetContacts)
#define NvFlexGetNeighbors (*NvFlexRuntime::Get().p_NvFlexGetNeighbors)
#define NvFlexGetBounds (*NvFlexRuntime::Get().p_NvFlexGetBounds)
#define NvFlexGetDeviceLatency (*NvFlexRuntime::Get().p_NvFlexGetDeviceLatency)
#define NvFlexGetTimers (*NvFlexRuntime::Get().p_NvFlexGetTimers)
#define NvFlexGetDetailTimers (*NvFlexRuntime::Get().p_NvFlexGetDetailTimers)
#define NvFlexAllocBuffer (*NvFlexRuntime::Get().p_NvFlexAllocBuffer)
#define NvFlexFreeBuffer (*NvFlexRuntime::Get().p_NvFlexFreeBuffer)
#define NvFlexMap (*NvFlexRuntime::Get().p_NvFlexMap)
#define NvFlexUnmap (*NvFlexRuntime::Get().p_NvFlexUnmap)
#define NvFlexRegisterOGLBuffer (*NvFlexRuntime::Get().p_NvFlexRegisterOGLBuffer)
#define NvFlexUnregisterOGLBuffer (*NvFlexRuntime::Get().p_NvFlexUnregisterOGLBuffer)
#define NvFlexRegisterD3DBuffer (*NvFlexRuntime::Get().p_NvFlexRegisterD3DBuffer)
#define NvFlexUnregisterD3DBuffer (*NvFlexRuntime::Get().p_NvFlexUnregisterD3DBuffer)
#define NvFlexAcquireContext (*NvFlexRuntime::Get().p_NvFlexAcquireContext)
#define NvFlexRestoreContext (*NvFlexRuntime::Get().p_NvFlexRestoreContext)
#define NvFlexGetDeviceName (*NvFlexRuntime::Get().p_NvFlexGetDeviceName)
#define NvFlexGetDeviceAndContext (*NvFlexRuntime::Get().p_NvFlexGetDeviceAndContext)
#define NvFlexFlush (*NvFlexRuntime::Get().p_NvFlexFlush)
#define NvFlexWait (*NvFlexRuntime::Get().p_NvFlexWait)
#define NvFlexComputeWaitForGraphics (*NvFlexRuntime::Get().p_NvFlexComputeWaitForGraphics)

#define NvFlexExecuteContext (*NvFlexRuntime::Get().p_NvFlexExecuteContext)
#define NvFlexWaitContext (*NvFlexRuntime::Get().p_NvFlexWaitContext)
#define NvFlexResetContext (*NvFlexRuntime::Get().p_NvFlexResetContext)
#define NvFlexUpdateDistanceField2 (*NvFlexRuntime::Get().p_NvFlexUpdateDistanceField2)

#define NvFlexCreateTexture3D (*NvFlexRuntime::Get().p_NvFlexCreateTexture3D)
#define NvFlexTexture3DMap (*NvFlexRuntime::Get().p_NvFlexTexture3DMap)
#define NvFlexTexture3DUnmap (*NvFlexRuntime::Get().p_NvFlexTexture3DUnmap)
#define NvFlexReleaseTexture3D (*NvFlexRuntime::Get().p_NvFlexReleaseTexture3D)
#define NvFlexCreateComputeShader (*NvFlexRuntime::Get().p_NvFlexCreateComputeShader)
#define NvFlexReleaseBuffer (*NvFlexRuntime::Get().p_NvFlexReleaseBuffer)
#define NvFlexReleaseConstantBuffer (*NvFlexRuntime::Get().p_NvFlexReleaseConstantBuffer)
#define NvFlexReleaseComputeShader (*NvFlexRuntime::Get().p_NvFlexReleaseComputeShader)
#define NvFlexCreateConstantBuffer (*NvFlexRuntime::Get().p_NvFlexCreateConstantBuffer)
#define NvFlexConstantBufferMap (*NvFlexRuntime::Get().p_NvFlexConstantBufferMap)
#define NvFlexConstantBufferUnmap (*NvFlexRuntime::Get().p_NvFlexConstantBufferUnmap)
#define NvFlexBufferGetResourceRW (*NvFlexRuntime::Get().p_NvFlexBufferGetResourceRW)
#define NvFlexBufferGetResource (*NvFlexRuntime::Get().p_NvFlexBufferGetResource)
#define NvFlexContextDispatch (*NvFlexRuntime::Get().p_NvFlexContextDispatch)
#define NvFlexCreateBuffer (*NvFlexRuntime::Get().p_NvFlexCreateBuffer)
#define NvFlexContextUploadBuffer (*NvFlexRuntime::Get().p_NvFlexContextUploadBuffer)