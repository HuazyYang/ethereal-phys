#include "Types.h"
#include "Context.h"
#include "NvFlexImpl.h"
#include "Library.h"
#include "Data.h"
#include "Solver.h"
#include "ClientHelper.h"
#include <cstdarg>
#include <algorithm>

void FlexLogError(NvFlexErrorCallback errFunc, NvFlexErrorSeverity level, const char* msg,
                  const char* file, int line, ...) {
    if (errFunc) {
        va_list ap;
        va_start(ap, line);
        char buff[2048];
        vsnprintf(buff, countof(buff), msg, ap);
        (*errFunc)(level, buff, file, line);
        va_end(ap);
    }
}

void NvResourceTracker::add(void* p, ResourceType t) {
    switch (t) {
        case eSolver:
            mSolver.push_back((NvFlexSolver*)p);
            break;
        case eTriangleMesh:
            mTriangleMesh.push_back((NvFlexUint)(uintptr_t)p);
            break;
        case eSDF:
            mSDF.push_back((NvFlexUint)(uintptr_t)p);
            break;
        case eConvexMesh:
            mConvexMesh.push_back((NvFlexUint)(uintptr_t)p);
            break;
    }
}

void NvResourceTracker::remove(void* p, ResourceType t) {
    switch (t) {
        case eSolver: {
            auto it = std::remove(mSolver.begin(), mSolver.end(), (NvFlexSolver*)p);
            mSolver.erase(it, mSolver.end());
        } break;
        case eTriangleMesh: {
            auto it = std::remove(mTriangleMesh.begin(), mTriangleMesh.end(),
                                  (NvFlexUint)(uintptr_t)p);
            mTriangleMesh.erase(it, mTriangleMesh.end());
        } break;
        case eSDF: {
            auto it = std::remove(mSDF.begin(), mSDF.end(), (NvFlexUint)(uintptr_t)p);
            mSDF.erase(it, mSDF.end());
        } break;
        case eConvexMesh: {
            auto it = std::remove(mConvexMesh.begin(), mConvexMesh.end(),
                                  (NvFlexUint)(uintptr_t)p);
            mConvexMesh.erase(it, mConvexMesh.end());
        } break;
    }
}

int NvResourceTracker::get(void* p, uint64_t n, ResourceType t) {
    switch (t) {
        case eSolver:
            n = std::min<uint64_t>(mSolver.size(), n);
            memcpy(p, mSolver.data(), sizeof(NvFlexSolver*) * n);
            return mSolver.size();
        case eTriangleMesh:
            n = std::min<uint64_t>(mTriangleMesh.size(), n);
            memcpy(p, mTriangleMesh.data(), sizeof(NvFlexUint) * n);
            return mTriangleMesh.size();
        case eSDF:
            n = std::min<uint64_t>(mSDF.size(), n);
            memcpy(p, mSDF.data(), sizeof(NvFlexUint) * n);
            return mSDF.size();
        case eConvexMesh:
            n = std::min<uint64_t>(mConvexMesh.size(), n);
            memcpy(p, mConvexMesh.data(), sizeof(NvFlexUint) * n);
            return mConvexMesh.size();
        default:
            return ~0;
    }
}

void NvResourceTracker::cleanup(NvFlexLibrary* lib) {
    while (!mSolver.empty()) {
        NvFlexDestroySolver(mSolver[0]);
    }

    while (!mTriangleMesh.empty()) {
        NvFlexDestroyTriangleMesh(lib, mTriangleMesh[0]);
    }

    while (!mSDF.empty()) {
        NvFlexDestroyDistanceField(lib, mSDF[0]);
    }

    while (!mConvexMesh.empty()) {
        NvFlexDestroyConvexMesh(lib, mConvexMesh[0]);
    }
}

using namespace NvFlex;

NV_FLEX_API NvFlexLibrary* NvFlexInit(int version, NvFlexErrorCallback errorFunc,
                                     NvFlexInitDesc* desc) {
    auto lib = new Library();
    if(!lib->Init(desc, errorFunc)) {
        lib->release();
        lib = nullptr;
    }
    return lib;
}

NV_FLEX_API void NvFlexShutdown(NvFlexLibrary* lib) {
    if(lib)
        lib->release();
}

NV_FLEX_API int NvFlexGetVersion() {
    return NV_FLEX_VERSION;
}

NV_FLEX_API void NvFlexSetSolverDescDefaults(NvFlexSolverDesc* desc) {
    desc->featureMode = eNvFlexFeatureModeDefault;
    desc->maxParticles = 0x4000;
    desc->maxNeighborsPerParticle = 96;
    desc->maxDiffuseParticles = 0;
    desc->maxContactsPerParticle = 6;
}

NV_FLEX_API NvFlexSolver* NvFlexCreateSolver(NvFlexLibrary* lib,
                                            const NvFlexSolverDesc* desc) {
    auto solver = new Solver(lib, desc);
    if(!solver->Init()) {
        solver->release();
        solver = nullptr;
    }
    return solver;
}

NV_FLEX_API void NvFlexDestroySolver(NvFlexSolver* solver) {
    if(solver)
        solver->release();
}

NV_FLEX_API int NvFlexGetSolvers(NvFlexLibrary* lib, NvFlexSolver** solvers, int n) {
    auto lib2 = implCast<Library>(lib);
    return lib2->GetResourceTracker()->get(solvers, n, NvResourceTracker::eSolver);
}

NV_FLEX_API NvFlexLibrary* NvFlexGetSolverLibrary(NvFlexSolver* solver) {
    auto s = implCast<Solver>(solver);
    if(s)
        return s->GetLibrary();
    return nullptr;
}

NV_FLEX_API void NvFlexGetSolverDesc(NvFlexSolver* solver, NvFlexSolverDesc* desc) {
    auto s = implCast<Solver>(solver);
    if(s && desc) {
        memcpy(desc, s->GetDesc(), sizeof(NvFlexSolverDesc));
    }
}

NV_FLEX_API NvFlexSolverCallback NvFlexRegisterSolverCallback(
    NvFlexSolver* solver, NvFlexSolverCallback function, NvFlexSolverCallbackStage stage) {
    auto s = implCast<Solver>(solver);
    if(s)
        return s->RegisterCallback(stage, &function);
    return {};
}

NV_FLEX_API void NvFlexUpdateSolver(NvFlexSolver* solver, float dt, int substeps,
                                    bool enableTimers) {
    auto s = implCast<Solver>(solver);
    if(s)
        s->Update(dt, substeps, enableTimers);
}

NV_FLEX_API void NvFlexSetParams(NvFlexSolver* solver, const NvFlexParams* params) {
    auto s = implCast<Solver>(solver);
    if(s)
        s->SetParams(params);
}

NV_FLEX_API void NvFlexGetParams(NvFlexSolver* solver, NvFlexParams* params) {
    auto s = implCast<Solver>(solver);
    if(s && params)
        memcpy(params, &s->GetParams(), sizeof(NvFlexParams));
}

NV_FLEX_API void NvFlexSetActive(NvFlexSolver* solver, NvFlexBuffer* indices,
                                 const NvFlexCopyDesc* desc) {
    auto s = implCast<Solver>(solver);
    if(s)
        s->SetActive(indices, desc);
}

NV_FLEX_API void NvFlexGetActive(NvFlexSolver* solver, NvFlexBuffer* indices,
                                 const NvFlexCopyDesc* desc) {
    auto s = implCast<Solver>(solver);
    if(s)
        s->GetActive(indices, desc);
}

NV_FLEX_API void NvFlexSetActiveCount(NvFlexSolver* solver, int n) {
    auto s = implCast<Solver>(solver);
    if (s)
        s->SetActiveCount(n);
}

NV_FLEX_API int NvFlexGetActiveCount(NvFlexSolver* solver) {
    auto s = implCast<Solver>(solver);
    if(s)
        return s->GetActiveCount();
    return 0;
}

NV_FLEX_API void NvFlexSetParticles(NvFlexSolver* solver, NvFlexBuffer* p,
                                    const NvFlexCopyDesc* desc) {
    auto s = implCast<Solver>(solver);
    if(s)
        s->SetParticles(p, desc);
}

NV_FLEX_API void NvFlexGetParticles(NvFlexSolver* solver, NvFlexBuffer* p,
                                    const NvFlexCopyDesc* desc) {
    auto s = implCast<Solver>(solver);
    if(s)
        s->GetParticles(p, desc);
}

NV_FLEX_API void NvFlexSetRestParticles(NvFlexSolver* solver, NvFlexBuffer* p,
                                        const NvFlexCopyDesc* desc) {
    auto s = implCast<Solver>(solver);
    if (s)
        s->SetRestParticles(p, desc);
}

NV_FLEX_API void NvFlexGetRestParticles(NvFlexSolver* solver, NvFlexBuffer* p,
                                        const NvFlexCopyDesc* desc) {
    auto s = implCast<Solver>(solver);
    if (s)
        s->GetRestParticles(p, desc);
}

NV_FLEX_API void NvFlexGetSmoothParticles(NvFlexSolver* solver, NvFlexBuffer* p,
                                          const NvFlexCopyDesc* desc) {
    auto s = implCast<Solver>(solver);
    if (s)
        s->GetSmoothParticles(p, desc);
}

NV_FLEX_API void NvFlexSetVelocities(NvFlexSolver* solver, NvFlexBuffer* v,
                                     const NvFlexCopyDesc* desc) {
    auto s = implCast<Solver>(solver);
    if (s)
        s->SetVelocities(v, desc);
}

NV_FLEX_API void NvFlexGetVelocities(NvFlexSolver* solver, NvFlexBuffer* v,
                                     const NvFlexCopyDesc* desc) {
    auto s = implCast<Solver>(solver);
    if (s)
        s->GetVelocities(v, desc);
}

NV_FLEX_API void NvFlexSetPhases(NvFlexSolver* solver, NvFlexBuffer* phases,
                                 const NvFlexCopyDesc* desc) {
    auto s = implCast<Solver>(solver);
    if (s)
        s->SetPhases(phases, desc);
}

NV_FLEX_API void NvFlexGetPhases(NvFlexSolver* solver, NvFlexBuffer* phases,
                                 const NvFlexCopyDesc* desc) {
    auto s = implCast<Solver>(solver);
    if (s)
        s->GetPhases(phases, desc);
}

NV_FLEX_API void NvFlexSetNormals(NvFlexSolver* solver, NvFlexBuffer* normals,
                                  const NvFlexCopyDesc* desc) {
    auto s = implCast<Solver>(solver);
    if (s)
        s->SetNormals(normals, desc);
}

NV_FLEX_API void NvFlexGetNormals(NvFlexSolver* solver, NvFlexBuffer* normals,
                                  const NvFlexCopyDesc* desc) {
    auto s = implCast<Solver>(solver);
    if (s)
        s->GetNormals(normals, desc);
}

NV_FLEX_API void NvFlexSetSprings(NvFlexSolver* solver, NvFlexBuffer* indices,
                                  NvFlexBuffer* restLengths, NvFlexBuffer* stiffness,
                                  int numSprings) {
    auto s = implCast<Solver>(solver);
    if (s)
        s->SetSprings(indices, restLengths, stiffness, numSprings);
}

NV_FLEX_API void NvFlexGetSprings(NvFlexSolver* solver, NvFlexBuffer* indices,
                                  NvFlexBuffer* restLengths, NvFlexBuffer* stiffness,
                                  int numSprings) {
    NVFLEX_ASSERT(0);
}

NV_FLEX_API void NvFlexSetRigids(NvFlexSolver* solver, NvFlexBuffer* offsets,
                                 NvFlexBuffer* indices, NvFlexBuffer* restPositions,
                                 NvFlexBuffer* restNormals, NvFlexBuffer* stiffness,
                                 NvFlexBuffer* thresholds, NvFlexBuffer* creeps,
                                 NvFlexBuffer* rotations, NvFlexBuffer* translations,
                                 int numRigids, int numIndices) {
    auto s = implCast<Solver>(solver);
    if(s)
        s->SetRigids(offsets, indices, restPositions, restNormals, stiffness, thresholds,
                      creeps, rotations, translations, numRigids, numIndices);
}

NV_FLEX_API void NvFlexGetRigids(NvFlexSolver* solver, NvFlexBuffer* offsets,
                                 NvFlexBuffer* indices, NvFlexBuffer* restPositions,
                                 NvFlexBuffer* restNormals, NvFlexBuffer* stiffness,
                                 NvFlexBuffer* thresholds, NvFlexBuffer* creeps,
                                 NvFlexBuffer* rotations, NvFlexBuffer* translations) {
    auto s = implCast<Solver>(solver);
    if(s)
        s->GetRigids(offsets, indices, restPositions, restNormals, stiffness, thresholds,
                      creeps, rotations, translations);
}

NV_FLEX_API NvFlexTriangleMeshId NvFlexCreateTriangleMesh(NvFlexLibrary* lib) {
    auto l = implCast<Library>(lib);
    if(l)
        return l->CreateTriangleMesh();
    return 0;
}

NV_FLEX_API void NvFlexDestroyTriangleMesh(NvFlexLibrary* lib, NvFlexTriangleMeshId mesh) {
    auto l = implCast<Library>(lib);
    if(l)
        return l->ReleaseTriangleMesh(mesh);
}

NV_FLEX_API int NvFlexGetTriangleMeshes(NvFlexLibrary* lib, NvFlexTriangleMeshId* meshes,
                                        int n) {
    auto l = implCast<Library>(lib);
    if(l)
        return l->GetTriangleMeshes(meshes, n);
    return 0;
}

NV_FLEX_API void NvFlexUpdateTriangleMesh(NvFlexLibrary* lib, NvFlexTriangleMeshId mesh,
                                          NvFlexBuffer* vertices, NvFlexBuffer* indices,
                                          int numVertices, int numTriangles,
                                          const float* lower, const float* upper) {
    auto l = implCast<Library>(lib);
    if(l)
        return l->UpdateTriangleMesh(mesh, vertices, indices, numVertices, numTriangles,
                                     (const NvFlexFloat3*)lower,
                                     (const NvFlexFloat3*)upper);
}

NV_FLEX_API void NvFlexGetTriangleMeshBounds(NvFlexLibrary* lib,
                                             const NvFlexTriangleMeshId mesh, float* lower,
                                             float* upper) {
    auto l = implCast<Library>(lib);
    if(l)
        return l->GetTriangleMeshBounds(mesh, (NvFlexFloat3 *)lower, (NvFlexFloat3 *)upper);
}

NV_FLEX_API NvFlexDistanceFieldId NvFlexCreateDistanceField(NvFlexLibrary* lib) {
    auto l = implCast<Library>(lib);
    if(l)
        return l->CreateDistanceField();
    return 0;
}

NV_FLEX_API void NvFlexDestroyDistanceField(NvFlexLibrary* lib, NvFlexDistanceFieldId sdf) {
    auto l = implCast<Library>(lib);
    if(l)
        return l->ReleaseDistanceField(sdf);
}

NV_FLEX_API int NvFlexGetDistanceFields(NvFlexLibrary* lib, NvFlexDistanceFieldId* sdfs,
                                        int n) {
    auto l = implCast<Library>(lib);
    if(l)
        return l->GetDistanceFields(sdfs, n);
    return 0;
}

NV_FLEX_API void NvFlexUpdateDistanceField(NvFlexLibrary* lib, NvFlexDistanceFieldId sdf,
                                           int dimx, int dimy, int dimz,
                                           NvFlexBuffer* field) {
    NVFLEX_NOT_IMPLEMENTED_ERROR();
}

NV_FLEX_API void NvFlexUpdateDistanceField2(NvFlexLibrary* lib, NvFlexDistanceFieldId sdf,
                                            NvFlexTexture3D* field) {
    auto l = implCast<Library>(lib);
    if(l)
        return l->UpdateDistanceField(sdf, field);
}

NV_FLEX_API NvFlexConvexMeshId NvFlexCreateConvexMesh(NvFlexLibrary* lib) {
    auto l = implCast<Library>(lib);
    if(l)
        return l->CreateConvexMesh();
    return 0;
}

NV_FLEX_API void NvFlexDestroyConvexMesh(NvFlexLibrary* lib, NvFlexConvexMeshId convex) {
    auto l = implCast<Library>(lib);
    if(l)
        return l->ReleaseConvexMesh(convex);
}

NV_FLEX_API int NvFlexGetConvexMeshes(NvFlexLibrary* lib, NvFlexConvexMeshId* meshes,
                                      int n) {
    auto l = implCast<Library>(lib);
    if(l)
        return l->GetConvexMeshes(meshes, n);
    return 0;
}

NV_FLEX_API void NvFlexUpdateConvexMesh(NvFlexLibrary* lib, NvFlexConvexMeshId convex,
                                        NvFlexBuffer* planes, int numPlanes,
                                        const float* lower, const float* upper) {
    auto l = implCast<Library>(lib);
    if(l)
        return l->UpdateConvexMesh(convex, planes, numPlanes, (const NvFlexFloat3*)lower,
                                   (NvFlexFloat3*)upper);
}

NV_FLEX_API void NvFlexGetConvexMeshBounds(NvFlexLibrary* lib, NvFlexConvexMeshId mesh,
                                           float* lower, float* upper) {
    auto l = implCast<Library>(lib);
    if(l)
        l->GetConvexMeshBounds(mesh, (NvFlexFloat3*)lower, (NvFlexFloat3*)upper);
}

NV_FLEX_API void NvFlexSetShapes(NvFlexSolver* solver, NvFlexBuffer* geometry,
                                 NvFlexBuffer* shapePositions, NvFlexBuffer* shapeRotations,
                                 NvFlexBuffer* shapePrevPositions,
                                 NvFlexBuffer* shapePrevRotations, NvFlexBuffer* shapeFlags,
                                 int numShapes) {
    auto s = implCast<Solver>(solver);
    if(s)
        s->SetShapes(geometry, shapePositions, shapeRotations, shapePrevPositions,
                     shapePrevRotations, shapeFlags, numShapes);
}

NV_FLEX_API void NvFlexSetDynamicTriangles(NvFlexSolver* solver, NvFlexBuffer* indices,
                                           NvFlexBuffer* normals, int numTris) {
    auto s = implCast<Solver>(solver);
    if(s)
        s->SetDynamicTriangles(indices, normals, numTris);
}

NV_FLEX_API void NvFlexGetDynamicTriangles(NvFlexSolver* solver, NvFlexBuffer* indices,
                                           NvFlexBuffer* normals, int numTris) {
    auto s = implSafeCast<Solver>(solver);
    if(s)
        s->GetDynamicTriangles(indices, normals, numTris);
}

NV_FLEX_API void NvFlexSetInflatables(NvFlexSolver* solver, NvFlexBuffer* startTris,
                                      NvFlexBuffer* numTris, NvFlexBuffer* restVolumes,
                                      NvFlexBuffer* overPressures,
                                      NvFlexBuffer* constraintScales, int numInflatables) {
    auto s = implCast<Solver>(solver);
    if(s)
        s->SetInflatables(startTris, numTris, restVolumes, overPressures, constraintScales,
                          numInflatables);
}

NV_FLEX_API void NvFlexGetDensities(NvFlexSolver* solver, NvFlexBuffer* densities,
                                    const NvFlexCopyDesc* desc) {
    auto s = implCast<Solver>(solver);
    if(s)
        s->GetDensities(densities, desc);
}

NV_FLEX_API void NvFlexGetAnisotropy(NvFlexSolver* solver, NvFlexBuffer* q1,
                                     NvFlexBuffer* q2, NvFlexBuffer* q3,
                                     const NvFlexCopyDesc* desc) {
    auto s = implCast<Solver>(solver);
    if(s)
        return s->GetAnisotropy(q1, q2, q3, desc);
}

NV_FLEX_API void NvFlexGetDiffuseParticles(NvFlexSolver* solver, NvFlexBuffer* p,
                                           NvFlexBuffer* v, NvFlexBuffer* count) {
    auto s = implCast<Solver>(solver);
    if(s)
        return s->GetDiffuseParticles(p, v, count);
}

NV_FLEX_API void NvFlexSetDiffuseParticles(NvFlexSolver* solver, NvFlexBuffer* p,
                                           NvFlexBuffer* v, int n) {
    auto s = implCast<Solver>(solver);
    if (s)
        return s->SetDiffuseParticles(p, v, n);
}

NV_FLEX_API void NvFlexGetContacts(NvFlexSolver* solver, NvFlexBuffer* planes,
                                   NvFlexBuffer* velocities, NvFlexBuffer* indices,
                                   NvFlexBuffer* counts) {
    auto s = implCast<Solver>(solver);
    if(s)
        s->GetContacts(planes, velocities, indices, counts);
}

NV_FLEX_API void NvFlexGetNeighbors(NvFlexSolver* solver, NvFlexBuffer* neighbors,
                                    NvFlexBuffer* counts, NvFlexBuffer* apiToInternal,
                                    NvFlexBuffer* internalToApi) {
    auto s = implCast<Solver>(solver);
    if(s)
        s->GetNeighbors(neighbors, counts, apiToInternal, internalToApi);
}

NV_FLEX_API void NvFlexGetBounds(NvFlexSolver* solver, NvFlexBuffer* lower,
                                 NvFlexBuffer* upper) {
    auto s = implCast<Solver>(solver);
    if(s)
        s->GetBounds(lower, upper);
}

NV_FLEX_API float NvFlexGetDeviceLatency(NvFlexSolver* solver, unsigned long long* begin,
                                         unsigned long long* end,
                                         unsigned long long* frequency) {
    auto s = implCast<Solver>(solver);
    if(s)
        return s->GetDeviceLatency(begin, end, frequency);
    return 0.f;
}

NV_FLEX_API void NvFlexGetTimers(NvFlexSolver* solver, NvFlexTimers* timers) {
    auto s = implCast<Solver>(solver);
    if(s)
        return s->GetTimers(timers);
}

NV_FLEX_API int NvFlexGetDetailTimers(NvFlexSolver* solver, NvFlexDetailTimer** timers) {
    auto s = implCast<Solver>(solver);
    if(s)
        return s->GetDetailTimers(timers);
    return 0;
}

NV_FLEX_API NvFlexBuffer* NvFlexAllocBuffer(NvFlexLibrary* lib, int elementCount,
                                            int elementByteStride, NvFlexBufferType type) {
    auto l = implCast<Library>(lib);
    if(l) {
        NVFLEX_ASSERT(type == eNvFlexBufferHost);
        NvFlexBufferDesc bufDesc = {};
        bufDesc.memType = type == eNvFlexBufferHost ? eNvFlexMemoryType_readback
                                                    : eNvFlexMemoryType_default;
        bufDesc.dim = elementCount;
        bufDesc.structStride = elementByteStride;
        bufDesc.cpuAccessMode = eNvFlexCpuAccessMode_readwrite;
        return NvFlexCreateBuffer(l->GetContext(), &bufDesc);
    }
    return nullptr;
}

NV_FLEX_API void NvFlexFreeBuffer(NvFlexBuffer* buf) {
    return NvFlexReleaseBuffer(buf);
}

NV_FLEX_API void* NvFlexMap(NvFlexBuffer* bufferIn, int flags) {
    auto buffer = implCast<Buffer>(bufferIn);
    return implCast<Context>(buffer->getContext())
        ->map(buffer, eNvFlexStagingCpuAccess_readwrite,
              (flags & eNvFlexMapDoNotWait) != 0);
}

NV_FLEX_API void NvFlexUnmap(NvFlexBuffer* bufferIn) {
    auto buffer = implCast<Buffer>(bufferIn);
    return implCast<Context>(buffer->getContext())->unmap(buffer);
}

NV_FLEX_API NvFlexBuffer* NvFlexRegisterOGLBuffer(NvFlexLibrary* lib, int buf,
                                                  int elementCount, int elementByteStride) {
    NVFLEX_NOT_IMPLEMENTED_ERROR();
    return nullptr;
}

NV_FLEX_API void NvFlexUnregisterOGLBuffer(NvFlexBuffer* buf) {
    NVFLEX_NOT_IMPLEMENTED_ERROR();
}

NV_FLEX_API NvFlexBuffer* NvFlexRegisterD3DBuffer(NvFlexLibrary* lib, void* buffer,
                                                  int elementCount, int elementByteStride) {
    auto l = implCast<Library>(lib);
    Buffer* pBuffer = nullptr;
    if(l) {
        auto context = implCast<Context>(l->GetContext());
        NvFlexResult rc =
            context->registerD3DBuffer(buffer, elementCount, elementByteStride, &pBuffer);
        NVFLEX_ASSERT(rc == eNvFlexSuccess);
    }

    return pBuffer;
}

NV_FLEX_API void NvFlexUnregisterD3DBuffer(NvFlexBuffer* buf) {
    NvFlexReleaseBuffer(buf);
}

NV_FLEX_API void NvFlexAcquireContext(NvFlexLibrary* lib) {}

NV_FLEX_API void NvFlexRestoreContext(NvFlexLibrary* lib) {}

NV_FLEX_API const char* NvFlexGetDeviceName(NvFlexLibrary* lib) {
    return implCast<Library>(lib)->GetDeviceName();
}

NV_FLEX_API void NvFlexGetDeviceAndContext(NvFlexLibrary* lib, void** device,
                                           void** context) {
    auto l = implCast<Library>(lib);
    if(l) {
        if(device)
            *device = l->GetDevice();
        if(context)
            *context = l->GetContext();
    }
}

NV_FLEX_API void NvFlexFlush(NvFlexLibrary* lib) {
    NVFLEX_NOT_IMPLEMENTED_ERROR();
}

NV_FLEX_API void NvFlexWait(NvFlexLibrary* lib) {
    NVFLEX_NOT_IMPLEMENTED_ERROR();
}

NV_FLEX_API void NvFlexExecuteContext(NvFlexLibrary* lib) {
    auto l = implCast<Library>(lib);
    if(l)
        l->ExecuteContext();
}

NV_FLEX_API void NvFlexWaitContext(NvFlexLibrary* lib) {
    auto l = implCast<Library>(lib);
    if(l)
        l->WaitContext();
}

NV_FLEX_API void NvFlexResetContext(NvFlexLibrary* lib, bool waitForPrevious) {
    auto l = implCast<Library>(lib);
    if (l)
        l->ResetContext(waitForPrevious);
}

NV_FLEX_API void NvFlexSetDebug(NvFlexSolver* solver, bool enable) {
    NVFLEX_NOT_IMPLEMENTED_ERROR();
}

NV_FLEX_API void NvFlexGetShapeBVH(NvFlexSolver* solver, void* bvh) {
    NVFLEX_NOT_IMPLEMENTED_ERROR();
}

NV_FLEX_API void NvFlexCopySolver(NvFlexSolver* dst, NvFlexSolver* src) {
    NVFLEX_NOT_IMPLEMENTED_ERROR();
}

NV_FLEX_API void NvFlexCopyDeviceToHost(NvFlexSolver* solver, NvFlexBuffer* pDevice,
                                        void* pHost, int size, int stride) {
    NVFLEX_NOT_IMPLEMENTED_ERROR();
}

NV_FLEX_API void NvFlexComputeWaitForGraphics(NvFlexLibrary* lib) {
    NVFLEX_NOT_IMPLEMENTED_ERROR();
}

NV_FLEX_API void NvFlexGetDataAftermath(NvFlexLibrary* lib, void* pDataOut,
                                        void* pStatusOut) {
    NVFLEX_NOT_IMPLEMENTED_ERROR();
}
