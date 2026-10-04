#ifndef FLEX_H
#define FLEX_H
#include "NvFlex.h"
#include "Allocable.h"
#include "NvFlexObjectImpl.h"
#include "VectorCached.h"

struct NvFlexSolver;

struct NvResourceTracker : NvFlex::Allocable {
    NvResourceTracker() {}
    ~NvResourceTracker() {}

    enum ResourceType { eSolver = 0, eTriangleMesh = 1, eSDF = 2, eConvexMesh = 3 };
    NvFlex::VectorCached<NvFlexSolver *> mSolver;
    NvFlex::VectorCached<NvFlexUint> mTriangleMesh;
    NvFlex::VectorCached<NvFlexUint> mSDF;
    NvFlex::VectorCached<NvFlexUint> mConvexMesh;

    void add(void *p, ResourceType t);
    void remove(void *p, ResourceType t);
    int get(void *p, uint64_t n, ResourceType t);
    void cleanup(NvFlexLibrary *lib);
};

struct NvFlexLibrary : NvFlexObject {
    virtual NvFlexErrorCallback GetLogger() = 0;
};

struct NvFlexSolver : NvFlexObject {};

void FlexLogError(NvFlexErrorCallback errorFunc, NvFlexErrorSeverity level, const char* msg,
                 const char* file, int line, ...);

#define FLEX_LOG_ERROR(lib, msg, ...) \
    FlexLogError(lib->GetLogger(), eNvFlexLogError, msg, __FILE__, __LINE__, ##__VA_ARGS__)
#define FLEX_LOG_INFO(lib, msg, ...) \
    FlexLogError(lib->GetLogger(), eNvFlexLogInfo, msg, __FILE__, __LINE__, ##__VA_ARGS__)
#define FLEX_LOG_WARNING(lib, msg, ...) \
    FlexLogError(lib->GetLogger(), eNvFlexLogWarning, msg, __FILE__, __LINE__, ##__VA_ARGS__)
#define FLEX_LOG_DEBUG(lib, msg, ...) \
    FlexLogError(lib->GetLogger(), eNvFlexLogDebug, msg, __FILE__, __LINE__, ##__VA_ARGS__)

#define FLEX_IMPL_LOG_ERROR(logger, msg, ...) \
    FlexLogError(logger, eNvFlexLogError, msg, __FILE__, __LINE__, ##__VA_ARGS__)
#define FLEX_IMPL_LOG_INFO(logger, msg, ...) \
    FlexLogError(logger, eNvFlexLogInfo, msg, __FILE__, __LINE__, ##__VA_ARGS__)
#define FLEX_IMPL_LOG_WARNING(logger, msg, ...)                                        \
    FlexLogError(logger, eNvFlexLogWarning, msg, __FILE__, __LINE__, \
                 ##__VA_ARGS__)
#define FLEX_IMPL_LOG_DEBUG(logger, msg, ...) \
    FlexLogError(logger, eNvFlexLogDebug, msg, __FILE__, __LINE__, ##__VA_ARGS__)

#endif /* FLEX_H */
