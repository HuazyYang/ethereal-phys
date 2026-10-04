#ifndef TIMERPOOL_H
#define TIMERPOOL_H
#include "Types.h"
#include "Object.h"
#include "VectorCached.h"
#include <nvflex/NvFlexContextExt.h>
#include <map>

struct NvFlexDetailTimer;

namespace NvFlex {

struct TimerPool : Object {
    int begin();
    int begin(const wchar_t *name);
    void end(int index);
    void end(const wchar_t *name, int index);
    void clear();
    NvFlexResult get(int index, float *timeGPU, float *timeCPU, NvFlexUint64 *gpuStartStamp, NvFlexUint64 *gpuEndStamp, NvFlexUint64 *gpuFreq);
    NvFlexResult get(const wchar_t *name, float *timeGPU);

    void enable(bool enabled);

    void reserve(NvFlexUint sz);

    NvFlexUint getStatistics(NvFlexDetailTimer *timers, NvFlexUint numTimers);

    TimerPool(NvFlexContext *ctx);
    ~TimerPool();

private:
   NvFlexContext *m_context;
   VectorCached<NvFlexContextTimer *, 1> m_elems;
   NvFlexUint m_usedElemCount;

   bool m_enabled;
   std::multimap<const wchar_t *, int, bool (*)(const wchar_t *, const wchar_t *),
                 Allocator<std::pair<const wchar_t *const, int>>>
       m_nameToTimerMapW;
};

struct NamedTimer {
    NamedTimer(TimerPool *pool, const wchar_t *name);
    ~NamedTimer();

    NamedTimer(const NamedTimer &) = delete;
    NamedTimer &operator=(const NamedTimer &) = delete;

private:
   TimerPool *m_pool;
   const wchar_t *m_name;
   int m_index;
};

#define NVFLEX_PROFILE_SECTION(name, pool) NamedTimer _{pool, L##name};

}

#endif /* TIMERPOOL_H */
