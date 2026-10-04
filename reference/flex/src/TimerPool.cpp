#include "TimerPool.h"
#include "ClientHelper.h"
#include "nvToolsExt.h"
#include "NvFlex.h"

namespace NvFlex {

// WString comparison function for timer name to timer index map
static inline bool ltwstr(const wchar_t *s1, const wchar_t *s2) noexcept {
    return wcscmp(s1, s2) < 0;
}

TimerPool::TimerPool(NvFlexContext *ctx)
    : m_context{ctx},
      m_elems{},
      m_usedElemCount{},
      m_enabled{},
      m_nameToTimerMapW(ltwstr) {}

TimerPool::~TimerPool() {
    for(auto timer : m_elems)
        NvFlexReleaseContextTimer(timer);
}

int TimerPool::begin() {
    if(!m_enabled)
        return -1;

    int index = m_usedElemCount++;
    if (index >= m_elems.size())
        m_elems.resize(m_usedElemCount);

    auto timer = m_elems[index];
    if (!timer)
        m_elems[index] = timer = NvFlexCreateContextTimer(m_context);
    else
        NvFlexContextTimerEnd(m_context, timer);

    NvFlexContextTimerBegin(m_context, timer);

    return index;
}

void TimerPool::end(int index) {
    if(index != -1) {
        auto timer = m_elems[index];
        NvFlexContextTimerEnd(m_context, timer);
    }
}

void TimerPool::end(const wchar_t *name, int index) {
    if (index != -1) {
        nvtxRangePop();
        end(index);
    }
}

void TimerPool::clear() {
    m_usedElemCount = 0;
    m_nameToTimerMapW.clear();
    for(auto timer : m_elems)
        NvFlexContextTimerEnd(m_context, timer);
}

NvFlexResult TimerPool::get(int index, float *timeGPU, float *timeCPU,
                            NvFlexUint64 *gpuStartStamp, NvFlexUint64 *gpuEndStamp,
                            NvFlexUint64 *gpuFreq) {
    float time_gpu = 0.f;
    NvFlexResult ret = eNvFlexFail;
    if(index != -1) {
        auto timer = m_elems[index];
        ret = NvFlexContextTimerGetResult(m_context, timer, timeGPU, timeCPU, gpuStartStamp,
                                    gpuEndStamp, gpuFreq);
    }

    return ret;
}

NvFlexResult TimerPool::get(const wchar_t *name, float *timeGPU) {
    NvFlexResult ret = eNvFlexSuccess, rc;
    float retval = 0.f;

    auto range = m_nameToTimerMapW.equal_range(name);
    for (auto it = range.first; it != range.second; ++it) {
        float timeGPU = 0.f;
        rc = get(it->second, &timeGPU, nullptr, nullptr, nullptr, nullptr);
        if(rc != eNvFlexSuccess)
            ret = rc;
        retval += timeGPU;
    }

    if(timeGPU)
        *timeGPU = retval;
    return ret;
}

int TimerPool::begin(const wchar_t *name) {
    int index = -1;
    if(m_enabled) {
        nvtxRangePushW(name);
        index = begin();
        m_nameToTimerMapW.insert(m_nameToTimerMapW.end(), std::make_pair(name, index));
    }
    return index;
}

void TimerPool::enable(bool enabled) {
    m_enabled = enabled;
}

void TimerPool::reserve(NvFlexUint sz) {
    NvFlexUint oldSize = m_elems.size();
    if(oldSize < sz) {
        m_elems.resize(sz);
        for (NvFlexUint i = oldSize; i < sz; ++i)
            m_elems[i] = NvFlexCreateContextTimer(m_context);
    }
}

NvFlexUint TimerPool::getStatistics(NvFlexDetailTimer *timers, NvFlexUint numTimers) {
    if(m_enabled) {
        NvFlexUint nextIdx = 0;
        for (auto it = m_nameToTimerMapW.begin();
             it != m_nameToTimerMapW.end() && nextIdx < numTimers; ++nextIdx) {
            const wchar_t *label = it->first;
            auto labelLen = wcslen(label);
            float time = 0.f, currTime;
            auto &detailTimer = timers[nextIdx];

            for (; it != m_nameToTimerMapW.end(); ++it) {
                if(wcscmp(label, it->first) != 0)
                    break;

                get(it->second, &currTime, nullptr, nullptr, nullptr, nullptr);
                time += currTime;
            }

            int mbLen = WideCharToMultiByte(CP_ACP, 0, label, labelLen, nullptr, 0, nullptr, nullptr);
            mbLen = min(mbLen, 255);
            WideCharToMultiByte(CP_ACP, 0, label, labelLen, detailTimer.name, mbLen,
                                nullptr, nullptr);
            detailTimer.name[mbLen] = 0;
            detailTimer.time = time;
        }

        return nextIdx;
    }

    return 0;
}

NamedTimer::NamedTimer(TimerPool *pool, const wchar_t *name): m_pool(pool), m_name(name) {
    m_index = m_pool->begin(name);
}

NamedTimer::~NamedTimer() {
    m_pool->end(m_name, m_index);
}

}  // namespace NvFlex
