#ifndef ETHEREAL_PHYS_SRC_SOLVER_STAGE_TIMERS_H
#define ETHEREAL_PHYS_SRC_SOLVER_STAGE_TIMERS_H

// Module-private: StageTimerPool, the GPU timer queries of one solver, and how the device creates the
// IPhysTimers view of them (readback.h).
//
// It is FleX's TimerPool (FleX/src/TimerPool.cpp) on nvrhi timer queries: one query per stage a
// step runs (NVFLEX_PROFILE_SECTION, here SolverPipeline::StageScope) and one around the whole step (FleX's
// latency timer). What differs:
//
// - FleX waited for the GPU in every getter (NvFlexContextTimerGetResult maps the query results). Here the
//   queries of a step are read only once pollTimerQuery says all of them finished, and the getters return
//   the newest step read so far. Steps are kept in flight in a queue, oldest first.
// - FleX created a context timer per section without limit (D3D queries). An nvrhi device has a fixed
//   number of timer queries, shared with the application; the pool holds at most a budget of them and
//   measures a step up to the point where they run out.
// - Timers are per stage name and step, as FleX's, but are not reset when nothing was measured: the getters
//   keep the last completed step.

#include <ethereal/phys/device.h>
#include <ethereal/phys/readback.h>

#include "kernel_library.h"

#include <cstdint>
#include <deque>
#include <vector>

namespace ephys::detail {

class StageTimerPool {
 public:
    static constexpr uint32_t c_DefaultBudget = 128;

    // device and host stay valid while the pool lives (the solver that owns it keeps the device).
    StageTimerPool(IPhysDevice* device, KernelLibraryHost* host);
    ~StageTimerPool();
    StageTimerPool(const StageTimerPool&) = delete;
    StageTimerPool& operator=(const StageTimerPool&) = delete;

    [[nodiscard]] bool isQueueSupported(nvrhi::CommandQueue queue) const;

    void setBudget(uint32_t maxQueries);
    [[nodiscard]] uint32_t budget() const { return m_Budget; }

    // The timers of the solver were switched on or off. Off releases the queries no pending step uses.
    void setEnabled(bool enabled);

    // ---- Recording (SolverPipeline::update and StageScope) ------------------------------------------------

    // Starts the record of a step on `commandList` (ownList: the device's own command list). Polls first,
    // which recycles the queries of finished steps. Records nothing when the queue of the list has no
    // timestamp queries.
    void beginStep(nvrhi::ICommandList* commandList, bool ownList);
    // Brackets a stage; beginStage returns the slot for endStage, or -1 when nothing was recorded.
    int32_t beginStage(nvrhi::ICommandList* commandList, const char* name);
    void endStage(nvrhi::ICommandList* commandList, int32_t slot);
    void endStep(nvrhi::ICommandList* commandList);

    // ---- Reading ------------------------------------------------------------------------------------------

    // Reads every pending step whose queries have all finished, oldest first, without waiting.
    uint64_t poll();

    [[nodiscard]] PhysTimerStatus status() const;
    [[nodiscard]] const PhysTimers& timers() const { return m_Timers; }
    [[nodiscard]] float latency() const { return m_Latency; }
    [[nodiscard]] const std::vector<DetailTimer>& details() const { return m_Details; }

 private:
    struct Stage {
        const char* name = nullptr;
        nvrhi::TimerQueryHandle query;
        bool ended = false;
    };

    struct Record {
        uint64_t serial = 0;             // among the measured steps (PhysTimerStatus::recordedSteps)
        uint64_t timedStep = 0;          // among all steps with the timers on, measured or skipped
        bool ownList = false;
        uint64_t ownListExecutions = 0;  // the device's count when the step was recorded
        float scale = 1.0f;              // the frequency correction of the queue (D3D12 compute)
        nvrhi::TimerQueryHandle latency;
        std::vector<Stage> stages;
        uint32_t wanted = 0;  // the queries the step asked for (one per stage and the latency)
        bool complete = true;
        bool nudged = false;  // D3D11: the immediate context was flushed for it
    };

    // A query from the free list, or a new one within the budget; null when there is none.
    nvrhi::TimerQueryHandle acquire();
    void recycle(nvrhi::TimerQueryHandle& query);
    void releaseIdle();

    [[nodiscard]] bool isSubmitted(const Record& record) const;
    [[nodiscard]] bool isFinished(const Record& record) const;
    void read(Record& record);
    void drop(Record& record);

    IPhysDevice* m_Device;
    nvrhi::IDevice* m_Rhi;
    KernelLibraryHost* m_Host;
    nvrhi::GraphicsAPI m_Api;

    bool m_QueueSupported[uint32_t(nvrhi::CommandQueue::Count)] = {};
    float m_QueueScale[uint32_t(nvrhi::CommandQueue::Count)] = {1.0f, 1.0f, 1.0f};

    bool m_Enabled = false;
    uint32_t m_Budget = c_DefaultBudget;
    uint32_t m_Allocated = 0;
    bool m_CreateFailed = false;  // the device refused a query: no more until the budget changes
    std::vector<nvrhi::TimerQueryHandle> m_Free;

    std::deque<Record> m_Pending;  // recorded and not read, oldest first
    Record* m_Recording = nullptr;  // the step being recorded (the back of m_Pending), or null
    uint64_t m_RecordedSteps = 0;
    uint64_t m_TimedSteps = 0;  // steps with the timers on, measured or skipped
    uint32_t m_SkippedSteps = 0;
    uint32_t m_QueriesLastStep = 0;
    uint32_t m_Need = 0;  // the queries the newest measured step asked for: what the next one is assumed to
                          // need; 0 before one
    bool m_LastSupported = false;

    // The newest completed step.
    uint64_t m_CompletedStep = 0;
    bool m_CompletedFull = false;
    PhysTimers m_Timers;
    float m_Latency = 0.0f;
    std::vector<DetailTimer> m_Details;
};

// The timer pool of a solver this library created (PBD or XPBD); null for any other IPhysSolver.
StageTimerPool* stageTimersOf(IPhysSolver* solver) noexcept;

// Creates an IPhysTimers view of `pool`, which belongs to `solver`. The view keeps the solver alive.
nvrhi::FRESULT createTimers(IPhysSolver* solver, StageTimerPool* pool, IPhysTimers** ppTimers) noexcept;

}  // namespace ephys::detail

#endif /* ETHEREAL_PHYS_SRC_SOLVER_STAGE_TIMERS_H */
