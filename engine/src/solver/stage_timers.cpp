#include "stage_timers.h"

#include "phys_messages.h"

#include <nvrhi/core/foundation.h>

#include <algorithm>
#include <cstring>
#include <new>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <d3d11.h>
#include <d3d12.h>
#endif

// StageTimerPool (stage_timers.h) and the IPhysTimers view of it.
//
// What the native APIs are asked, and why (nvrhi has no query for any of them):
//
// - Vulkan: whether the graphics and the compute queue have timestamps. A queue family may have
//   timestampValidBits 0, and vkCmdWriteTimestamp is invalid there (nvrhi does not check).
// - D3D12: the timestamp frequency of the compute queue. nvrhi converts every timer query with the
//   frequency of the graphics queue (d3d12-queries.cpp), and a timestamp on the compute queue counts at that
//   queue's frequency; the two are usually the same, and the pool corrects by their ratio where they are not.
// - D3D11: the immediate context is flushed once for a step that is not finished when it is polled. nvrhi
//   polls with D3D11_ASYNC_GETDATA_DONOTFLUSH and its executeCommandLists submits nothing, so without a
//   flush (a Present, a wait) the queries of a headless application could stay in the driver's buffer.

namespace ephys::detail {

namespace {

// FleX's Solver::GetTimers table (FleX/src/Solver.cpp), in its order: which stage adds into
// which member of NvFlexTimers. total is the sum of these, in this order.
struct Category {
    const char* stage;
    float PhysTimers::*member;
};

const Category c_Categories[] = {
    {"Predicate", &PhysTimers::predict},
    {"CalculateParticleHash", &PhysTimers::createCellIndices},
    {"SortParticleHash", &PhysTimers::sortCellIndices},
    {"CreateGrid", &PhysTimers::createGrid},
    {"RecordParticles", &PhysTimers::reorder},
    {"CollideParticles", &PhysTimers::collideParticles},
    {"CollideShapes", &PhysTimers::collideShapes},
    {"CollideTriangles", &PhysTimers::collideTriangles},
    {"CollideFields", &PhysTimers::collideFields},
    {"CalculateDensity", &PhysTimers::calculateDensity},
    {"SolveDensity", &PhysTimers::solveDensities},
    {"UpdateVelocities", &PhysTimers::solveVelocities},
    {"CalculateVorticity", &PhysTimers::solveVelocities},
    {"SolveVelocities", &PhysTimers::solveVelocities},
    {"SolveShapes", &PhysTimers::solveShapes},
    {"SolveSprings", &PhysTimers::solveSprings},
    {"SolveContact", &PhysTimers::solveContacts},
    {"SolveInflatables", &PhysTimers::solveInflatables},
    {"ApplyDeltas", &PhysTimers::applyDeltas},
    {"SmoothPositions", &PhysTimers::calculateAnisotropy},
    {"CalculateAnisotropy", &PhysTimers::calculateAnisotropy},
    {"CreateDiffuseParticles", &PhysTimers::updateDiffuse},
    {"UpdateDiffuseParticles", &PhysTimers::updateDiffuse},
    {"ClampDiffuseParticleCount", &PhysTimers::updateDiffuse},
    {"CompactDiffuseParticles", &PhysTimers::updateDiffuse},
    {"UpdateTriangles", &PhysTimers::updateTriangles},
    {"UpdateNormals", &PhysTimers::updateNormals},
    {"NormalizeNormals", &PhysTimers::updateNormals},
    {"Finalize", &PhysTimers::finalize},
    {"ComputeBounds", &PhysTimers::updateBounds},
};

// A step that has not finished this many steps after it was recorded is taken to have been dropped (its
// command list never executed) and stops holding back the steps after it.
constexpr uint64_t c_AbandonAfterSteps = 64;

// Vulkan, declared here instead of including the Vulkan headers (the engine is not built against them):
// VkQueueFamilyProperties and the two entry points, whose layout and signatures are those of Vulkan 1.0 and
// frozen. Dispatchable handles are pointers.
struct VulkanQueueFamilyProperties {
    uint32_t queueFlags;
    uint32_t queueCount;
    uint32_t timestampValidBits;
    uint32_t minImageTransferGranularity[3];
};
static_assert(sizeof(VulkanQueueFamilyProperties) == 24);
constexpr uint32_t c_VulkanQueueGraphicsBit = 0x1;  // VK_QUEUE_GRAPHICS_BIT
constexpr uint32_t c_VulkanQueueComputeBit = 0x2;   // VK_QUEUE_COMPUTE_BIT
typedef void (*VulkanVoidFunction)();
typedef VulkanVoidFunction (*VulkanGetInstanceProcAddr)(void* instance, const char* name);
typedef void (*VulkanGetQueueFamilyProperties)(void* physicalDevice, uint32_t* count,
                                               VulkanQueueFamilyProperties* properties);

// Whether every queue family with `flag` (graphics or compute) writes timestamps (timestampValidBits above
// 0). nvrhi does not say which family it chose for a queue, so the answer is for all of them: a device where
// one compute family lacks timestamps counts as without them on the compute queue. Looked up in the loader
// the application loaded; false when that fails.
bool vulkanTimestamps(nvrhi::IDevice* device, uint32_t flag) {
#if defined(_WIN32)
    void* instance = device->getNativeObject(nvrhi::ObjectTypes::VK_Instance);
    void* physicalDevice = device->getNativeObject(nvrhi::ObjectTypes::VK_PhysicalDevice);
    HMODULE loader = GetModuleHandleW(L"vulkan-1.dll");
    if (!instance || !physicalDevice || !loader) return false;
    const auto getInstanceProcAddr =
        reinterpret_cast<VulkanGetInstanceProcAddr>(reinterpret_cast<void*>(GetProcAddress(loader, "vkGetInstanceProcAddr")));
    if (!getInstanceProcAddr) return false;
    const auto getQueueFamilyProperties = reinterpret_cast<VulkanGetQueueFamilyProperties>(
        getInstanceProcAddr(instance, "vkGetPhysicalDeviceQueueFamilyProperties"));
    if (!getQueueFamilyProperties) return false;
    uint32_t count = 0;
    getQueueFamilyProperties(physicalDevice, &count, nullptr);
    if (count == 0 || count > 64) return false;
    VulkanQueueFamilyProperties families[64] = {};
    getQueueFamilyProperties(physicalDevice, &count, families);
    bool any = false;
    for (uint32_t i = 0; i < count; ++i) {
        if (!(families[i].queueFlags & flag)) continue;
        if (families[i].timestampValidBits == 0) return false;
        any = true;
    }
    return any;
#else
    (void)device;
    (void)flag;
    return false;
#endif
}

// The ratio of the graphics queue's timestamp frequency to the compute queue's (see the top of the file).
float d3d12ComputeScale(nvrhi::IDevice* device) {
#if defined(_WIN32)
    auto* graphics = static_cast<ID3D12CommandQueue*>(
        device->getNativeQueue(nvrhi::ObjectTypes::D3D12_CommandQueue, nvrhi::CommandQueue::Graphics));
    auto* compute = static_cast<ID3D12CommandQueue*>(
        device->getNativeQueue(nvrhi::ObjectTypes::D3D12_CommandQueue, nvrhi::CommandQueue::Compute));
    UINT64 graphicsFrequency = 0, computeFrequency = 0;
    if (graphics && compute && SUCCEEDED(graphics->GetTimestampFrequency(&graphicsFrequency)) &&
        SUCCEEDED(compute->GetTimestampFrequency(&computeFrequency)) && graphicsFrequency && computeFrequency)
        return float(double(graphicsFrequency) / double(computeFrequency));
#else
    (void)device;
#endif
    return 1.0f;
}

void flushD3D11(nvrhi::IDevice* device) {
#if defined(_WIN32)
    if (auto* context = static_cast<ID3D11DeviceContext*>(device->getNativeObject(nvrhi::ObjectTypes::D3D11_DeviceContext)))
        context->Flush();
#else
    (void)device;
#endif
}

}  // namespace

// ---- StageTimerPool ------------------------------------------------------------------------------------------

StageTimerPool::StageTimerPool(IPhysDevice* device, KernelLibraryHost* host)
    : m_Device(device), m_Rhi(device->getRHIDevice()), m_Host(host), m_Api(device->getInfo().graphicsAPI) {
    const uint32_t graphics = uint32_t(nvrhi::CommandQueue::Graphics);
    const uint32_t compute = uint32_t(nvrhi::CommandQueue::Compute);
    m_QueueSupported[graphics] = true;
    switch (m_Api) {
        case nvrhi::GraphicsAPI::D3D12:
            // Timestamp queries are valid on direct and compute command lists (only a copy queue needs a cap).
            m_QueueSupported[compute] = true;
            m_QueueScale[compute] = d3d12ComputeScale(m_Rhi);
            break;
        case nvrhi::GraphicsAPI::VULKAN:
            m_QueueSupported[graphics] = vulkanTimestamps(m_Rhi, c_VulkanQueueGraphicsBit);
            m_QueueSupported[compute] = vulkanTimestamps(m_Rhi, c_VulkanQueueComputeBit);
            break;
        default:
            break;  // D3D11: one queue
    }
    m_LastSupported = m_QueueSupported[uint32_t(device->getInfo().executionQueue)];
}

StageTimerPool::~StageTimerPool() = default;

bool StageTimerPool::isQueueSupported(nvrhi::CommandQueue queue) const {
    return uint32_t(queue) < uint32_t(nvrhi::CommandQueue::Count) && m_QueueSupported[uint32_t(queue)];
}

void StageTimerPool::setBudget(uint32_t maxQueries) {
    m_Budget = maxQueries;
    m_CreateFailed = false;
    while (m_Allocated > m_Budget && !m_Free.empty()) {
        m_Free.pop_back();
        --m_Allocated;
    }
}

void StageTimerPool::setEnabled(bool enabled) {
    m_Enabled = enabled;
    if (!enabled) releaseIdle();
}

void StageTimerPool::releaseIdle() {
    m_Allocated -= uint32_t(m_Free.size());
    m_Free.clear();
}

nvrhi::TimerQueryHandle StageTimerPool::acquire() {
    nvrhi::TimerQueryHandle query;
    if (!m_Free.empty()) {
        query = m_Free.back();
        m_Free.pop_back();
        return query;
    }
    if (m_Allocated >= m_Budget || m_CreateFailed) return query;
    if (NVRHI_FAILED(m_Rhi->createTimerQuery(&query)) || !query) {
        // The device has no more (its maxTimerQueries): measure with what the pool has, until the budget is
        // set again.
        m_CreateFailed = true;
        report(m_Device->getDesc().messageCallback, PhysMessageSeverity::Warning,
               "IPhysTimers: the nvrhi device has no more timer queries (%u held by this solver); the steps are "
               "measured in part",
               m_Allocated);
        return nullptr;
    }
    ++m_Allocated;
    return query;
}

void StageTimerPool::recycle(nvrhi::TimerQueryHandle& query) {
    if (!query) return;
    m_Rhi->resetTimerQuery(query);
    if (m_Enabled && m_Allocated <= m_Budget) {
        m_Free.push_back(query);
    } else {
        --m_Allocated;
    }
    query = nullptr;
}

void StageTimerPool::beginStep(nvrhi::ICommandList* commandList, bool ownList) {
    m_Recording = nullptr;
    poll();
    ++m_TimedSteps;

    const nvrhi::CommandQueue queue = commandList->getDesc().queueType;
    m_LastSupported = isQueueSupported(queue);
    if (!m_LastSupported || m_Budget == 0) {
        ++m_SkippedSteps;
        return;
    }

    // All or nothing: a step is measured only when the pool can give it as many queries as the last step
    // asked for (or the whole of what it may hold, when that is less). A step started while its
    // predecessors still hold the queries is skipped, instead of being measured for its first few stages.
    const uint32_t limit = m_CreateFailed ? m_Allocated : m_Budget;
    const uint32_t available = uint32_t(m_Free.size()) + (limit > m_Allocated ? limit - m_Allocated : 0u);
    const uint32_t required = std::max(1u, std::min(m_Need, limit));
    if (available < required) {
        ++m_SkippedSteps;
        return;
    }

    m_QueriesLastStep = 0;
    Record& record = m_Pending.emplace_back();
    record.serial = ++m_RecordedSteps;
    record.timedStep = m_TimedSteps;
    record.ownList = ownList;
    record.ownListExecutions = m_Host->ownListExecutions();
    record.scale = m_QueueScale[uint32_t(queue)];
    record.wanted = 1;
    record.latency = acquire();
    if (record.latency) {
        commandList->beginTimerQuery(record.latency);
        ++m_QueriesLastStep;
    } else {
        record.complete = false;
    }
    m_Recording = &record;
}

int32_t StageTimerPool::beginStage(nvrhi::ICommandList* commandList, const char* name) {
    if (!m_Recording) return -1;
    ++m_Recording->wanted;
    nvrhi::TimerQueryHandle query = acquire();
    if (!query) {
        m_Recording->complete = false;
        return -1;
    }
    commandList->beginTimerQuery(query);
    Stage stage;
    stage.name = name;
    stage.query = query;
    m_Recording->stages.push_back(stage);
    ++m_QueriesLastStep;
    return int32_t(m_Recording->stages.size() - 1);
}

void StageTimerPool::endStage(nvrhi::ICommandList* commandList, int32_t slot) {
    if (!m_Recording || slot < 0 || size_t(slot) >= m_Recording->stages.size()) return;
    Stage& stage = m_Recording->stages[size_t(slot)];
    commandList->endTimerQuery(stage.query);
    stage.ended = true;
}

void StageTimerPool::endStep(nvrhi::ICommandList* commandList) {
    if (!m_Recording) return;
    if (m_Recording->latency) commandList->endTimerQuery(m_Recording->latency);
    m_Need = m_Recording->wanted;
    m_Recording = nullptr;
}

bool StageTimerPool::isSubmitted(const Record& record) const {
    // Polling a Vulkan query before the command list that resets it was submitted would read the results of
    // its previous use (or of a query never reset). D3D12 reports an unsubmitted query as not finished, and
    // D3D11 issues the queries when they are recorded. On an application's command list, the next step (on
    // any list, measured or not) is taken as the sign that the list was submitted: the application executes
    // a frame's command list before it records the next frame's step.
    if (m_Api != nvrhi::GraphicsAPI::VULKAN) return true;
    return record.ownList ? m_Host->ownListExecutions() > record.ownListExecutions : m_TimedSteps > record.timedStep;
}

bool StageTimerPool::isFinished(const Record& record) const {
    // The step's queries finish in recording order on one queue: the latency query, ended last, first.
    if (record.latency && !m_Rhi->pollTimerQuery(record.latency)) return false;
    for (const Stage& stage : record.stages)
        if (stage.ended && !m_Rhi->pollTimerQuery(stage.query)) return false;
    return true;
}

void StageTimerPool::drop(Record& record) {
    recycle(record.latency);
    for (Stage& stage : record.stages) recycle(stage.query);
}

void StageTimerPool::read(Record& record) {
    // The time of every stage name, in the order the step first ran it (FleX's TimerPool::get adds the
    // instances of a name in the order they were recorded).
    struct Named {
        const char* name;
        float time;
    };
    std::vector<Named> named;
    for (Stage& stage : record.stages) {
        float time = 0.0f;
        if (stage.ended) time = m_Rhi->getTimerQueryTime(stage.query) * record.scale;
        recycle(stage.query);
        auto it = std::find_if(named.begin(), named.end(), [&](const Named& n) { return std::strcmp(n.name, stage.name) == 0; });
        if (it == named.end())
            named.push_back({stage.name, time});
        else
            it->time += time;
    }
    float latency = 0.0f;
    if (record.latency) latency = m_Rhi->getTimerQueryTime(record.latency) * record.scale;
    recycle(record.latency);

    // Solver::GetTimers.
    PhysTimers timers;
    float total = 0.0f;
    for (const Category& category : c_Categories) {
        auto it = std::find_if(named.begin(), named.end(), [&](const Named& n) { return std::strcmp(n.name, category.stage) == 0; });
        const float time = it == named.end() ? 0.0f : it->time;
        timers.*category.member += time;
        total += time;
    }
    timers.total = total;

    // Solver::GetDetailTimers: the names in the order of FleX's name map, then the latency as "Total".
    std::sort(named.begin(), named.end(), [](const Named& a, const Named& b) { return std::strcmp(a.name, b.name) < 0; });
    std::vector<DetailTimer> details;
    details.reserve(named.size() + 1);
    for (const Named& n : named) {
        DetailTimer detail;
        detail.name = n.name;
        detail.time = n.time;
        details.push_back(detail);
    }
    DetailTimer totalDetail;
    totalDetail.name = "Total";
    totalDetail.time = latency;
    details.push_back(totalDetail);

    m_Timers = timers;
    m_Latency = latency;
    m_Details.swap(details);
    m_CompletedStep = record.serial;
    m_CompletedFull = record.complete;
}

uint64_t StageTimerPool::poll() {
    while (!m_Pending.empty()) {
        Record& record = m_Pending.front();
        if (&record == m_Recording) break;  // a stage callback polled in the middle of a step
        if (!isSubmitted(record) || !isFinished(record)) {
            if (m_TimedSteps - record.timedStep >= c_AbandonAfterSteps) {
                drop(record);
                m_Pending.pop_front();
                continue;
            }
            if (m_Api == nvrhi::GraphicsAPI::D3D11 && !record.nudged) {
                flushD3D11(m_Rhi);
                record.nudged = true;
            }
            break;
        }
        read(record);
        m_Pending.pop_front();
    }
    return m_CompletedStep;
}

PhysTimerStatus StageTimerPool::status() const {
    PhysTimerStatus status;
    status.recordedSteps = m_RecordedSteps;
    status.completedStep = m_CompletedStep;
    status.pendingSteps = uint32_t(m_Pending.size());
    status.queriesAllocated = m_Allocated;
    status.queriesLastStep = m_QueriesLastStep;
    status.skippedSteps = m_SkippedSteps;
    status.supported = m_LastSupported;
    status.complete = m_CompletedStep != 0 && m_CompletedFull;
    return status;
}

// ---- IPhysTimers -------------------------------------------------------------------------------------------

namespace {

NVRHI_CLASS_CLSID(PhysTimersView, "0409e1b4-5400-4f63-a596-edd83fcda947")
class PhysTimersView final : public nvrhi::ObjectImpl<IPhysTimers> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(PhysTimersView)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysTimersView)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysTimers)
    NVRHI_IMPLEMENTS_CLASS(PhysTimersView)
    NVRHI_END_INTERFACE_TABLE()

    PhysTimersView(IPhysSolver* solver, StageTimerPool* pool) : m_Solver(solver), m_Pool(pool) {}

    IPhysSolver* getSolver() const noexcept override { return m_Solver; }
    bool isQueueSupported(nvrhi::CommandQueue queue) const noexcept override { return m_Pool->isQueueSupported(queue); }

    void setQueryBudget(uint32_t maxQueries) noexcept override { m_Pool->setBudget(maxQueries); }
    uint32_t getQueryBudget() const noexcept override { return m_Pool->budget(); }

    uint64_t poll() noexcept override {
        try {
            return m_Pool->poll();
        } catch (...) {
            return m_Pool->status().completedStep;
        }
    }

    PhysTimerStatus& getStatus(PhysTimerStatus& retVal) const noexcept override {
        retVal = m_Pool->status();
        return retVal;
    }

    PhysTimers& getTimers(PhysTimers& retVal) const noexcept override {
        retVal = m_Pool->timers();
        return retVal;
    }

    float getDeviceLatency() const noexcept override { return m_Pool->latency(); }

    uint32_t getDetailTimerCount() const noexcept override { return uint32_t(m_Pool->details().size()); }

    DetailTimer& getDetailTimer(DetailTimer& retVal, uint32_t index) const noexcept override {
        const std::vector<DetailTimer>& details = m_Pool->details();
        retVal = index < details.size() ? details[index] : DetailTimer();
        return retVal;
    }

 private:
    PhysSolverHandle m_Solver;  // keeps m_Pool alive
    StageTimerPool* m_Pool;
};

}  // namespace

nvrhi::FRESULT createTimers(IPhysSolver* solver, StageTimerPool* pool, IPhysTimers** ppTimers) noexcept {
    if (!ppTimers) return nvrhi::FE_INVALID_ARGS;
    *ppTimers = nullptr;
    try {
        *ppTimers = MAKE_RC_OBJ(PhysTimersView, solver, pool);
        return nvrhi::FS_OK;
    } catch (const std::bad_alloc&) {
        return nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        return nvrhi::FE_GENERIC_ERROR;
    }
}

}  // namespace ephys::detail
