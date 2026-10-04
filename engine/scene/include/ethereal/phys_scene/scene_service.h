#ifndef ETHEREAL_PHYS_SCENE_SCENE_SERVICE_H
#define ETHEREAL_PHYS_SCENE_SCENE_SERVICE_H

// PhysSceneService: the per-frame passes that connect a PhysSceneGraph (scene_graph.h) with the engine.
//
// A plain class, not a COM object: it lives in the application, next to the graph it serves.
//
// A frame:
//
//     graph->Refresh(frameIndex);          // Donut: transforms, previous transforms, bounds, content flags
//     service.SyncToPhysics(cl);           // placements checked, shapes (current + previous) to the domains
//     service.Step(dt, substeps, cl);      // one step of every domain with a valid placement
//     service.SyncFromPhysics(cl);         // particle bounds back to the instances, without waiting
//
// One Refresh and one SyncToPhysics per frame: a shape's previous transform is the node's transform at the
// previous Refresh (SceneGraphNode::GetPrevLocalToWorldTransform), and the step sweeps the shape from there.
//
// Command lists. As everywhere in the engine (ephys::IPhysDevice, "The device owns one command list"), a
// pass takes the open command list of the application, or nullptr for the device's own:
//
// - nullptr: the passes record on the device's own command list, and SyncFromPhysics flushes it
//   (IPhysDevice::flush) and returns the instance, for nvrhi::IDevice::queueWaitForCommandList.
// - an application command list (graphics): the passes record on it and do not execute it. After the
//   application has executed it, it calls NotifySubmitted(queue), which tells the service that the bounds
//   copies recorded on it are on the GPU. Use the same command list for the three passes of a frame.
//
// The solver does not run on a compute queue yet (D3D12 removes the device, Vulkan records fragment-stage
// barriers on the compute-only family; see tests/readback_test.cpp). The service therefore steps only on a
// graphics command list: Step refuses, with a message, the device's own list when the device was created
// with preferComputeQueue and got a compute queue.
//
// Bounds without blocking. SyncFromPhysics records a copy of every stepped domain's particle bounds
// (IParticleState::getBounds) into one of a few Readback buffers per domain and marks it with an nvrhi event
// query once it is submitted. At the start of each SyncFromPhysics it polls those queries (never waits) and
// maps only the buffers whose copies the GPU has finished; the newest becomes the local bounding box of every
// instance of the domain. So GetLocalBoundingBox at frame N describes the step of frame N - k, with k >= 1
// the number of frames the GPU is behind plus one (the copy of frame N is polled from frame N + 1 on). With
// all buffers of a domain in flight a frame records no copy (GetBoundsCopiesSkipped) rather than waiting; the
// next SyncFromPhysics with a free buffer copies the newest step's bounds instead, stepped or not. On D3D11
// the completion fence is a native event query: nvrhi's D3D11 pollEventQuery reports a query done when
// GetData says S_FALSE ("not yet").
// When the box changes, the service re-sets the node's local transform to its own value, which makes the
// next Refresh recompute the node's global bounding box: Donut recomputes bounding boxes only for nodes with
// transform or structure changes, and InvalidateContent does not count as one.
//
// Shapes. The service owns the shape set of every domain in the graph: SyncToPhysics replaces it with the
// shape leaves that collide with the domain (scene_graph.h), possibly none, and uploads only when the arrays
// differ from the last upload (a static scene does not make the solver rebuild its shape hierarchy).
//
// Messages (an invalid placement, a shape the geometry cannot take, an engine call that failed) go to the
// callback of the desc, or to donut::log; each state change is reported once, not every frame.

#include <ethereal/phys_scene/scene_graph.h>

#include <ethereal/phys/device.h>
#include <ethereal/phys/solver.h>

#include <nvrhi/nvrhi.h>

#include <cstdint>
#include <functional>
#include <memory>

namespace ephys::scene {

using PhysSceneMessageCallback = std::function<void(PhysMessageSeverity severity, const char* text)>;

struct PhysSceneServiceDesc {
    // Required: the device of the domains (a domain of another device is ignored with a message).
    IPhysDevice* device = nullptr;
    // Readback buffers per domain for the bounds: how many frames of copies may be in flight. At least 1.
    uint32_t boundsReadbackSlots = 4;
    // Optional; default: donut::log.
    PhysSceneMessageCallback messageCallback;
};

class PhysSceneService {
 public:
    // Takes over the graph's OnDomainAdded / OnDomainRemoved and chains to the callbacks that were set
    // before; the destructor puts them back. The service keeps the graph alive.
    PhysSceneService(PhysSceneGraph* graph, const PhysSceneServiceDesc& desc);
    ~PhysSceneService();

    PhysSceneService(const PhysSceneService&) = delete;
    PhysSceneService& operator=(const PhysSceneService&) = delete;

    // Pass 1, after SceneGraph::Refresh. Checks the node transform of every domain instance (rigid, uniform
    // scale), picks the primary instance of each domain, and replaces the shape set of each domain with its
    // shapes in the domain's simulation space. Returns false when anything
    // was rejected or failed (the messages say what); the rest is synchronised regardless.
    bool SyncToPhysics(nvrhi::ICommandList* commandList = nullptr);

    // Pass 2. Steps every domain that has a solver and a valid placement this frame, once, by dt in
    // `substeps` substeps (IPhysSolver::step). Returns false when a step failed or was refused.
    bool Step(float dt, uint32_t substeps, nvrhi::ICommandList* commandList = nullptr);

    // Pass 3. Polls the bounds copies of earlier frames (no wait) and gives the instances the newest, then
    // records this frame's copies. With a null command list, flushes the device's own command list and
    // returns IPhysDevice::flush's instance; otherwise returns 0.
    uint64_t SyncFromPhysics(nvrhi::ICommandList* commandList = nullptr);

    // With an application command list: call after executing it, with the queue it ran on.
    void NotifySubmitted(nvrhi::CommandQueue queue = nvrhi::CommandQueue::Graphics);

    // The solver of a domain of the graph (no reference added); nullptr when the domain is not in the graph.
    // The service creates it when the domain joins the graph (OnDomainAdded, or the constructor for the
    // domains already there), with the SolverDesc of the first instance that holds the domain, and releases
    // it when the domain's last instance leaves. Creating a solver records clears on the device's own command
    // list (IPhysDevice::createSolver), which the next flush submits; on D3D11 flush before opening an
    // application command list, as for any work on the device's own list.
    [[nodiscard]] IPhysSolver* GetSolver(const IPhysDomain* domain) const;

    // The primary instance of a domain this frame (the one whose space the shapes are expressed in), or
    // nullptr when it has no instance with a valid placement.
    [[nodiscard]] PhysDomainInstance* GetPrimaryInstance(const IPhysDomain* domain) const;

    // The number of shapes the last SyncToPhysics gave a domain.
    [[nodiscard]] uint32_t GetNumShapes(const IPhysDomain* domain) const;

    // The service frame (SyncToPhysics calls) whose step the domain's bounds describe; 0: none yet.
    [[nodiscard]] uint64_t GetBoundsFrame(const IPhysDomain* domain) const;

    // SyncToPhysics calls so far.
    [[nodiscard]] uint64_t GetFrame() const;

    // Bounds copies not recorded because every readback buffer of the domain was still in flight.
    [[nodiscard]] uint64_t GetBoundsCopiesSkipped() const;

    [[nodiscard]] PhysSceneGraph* GetGraph() const;
    [[nodiscard]] IPhysDevice* GetDevice() const;

 private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
};

}  // namespace ephys::scene

#endif /* ETHEREAL_PHYS_SCENE_SCENE_SERVICE_H */
