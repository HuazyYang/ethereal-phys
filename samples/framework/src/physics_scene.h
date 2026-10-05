#ifndef ETHEREAL_PHYS_FRAMEWORK_SRC_PHYSICS_SCENE_H
#define ETHEREAL_PHYS_FRAMEWORK_SRC_PHYSICS_SCENE_H

// PhysicsScene: what the FleX demo's Init() and frame did with NvFlex* calls, on the engine and on Donut's scene
// graph (ethereal_phys_scene).
//
// A scene's SimBuffers becomes one domain (IPhysDomain) under a PhysDomainInstance leaf, and its shapes become
// PhysShapeLeaf nodes of the same PhysSceneGraph; a PhysSceneService creates the solver when the domain joins the
// graph and runs SyncToPhysics / Step / SyncFromPhysics every frame on the frame's command list.
//
// The renderer reads the particles without a host copy: the position, phase, normal and active-index arrays of
// the domain are vertex and index buffers of the renderer (IParticleState::setArray, zero copy) where the device
// supports it, and renderer buffers updated by IParticleState::updateRenderArrays (a device-to-device copy) where
// it does not (D3D11). The smoothed positions, anisotropy, densities and diffuse particles are solver outputs and
// domain state, copied by IPhysReadback into renderer buffers on the GPU when they are drawn (FleX's interop
// path: NvFlexGet* into registered buffers).
//
// The particle arrays of SimBuffers are host mirrors: valid after Create, stale after a step, brought up to date
// by MapBuffers (a download that waits for the GPU, FleX's map), and sent back by the next BeginFrame when they
// were mapped (FleX's NvFlexSetParticles / Velocities / Phases / Active of every frame).
//
// Collision shapes are PhysShapeLeaf nodes; a triangle mesh or a distance field of SimBuffers (TriangleMeshId,
// DistanceFieldId) becomes one ITriangleMeshAsset or ISdfAsset, and one GPU mesh to draw it, however many shapes
// name it (FleX's g_meshes and g_fields). The force fields of DemoState are PhysForceFieldLeaf nodes, sent every
// frame. What a scene's Sync asked for (DemoState::SetSprings and its siblings) is uploaded on the frame's command
// list before the step.

#include <ethereal/phys/device.h>
#include <ethereal/phys_framework/demo_context.h>
#include <ethereal/phys_framework/demo_state.h>
#include <ethereal/phys_scene/scene_graph.h>
#include <ethereal/phys_scene/scene_service.h>

#include <nvrhi/nvrhi.h>

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <vector>

namespace ephys::framework {

class PhysicsScene {
 public:
    using MessageCallback = std::function<void(PhysMessageSeverity severity, const char* text)>;

    PhysicsScene(nvrhi::IDevice* device, IPhysDevice* phys, DemoContext* context, MessageCallback messages);
    ~PhysicsScene();

    PhysicsScene(const PhysicsScene&) = delete;
    PhysicsScene& operator=(const PhysicsScene&) = delete;

    // The domain (maxParticles particles, numParticles of them active), its data from g.buffers, the scene graph,
    // the service and its solver, the render buffers. Records on the device's own command list and waits for it.
    bool Create(DemoState& g, uint32_t maxParticles, uint32_t numParticles);

    // FleX's warm-up: relaxes the particles to the rest density without changing their velocity (100 steps of
    // 0.0001 s with 4 iterations, the velocities sent again after each). Waits for the GPU.
    void Warmup(DemoState& g);

    // Downloads the particle arrays and the rigid transforms into g.buffers and waits for them (once per frame).
    void MapBuffers(DemoState& g);
    [[nodiscard]] bool IsMapped() const { return m_Mapped; }

    // Start of a frame, on its command list: sends the mapped host arrays, then brings every buffer the renderer
    // reads up to date (the render array copies where they are not shared, the solver output copies the draw
    // options need).
    void BeginFrame(nvrhi::ICommandList* commandList, DemoState& g);

    // The physics of a frame, on its command list: parameters, shapes, the scene graph's Refresh and the
    // service's three passes; the solver steps when `advance`.
    bool Simulate(nvrhi::ICommandList* commandList, DemoState& g, bool advance, uint32_t frameIndex);

    // After the frame's command list was executed.
    void Submitted();

    // ---- for the renderer and the UI ----
    [[nodiscard]] FluidRenderBuffers GetFluidBuffers(bool smoothed, bool density) const;
    [[nodiscard]] DiffuseRenderBuffers GetDiffuseBuffers() const;
    [[nodiscard]] ClothRenderBuffers GetClothBuffers() const;
    // The GPU mesh a shape is drawn with: a convex's hull, a triangle mesh, a distance field's mesh; or null.
    [[nodiscard]] GpuMesh* GetShapeMesh(size_t shapeIndex) const;
    [[nodiscard]] uint32_t GetActiveCount() const;
    [[nodiscard]] bool AreRenderArraysShared() const { return m_Shared; }
    // The newest diffuse count the GPU reported (a few frames old; it is never waited for).
    [[nodiscard]] uint32_t GetDiffuseCount() const { return m_DiffuseCount; }

    [[nodiscard]] IPhysDomain* GetDomain() const { return m_Domain; }
    [[nodiscard]] IPhysSolver* GetSolver() const;
    [[nodiscard]] IPhysTimers* GetTimers() const { return m_Timers; }
    [[nodiscard]] IPhysReadback* GetReadback() const { return m_Readback; }

 private:
    struct ShapeNode;

    PhysBufferHandle uploadBuffer(const void* data, uint32_t count, uint32_t stride, const char* name);
    PhysBufferHandle readbackBuffer(uint32_t count, uint32_t stride, const char* name);
    void createShapeNodes(DemoState& g, nvrhi::ICommandList* commandList);
    void updateShapeNodes(DemoState& g, nvrhi::ICommandList* commandList);
    void syncForceFields(DemoState& g);
    void applySyncRequests(nvrhi::ICommandList* commandList, DemoState& g);
    void applyCompliance(const DemoState& g);
    PhysBufferHandle transientUpload(const void* data, uint32_t count, uint32_t stride, const char* name);
    bool createRenderBuffers(DemoState& g);
    void report(PhysMessageSeverity severity, const char* format, ...);

    nvrhi::DeviceHandle m_Device;
    PhysDeviceHandle m_Phys;
    DemoContext* m_Context;
    MessageCallback m_Messages;

    PhysDomainHandle m_Domain;
    nvrhi::AutoPtr<scene::PhysSceneGraph> m_Graph;
    nvrhi::AutoPtr<donut::engine::SceneGraphNode> m_ShapesNode;
    nvrhi::AutoPtr<scene::PhysDomainInstance> m_DomainInstance;
    std::unique_ptr<scene::PhysSceneService> m_Service;
    PhysReadbackHandle m_Readback;
    PhysTimersHandle m_Timers;

    std::vector<ShapeNode> m_Shapes;

    // the geometry assets of the triangle meshes and distance fields, one per description, and their GPU meshes
    struct TriangleMeshEntry {
        helpers::TriangleMeshId data;  // keeps the key alive
        TriangleMeshAssetHandle asset;
        std::shared_ptr<GpuMesh> renderMesh;
    };
    struct FieldEntry {
        helpers::DistanceFieldId data;
        SdfAssetHandle asset;
        std::shared_ptr<GpuMesh> renderMesh;
    };
    std::map<const helpers::TriangleMeshData*, TriangleMeshEntry> m_TriangleMeshes;
    std::map<const helpers::DistanceFieldData*, FieldEntry> m_Fields;

    // the force fields
    struct ForceFieldNode {
        nvrhi::AutoPtr<donut::engine::SceneGraphNode> node;
        nvrhi::AutoPtr<scene::PhysForceFieldLeaf> leaf;
    };
    std::vector<ForceFieldNode> m_ForceFields;
    nvrhi::AutoPtr<donut::engine::SceneGraphNode> m_ForceFieldsNode;

    // the compliance last given to an XPBD solver
    XpbdComplianceParams m_Compliance;

    // upload buffers of the frames in flight (a scene's Sync), released four frames later
    std::vector<PhysBufferHandle> m_Transient[4];
    uint32_t m_TransientSlot = 0;

    uint32_t m_MaxParticles = 0;
    bool m_Shared = false;
    bool m_HostValid = true;  // the host mirrors hold the GPU's arrays
    bool m_Mapped = false;    // MapBuffers ran this frame

    // the particle render arrays: positions, normals, phases (vertex buffers) and the active indices
    nvrhi::BufferHandle m_PositionsVB, m_NormalsVB, m_PhasesVB, m_ActiveIB;
    PhysBufferHandle m_PositionsRA, m_NormalsRA, m_PhasesRA, m_ActiveRA;
    // solver outputs
    nvrhi::BufferHandle m_SmoothVB, m_DensityVB, m_AnisotropyVB[3];
    PhysBufferHandle m_SmoothDst, m_DensityDst, m_AnisotropyDst[3];
    // diffuse
    uint32_t m_MaxDiffuse = 0;
    nvrhi::BufferHandle m_DiffusePositionsVB, m_DiffuseVelocitiesVB, m_DiffuseArgs;
    PhysBufferHandle m_DiffusePositionsDst, m_DiffuseVelocitiesDst, m_DiffuseArgsDst;
    struct CountSlot {
        PhysBufferHandle buffer;
        bool recorded = false;
    };
    CountSlot m_CountSlots[4];
    uint32_t m_CountSlot = 0;
    uint32_t m_DiffuseCount = 0;
    // cloth
    nvrhi::BufferHandle m_ClothIndices;
    PhysBufferHandle m_ClothIndicesRA;
    uint32_t m_NumTriangles = 0;

    // host mirror transfers
    PhysBufferHandle m_UploadPositions, m_UploadVelocities, m_UploadPhases, m_UploadActive;
    PhysBufferHandle m_ReadPositions, m_ReadVelocities, m_ReadNormals, m_ReadRotations, m_ReadTranslations;
};

}  // namespace ephys::framework

#endif /* ETHEREAL_PHYS_FRAMEWORK_SRC_PHYSICS_SCENE_H */
