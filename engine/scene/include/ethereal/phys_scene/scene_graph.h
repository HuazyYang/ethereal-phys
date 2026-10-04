#ifndef ETHEREAL_PHYS_SCENE_SCENE_GRAPH_H
#define ETHEREAL_PHYS_SCENE_SCENE_GRAPH_H

// Physics objects as leaves of Donut's own scene graph (step 8 of the migration plan).
//
// This is ethereal_phys_scene, a STATIC library beside the ethereal_phys DLL, and not part of it:
// donut::engine::SceneGraphLeaf derives from nvrhi::WeakReferenceSourceImpl, an implementation class the
// nvrhi object model declares module-private (nvrhi/core/foundation.h, "Module boundaries"), SceneGraph holds
// std:: containers, and donut_core / donut_engine are static libraries. None of that may cross the DLL
// boundary, so the scene integration is compiled into the application, links ethereal_phys and donut_engine,
// and talks to the engine through its COM interfaces only. The rules of com-interface-abi.md do not apply
// to these headers (std::, dynamic_cast and donut math are fine here), and the ABI checks of tests/ are not
// pointed at them.
//
// The design follows Donut's own mesh classes and the gvdb / sph samples (GVDBScene.h, SPHFluidScene.h):
//
//   IPhysDomain         shared data, as MeshInfo is (engine/include/ethereal/phys/domain.h)
//   PhysDomainInstance  the leaf that places one domain in the scene, as MeshInstance places a mesh; several
//                       instances may share one domain (Clone shares it)
//   PhysShapeLeaf       the leaf of one collision shape, whose node drives the shape's transform
//   PhysSceneGraph      the SceneGraph that tracks both kinds next to Donut's meshes, lights and cameras,
//                       with a ResourceTracker of the distinct domains (OnDomainAdded / OnDomainRemoved)
//   PhysSceneTypeFactory  creates the graph and the leaves for Donut's JSON scene loader
//
// FleX has no articulation hierarchy, so every physics object that needs a transform is a leaf of its own,
// and the hierarchy, the dirty flags, the content flags and the previous transforms are Donut's. The
// per-frame work (shapes to the engine, the step, bounds back) is PhysSceneService (scene_service.h).
//
// Transform authority, per leaf kind:
//
//   PhysShapeLeaf       Donut -> physics, every frame. The current and the previous world transform of the
//                       node become the shape's position / rotation and prevPosition / prevRotation, so an
//                       animated node gets continuous collision from Donut's animation system.
//   PhysDomainInstance  physics space -> scene space, static. FleX simulates in one world space (gravity and
//                       the collision planes are in it; a solver has no per-domain transform). The node
//                       transform only places that space in the scene for rendering, and must be rigid with
//                       a uniform scale: a non-uniform scale would make the particle radius anisotropic and
//                       the physics silently wrong. PhysSceneService rejects such an instance with a message.
//
// Which domain a shape collides with, and in which space. A shape leaf names its domain explicitly
// (SetTargetDomain, or SetTargetDomainPath for a node path, which is what the JSON "domain" property sets);
// a shape that names none collides with every domain of the graph. The simulation space of a domain is the
// space of its "primary" instance: the first instance of the domain, in registration order, whose placement
// is valid. A shape's world transform W (scene space) is expressed in it as W * inverse(P), with P the
// primary instance's node-to-world transform, the current P for both the current and the previous shape
// transform (the placement is static within a step). The other instances of a shared domain are render-only
// copies. A scale in W * inverse(P) is baked into the shape's geometry (scene_service.h says how).
//
// Asset instances and force fields (step 9b; ADR 0004):
//
//   PhysInstanceLeaf    an asset (ext/asset.h) instanced into the container of its domain (ext/container.h).
//                       Donut -> physics once, then physics is authoritative: the node's world transform at the
//                       first frame the leaf is synchronised (in the domain's simulation space) is the spawn
//                       transform, and from then on the particles are the solver's. Moving the node later moves
//                       nothing, and particle positions are never written back into the node (a cloth has no
//                       rigid transform to write). Respawn() spawns it again at the node's transform then.
//   PhysForceFieldLeaf  a radial force field (ext/force_field.h). Donut -> physics every frame, as a shape: the
//                       node's world translation, in the domain's simulation space, is the field's position.
//
// Both name their domain as a shape does (SetTargetDomain, SetTargetDomainPath, the JSON "domain"). A force
// field that names none acts in every domain of the graph; an instance that names none goes to the only domain
// of the graph, and is not spawned (with a message) when there are several. They are expressed in the space of
// the domain's primary instance, as shapes are. The containers and the force field sets are the service's, one
// of each per domain that needs one, created over the service's solver of that domain (PhysSceneService); the
// graph tracks the two leaf kinds in lists, as it does the shapes (no ResourceTracker of containers: the
// service, not a leaf, owns a container).
//
// Both carry SceneContentFlags_Physics as well.

#include <ethereal/phys/collision.h>
#include <ethereal/phys/device.h>
#include <ethereal/phys/domain.h>
#include <ethereal/phys/ext/asset.h>
#include <ethereal/phys/ext/container.h>
#include <ethereal/phys/ext/force_field.h>
#include <ethereal/phys/params.h>

#include <donut/engine/SceneGraph.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace ephys::scene {

// The content flag of every physics leaf kind. Outside the bits Donut defines (0x01..0x20) and the samples'
// gvdb::SceneContentFlags_Volumes (0x100) and sph::SceneContentFlags_Particles (0x200). Donut propagates it
// to the ancestors of a physics leaf (SceneGraphNode::GetSubgraphContentFlags) after SceneGraph::Refresh.
constexpr donut::engine::SceneContentFlags SceneContentFlags_Physics = donut::engine::SceneContentFlags(0x400);

class PhysSceneGraph;
class PhysSceneService;

// Whether the node transform of a domain instance may place a simulation space (checked by
// PhysSceneService::SyncToPhysics).
enum class PhysPlacement : uint8_t {
    Unchecked = 0,  // not seen by a service yet
    Valid = 1,      // rigid with a uniform positive scale
    Rejected = 2,   // non-uniform scale, shear, a reflection, or a scale of 0: the instance is ignored
};

// One domain placed in the scene. The node transform maps the domain's simulation space to scene space.
//
// Ownership. The leaf holds a reference to its domain; Clone shares it, as MeshInstance::Clone shares the
// MeshInfo. The solver is NOT the leaf's: it belongs to PhysSceneService, one per distinct domain, created
// when the domain first appears in the graph (PhysSceneGraph::OnDomainAdded) and released when its last
// instance leaves (OnDomainRemoved). A solver per leaf would step a shared domain once per instance; the
// ResourceTracker callbacks fire once per domain, which is exactly the lifetime a solver needs. The leaf only
// carries the SolverDesc the service creates that solver with (the desc of the first instance it sees).
class PhysDomainInstance : public donut::engine::SceneGraphLeaf {
    NVRHI_INHERIT_INTERFACE_TABLE()
 public:
    explicit PhysDomainInstance(IPhysDomain* domain = nullptr, const SolverDesc& solverDesc = SolverDesc());

    [[nodiscard]] IPhysDomain* GetDomain() const { return m_Domain; }

    // Replaces the domain. When the leaf is in a PhysSceneGraph, the graph's domain tracker follows (the
    // callbacks of the old and the new domain fire as for a leaf that leaves and joins).
    void SetDomain(IPhysDomain* domain);

    [[nodiscard]] const SolverDesc& GetSolverDesc() const { return m_SolverDesc; }
    void SetSolverDesc(const SolverDesc& desc) { m_SolverDesc = desc; }

    // The device Load creates a domain on, when the leaf has none (PhysSceneTypeFactory sets it).
    void SetDeviceForLoad(IPhysDevice* device) { m_LoadDevice = device; }

    // What the last PhysSceneService::SyncToPhysics found about the node transform.
    [[nodiscard]] PhysPlacement GetPlacement() const { return m_Placement; }

    // The bounds of the particles in the domain's simulation space (= this leaf's local space), as the
    // service last synchronised them (PhysSceneService::SyncFromPhysics); empty until then. Never reads the
    // GPU: the service copies the bounds into readback buffers and maps them only once the GPU has finished
    // with them, so the box lags the simulation by the frames the GPU is behind (scene_service.h).
    [[nodiscard]] dm::box3 GetLocalBoundingBox() override { return m_Bounds; }
    [[nodiscard]] nvrhi::AutoPtr<SceneGraphLeaf> Clone() override;
    [[nodiscard]] donut::engine::SceneContentFlags GetContentFlags() const override {
        return SceneContentFlags_Physics;
    }

    // JSON (type "PhysDomain", donut/doc/SceneFile.md):
    //   "maxParticles", "maxDiffuseParticles"     DomainDesc; a domain is created only when the leaf has none
    //   "solver": "pbd" | "xpbd"                  SolverDesc::mode
    //   "maxNeighborsPerParticle", "maxContactsPerParticle"
    //   any float member of PhysParams by name ("radius", "dynamicFriction", ...), "gravity" (3-vector),
    //   "numIterations", "planes" (array of 4-vectors)
    // Particles are not loaded from JSON: an application (or the asset containers of step 9) fills them.
    void Load(const Json::Value& node) override;

    // The node-to-world transform (identity when the leaf is not attached).
    [[nodiscard]] dm::daffine3 GetDomainToScene() const;

 private:
    friend class PhysSceneService;

    nvrhi::AutoPtr<IPhysDomain> m_Domain;
    nvrhi::AutoPtr<IPhysDevice> m_LoadDevice;
    SolverDesc m_SolverDesc;
    dm::box3 m_Bounds = dm::box3::empty();
    PhysPlacement m_Placement = PhysPlacement::Unchecked;
};

// One collision shape: its geometry in the local space of the node, its flags (type, dynamic, trigger,
// collision channels), a reference to its geometry asset when it has one, and the domain it collides with.
//
// The geometry is given unscaled; PhysSceneService bakes the scale of the node's transform (in the
// simulation space of the domain) into it every frame: a sphere radius and an SDF width by a uniform scale,
// a capsule radius by its uniform y / z scale and its half height by x, box half extents and mesh / convex
// scales per axis. A shape whose node scale the geometry cannot take (a non-uniform sphere, a sheared
// transform) is left out with a message.
class PhysShapeLeaf : public donut::engine::SceneGraphLeaf {
    NVRHI_INHERIT_INTERFACE_TABLE()
 public:
    PhysShapeLeaf();

    void SetSphere(float radius);
    void SetCapsule(float radius, float halfHeight);  // along the local x axis
    void SetBox(const dm::float3& halfExtents);
    // The leaf keeps a reference to the asset (and the shape set to every asset it was uploaded with), so
    // the application may release its own. The id is written into the geometry here: it is fixed while the
    // asset lives.
    void SetConvexMesh(IConvexAsset* asset, const dm::float3& scale = dm::float3(1.f));
    void SetTriangleMesh(ITriangleMeshAsset* asset, const dm::float3& scale = dm::float3(1.f));
    void SetSdf(ISdfAsset* asset, float width);

    [[nodiscard]] CollisionShapeType GetType() const { return m_Type; }
    [[nodiscard]] const CollisionGeometry& GetGeometry() const { return m_Geometry; }
    [[nodiscard]] nvrhi::IObject* GetAsset() const { return m_Asset; }

    // Flags. Dynamic shapes have a lower priority than static ones; a trigger reports contacts and has no
    // response. Channels: a combination of PhaseShapeChannel* (default: all), particles collide with the
    // shape when their phase shares one.
    void SetDynamic(bool dynamic) { m_Dynamic = dynamic; }
    void SetTrigger(bool trigger) { m_Trigger = trigger; }
    void SetChannels(int32_t shapeChannels) { m_Channels = shapeChannels & PhaseShapeChannelMask; }
    [[nodiscard]] bool IsDynamic() const { return m_Dynamic; }
    [[nodiscard]] bool IsTrigger() const { return m_Trigger; }
    [[nodiscard]] int32_t GetChannels() const { return m_Channels; }
    [[nodiscard]] int32_t GetFlags() const;  // what ICollisionShapeSet::upload takes

    // The domain the shape collides with. A domain wins over a path; with neither the shape collides with
    // every domain of the graph. A path is a node path as SceneGraph::FindNode takes it (absolute, or
    // relative to this leaf's node), resolved every frame by the service, to a node whose leaf is a
    // PhysDomainInstance. The leaf keeps a reference to a target domain.
    void SetTargetDomain(IPhysDomain* domain) { m_TargetDomain = domain; }
    void SetTargetDomainPath(const std::filesystem::path& path) { m_TargetDomainPath = path; }
    [[nodiscard]] IPhysDomain* GetTargetDomain() const { return m_TargetDomain; }
    [[nodiscard]] const std::filesystem::path& GetTargetDomainPath() const { return m_TargetDomainPath; }

    // The unscaled geometry's box in the node's local space.
    [[nodiscard]] dm::box3 GetLocalBoundingBox() override;
    [[nodiscard]] nvrhi::AutoPtr<SceneGraphLeaf> Clone() override;
    [[nodiscard]] donut::engine::SceneContentFlags GetContentFlags() const override {
        return SceneContentFlags_Physics;
    }

    // JSON (type "PhysShape"):
    //   "shape": "sphere" | "capsule" | "box"    with "radius", "halfHeight", "halfExtents" (3-vector)
    //   "dynamic", "trigger" (bool), "channels" (int, the PhaseShapeChannel* bits, or an array of channel
    //   numbers 0..6), "domain" (a node path, SetTargetDomainPath)
    // The asset shapes (convex, triangle mesh, distance field) are not loadable from JSON: the assets are
    // GPU objects of the device that a scene file cannot name; an application sets them.
    void Load(const Json::Value& node) override;

 private:
    friend class PhysSceneService;

    void notifyGeometryChanged();

    CollisionShapeType m_Type = CollisionShapeType::Sphere;
    CollisionGeometry m_Geometry;
    nvrhi::AutoPtr<nvrhi::IObject> m_Asset;
    bool m_Dynamic = false;
    bool m_Trigger = false;
    int32_t m_Channels = PhaseShapeChannelMask;
    nvrhi::AutoPtr<IPhysDomain> m_TargetDomain;
    std::filesystem::path m_TargetDomainPath;

    // The service frame this shape was last synchronised in: a shape not synchronised in the frame before
    // has no previous transform worth sweeping from (a node attached since, or a graph not refreshed).
    uint64_t m_LastSyncFrame = 0;
    // The service left the shape out in the last frame (reported once, when it starts).
    bool m_Rejected = false;
};

// One asset instanced into the container of a domain (ext/container.h). The leaf holds the asset and the
// arguments of IPhysContainer::createInstance besides the transform; the service creates the instance (it owns
// the containers) at the first PhysSceneService::SyncToPhysics that finds the leaf in a graph with its domain
// placed, with the node's world transform in the domain's simulation space as the spawn transform. That
// transform must be rigid there: the asset's rest shape is not scaled, so a scale would be undone by the
// constraints (and is refused with a message, as a shear or a mirroring is).
//
// The leaf holds the instance (GetInstance) while it is in a graph: unregistering the leaf releases it, which
// frees its particles, and a graph it joins spawns it again. Clone copies the settings, not the instance: the
// clone spawns one of its own.
class PhysInstanceLeaf : public donut::engine::SceneGraphLeaf {
    NVRHI_INHERIT_INTERFACE_TABLE()
 public:
    explicit PhysInstanceLeaf(IPhysAsset* asset = nullptr);

    // The asset to instance; the leaf keeps a reference (and the instance one more). Changing it respawns.
    void SetAsset(IPhysAsset* asset);
    [[nodiscard]] IPhysAsset* GetAsset() const { return m_Asset; }

    // The arguments of createInstance: the initial velocity (in the domain's space), the phase of every particle
    // (makePhase; the default, makePhase(0, 0), collides with every shape channel and not with group 0 itself),
    // and the factor on the inverse masses. They count at the next spawn.
    void SetVelocity(const dm::float3& velocity) { m_Velocity = velocity; }
    void SetPhase(int32_t phase) { m_Phase = phase; }
    void SetInvMassScale(float scale) { m_InvMassScale = scale; }
    [[nodiscard]] const dm::float3& GetVelocity() const { return m_Velocity; }
    [[nodiscard]] int32_t GetPhase() const { return m_Phase; }
    [[nodiscard]] float GetInvMassScale() const { return m_InvMassScale; }

    // The domain the instance goes to, as PhysShapeLeaf names one; with neither, the only domain of the graph.
    void SetTargetDomain(IPhysDomain* domain) { m_TargetDomain = domain; }
    void SetTargetDomainPath(const std::filesystem::path& path) { m_TargetDomainPath = path; }
    [[nodiscard]] IPhysDomain* GetTargetDomain() const { return m_TargetDomain; }
    [[nodiscard]] const std::filesystem::path& GetTargetDomainPath() const { return m_TargetDomainPath; }

    // The instance, once the service has spawned it; nullptr before, and after Respawn until the next spawn.
    [[nodiscard]] IPhysInstance* GetInstance() const { return m_Instance; }

    // Releases the instance (its particles are freed at the next push); the service spawns it again at the next
    // SyncToPhysics, at the node's world transform then.
    void Respawn() { m_Instance = nullptr; }

    // The asset's particles in the node's local space: where the instance spawns, not where its particles are
    // now (those are the solver's; the domain instance's box covers them).
    [[nodiscard]] dm::box3 GetLocalBoundingBox() override;
    [[nodiscard]] nvrhi::AutoPtr<SceneGraphLeaf> Clone() override;
    [[nodiscard]] donut::engine::SceneContentFlags GetContentFlags() const override {
        return SceneContentFlags_Physics;
    }

    // JSON (type "PhysInstance"): "domain" (a node path), "velocity" (3-vector), "phase" (an int, makePhase's
    // value) or "group" (int) with "selfCollide", "selfCollideFilter", "fluid" (bool), "invMassScale". The asset
    // is not loadable from a scene file (an asset builder makes it on the host); an application sets it.
    void Load(const Json::Value& node) override;

 private:
    friend class PhysSceneService;
    friend class PhysSceneGraph;

    PhysAssetHandle m_Asset;
    dm::float3 m_Velocity = dm::float3(0.f);
    int32_t m_Phase = makePhase(0, 0);
    float m_InvMassScale = 1.0f;
    nvrhi::AutoPtr<IPhysDomain> m_TargetDomain;
    std::filesystem::path m_TargetDomainPath;
    PhysInstanceHandle m_Instance;
    bool m_Rejected = false;  // reported once
};

// One radial force field (ext/force_field.h). The node's world translation, in the simulation space of the
// domain, is the field's position every frame; the radius, strength, mode and falloff are the leaf's (the
// node's scale and rotation do not change them).
class PhysForceFieldLeaf : public donut::engine::SceneGraphLeaf {
    NVRHI_INHERIT_INTERFACE_TABLE()
 public:
    PhysForceFieldLeaf() = default;

    void SetRadius(float radius) { m_Field.radius = radius; }
    void SetStrength(float strength) { m_Field.strength = strength; }
    void SetMode(ForceMode mode) { m_Field.mode = mode; }
    void SetLinearFalloff(bool linearFalloff) { m_Field.linearFalloff = linearFalloff; }
    // The field without its position (that is the node's).
    [[nodiscard]] const ForceField& GetField() const { return m_Field; }

    // The domain the field acts in, as PhysShapeLeaf names one; with neither, every domain of the graph.
    void SetTargetDomain(IPhysDomain* domain) { m_TargetDomain = domain; }
    void SetTargetDomainPath(const std::filesystem::path& path) { m_TargetDomainPath = path; }
    [[nodiscard]] IPhysDomain* GetTargetDomain() const { return m_TargetDomain; }
    [[nodiscard]] const std::filesystem::path& GetTargetDomainPath() const { return m_TargetDomainPath; }

    // The sphere of the field.
    [[nodiscard]] dm::box3 GetLocalBoundingBox() override {
        return dm::box3(dm::float3(-m_Field.radius), dm::float3(m_Field.radius));
    }
    [[nodiscard]] nvrhi::AutoPtr<SceneGraphLeaf> Clone() override;
    [[nodiscard]] donut::engine::SceneContentFlags GetContentFlags() const override {
        return SceneContentFlags_Physics;
    }

    // JSON (type "PhysForceField"): "radius", "strength", "mode": "force" | "impulse" | "velocityChange",
    // "linearFalloff" (bool), "domain" (a node path).
    void Load(const Json::Value& node) override;

 private:
    friend class PhysSceneService;

    ForceField m_Field;
    nvrhi::AutoPtr<IPhysDomain> m_TargetDomain;
    std::filesystem::path m_TargetDomainPath;
    bool m_Rejected = false;  // reported once
};

// The scene graph with physics leaves tracked next to Donut's. RegisterLeaf / UnregisterLeaf call the base
// class first, so meshes, lights, cameras and animations are tracked as by Donut.
class PhysSceneGraph : public donut::engine::SceneGraph {
    NVRHI_INHERIT_INTERFACE_TABLE()
 public:
    using SceneGraph::SceneGraph;

    // In registration order.
    [[nodiscard]] const std::vector<nvrhi::AutoPtr<PhysDomainInstance>>& GetDomainInstances() const {
        return m_DomainInstances;
    }
    [[nodiscard]] const std::vector<nvrhi::AutoPtr<PhysShapeLeaf>>& GetShapeLeaves() const { return m_ShapeLeaves; }
    [[nodiscard]] const std::vector<nvrhi::AutoPtr<PhysInstanceLeaf>>& GetInstanceLeaves() const {
        return m_InstanceLeaves;
    }
    [[nodiscard]] const std::vector<nvrhi::AutoPtr<PhysForceFieldLeaf>>& GetForceFieldLeaves() const {
        return m_ForceFieldLeaves;
    }

    // The distinct domains the instances hold: a domain is added with its first instance and removed with
    // its last, and each callback fires once per domain.
    [[nodiscard]] const donut::engine::ResourceTracker<IPhysDomain>& GetDomains() const { return m_Domains; }

    donut::engine::SceneResourceCallback<IPhysDomain> OnDomainAdded;
    donut::engine::SceneResourceCallback<IPhysDomain> OnDomainRemoved;

 protected:
    void RegisterLeaf(donut::engine::SceneGraphLeaf* leaf) override;
    void UnregisterLeaf(donut::engine::SceneGraphLeaf* leaf) override;

 private:
    friend class PhysDomainInstance;

    void addDomainRef(IPhysDomain* domain);
    void releaseDomainRef(IPhysDomain* domain);

    std::vector<nvrhi::AutoPtr<PhysDomainInstance>> m_DomainInstances;
    std::vector<nvrhi::AutoPtr<PhysShapeLeaf>> m_ShapeLeaves;
    std::vector<nvrhi::AutoPtr<PhysInstanceLeaf>> m_InstanceLeaves;
    std::vector<nvrhi::AutoPtr<PhysForceFieldLeaf>> m_ForceFieldLeaves;
    donut::engine::ResourceTracker<IPhysDomain> m_Domains;
};

// Creates a PhysSceneGraph and the physics leaves for Donut's scene loader (donut::engine::Scene takes it as
// its SceneTypeFactory). Leaf types: "PhysDomain" (PhysDomainInstance, its domain created on the device of
// the factory by Load), "PhysShape" (PhysShapeLeaf), "PhysInstance" (PhysInstanceLeaf, without its asset) and
// "PhysForceField" (PhysForceFieldLeaf); every other type goes to Donut's factory.
//
// Donut's loader attaches a leaf (SetLeaf, which registers it) BEFORE it calls Load, so a "PhysDomain" leaf
// joins the graph without a domain and Load gives it one through SetDomain, which brings the graph's domain
// tracker up to date. A shape's "domain" path is resolved by the service each frame, so the order of the
// nodes in the file does not matter.
class PhysSceneTypeFactory : public donut::engine::SceneTypeFactory {
    NVRHI_INHERIT_INTERFACE_TABLE()
 public:
    explicit PhysSceneTypeFactory(IPhysDevice* device = nullptr) : m_Device(device) {}

    nvrhi::AutoPtr<donut::engine::SceneGraph> CreateGraph() override;
    nvrhi::AutoPtr<donut::engine::SceneGraphLeaf> CreateLeaf(const std::string& type) override;

 private:
    nvrhi::AutoPtr<IPhysDevice> m_Device;
};

}  // namespace ephys::scene

#endif /* ETHEREAL_PHYS_SCENE_SCENE_GRAPH_H */
