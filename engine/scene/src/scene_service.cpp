// The physics scene service (scene_service.h).

#include <ethereal/phys_scene/scene_service.h>

#include "scene_util.h"

#include <donut/core/log.h>

#ifdef _WIN32
#include <d3d11.h>
#endif

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ephys::scene {

using donut::engine::SceneGraphNode;

namespace {

// The completion fence of a bounds copy. An nvrhi event query, except on D3D11, where nvrhi's
// pollEventQuery takes S_FALSE ("not yet") from ID3D11DeviceContext::GetData for done (SUCCEEDED(S_FALSE)
// is true): a poll would always report the copy finished, the map would then wait for the GPU, and the
// next End on the query would abandon a result never read (a debug-layer warning). On D3D11 the fence is
// a native event query polled for S_OK. GetData never flushes there (DONOTFLUSH), so a poll that finds a
// copy unfinished flushes the immediate context once, as the engine's timers do
// (engine/src/solver/stage_timers.cpp): an application that never presents would otherwise never see one.
class Fence {
 public:
    Fence() = default;
    Fence(const Fence&) = delete;
    Fence& operator=(const Fence&) = delete;
    Fence(Fence&& other) noexcept { swap(other); }
    Fence& operator=(Fence&& other) noexcept {
        swap(other);
        return *this;
    }
    ~Fence() {
#ifdef _WIN32
        if (m_D3D11) m_D3D11->Release();
#endif
    }

    bool create(nvrhi::IDevice* rhi) {
        m_Rhi = rhi;
#ifdef _WIN32
        if (rhi->getGraphicsAPI() == nvrhi::GraphicsAPI::D3D11) {
            m_Context = static_cast<ID3D11DeviceContext*>(rhi->getNativeObject(nvrhi::ObjectTypes::D3D11_DeviceContext));
            auto* device = static_cast<ID3D11Device*>(rhi->getNativeObject(nvrhi::ObjectTypes::D3D11_Device));
            D3D11_QUERY_DESC desc = {};
            desc.Query = D3D11_QUERY_EVENT;
            return m_Context && device && SUCCEEDED(device->CreateQuery(&desc, &m_D3D11));
        }
#endif
        return NVRHI_SUCCEEDED(rhi->createEventQuery(&m_Query));
    }

    // After the work it fences was submitted to `queue`.
    void signal(nvrhi::CommandQueue queue) {
#ifdef _WIN32
        if (m_D3D11) {
            m_Context->End(m_D3D11);
            return;
        }
#endif
        m_Rhi->resetEventQuery(m_Query);
        m_Rhi->setEventQuery(m_Query, queue);
    }

    // Never waits. *nudge: the work is not done yet and the D3D11 context should be flushed.
    bool poll(bool* nudge) {
#ifdef _WIN32
        if (m_D3D11) {
            const bool done = m_Context->GetData(m_D3D11, nullptr, 0, D3D11_ASYNC_GETDATA_DONOTFLUSH) == S_OK;
            *nudge |= !done;
            return done;
        }
#endif
        return m_Rhi->pollEventQuery(m_Query);
    }

 private:
    void swap(Fence& other) noexcept {
        std::swap(m_Rhi, other.m_Rhi);
        std::swap(m_Query, other.m_Query);
#ifdef _WIN32
        std::swap(m_D3D11, other.m_D3D11);
        std::swap(m_Context, other.m_Context);
#endif
    }

    nvrhi::IDevice* m_Rhi = nullptr;
    nvrhi::EventQueryHandle m_Query;
#ifdef _WIN32
    ID3D11Query* m_D3D11 = nullptr;
    ID3D11DeviceContext* m_Context = nullptr;  // the device keeps it alive
#endif
};

void flushD3D11(nvrhi::IDevice* device) {
#ifdef _WIN32
    if (auto* context =
            static_cast<ID3D11DeviceContext*>(device->getNativeObject(nvrhi::ObjectTypes::D3D11_DeviceContext)))
        context->Flush();
#else
    (void)device;
#endif
}

std::string nodePath(const donut::engine::SceneGraphLeaf* leaf) {
    SceneGraphNode* node = leaf->GetNode();
    return node ? node->GetPath().generic_string() : std::string("(detached)");
}

const char* shapeTypeName(CollisionShapeType type) {
    switch (type) {
        case CollisionShapeType::Sphere:
            return "sphere";
        case CollisionShapeType::Capsule:
            return "capsule";
        case CollisionShapeType::Box:
            return "box";
        case CollisionShapeType::ConvexMesh:
            return "convex mesh";
        case CollisionShapeType::TriangleMesh:
            return "triangle mesh";
        case CollisionShapeType::Sdf:
            return "distance field";
    }
    return "shape";
}

// One shape in a domain's simulation space, as ICollisionShapeSet::upload takes it.
struct ShapeArrays {
    std::vector<CollisionGeometry> geometry;
    std::vector<float> positions, rotations, prevPositions, prevRotations;
    std::vector<int32_t> flags;

    void clear() {
        geometry.clear();
        positions.clear();
        rotations.clear();
        prevPositions.clear();
        prevRotations.clear();
        flags.clear();
    }
    uint32_t size() const { return uint32_t(flags.size()); }

    // Everything, as bytes: what the last upload is compared with.
    std::vector<uint8_t> bytes() const {
        std::vector<uint8_t> out;
        auto append = [&out](const void* data, size_t size) {
            const auto* p = static_cast<const uint8_t*>(data);
            out.insert(out.end(), p, p + size);
        };
        append(geometry.data(), geometry.size() * sizeof(CollisionGeometry));
        append(positions.data(), positions.size() * sizeof(float));
        append(rotations.data(), rotations.size() * sizeof(float));
        append(prevPositions.data(), prevPositions.size() * sizeof(float));
        append(prevRotations.data(), prevRotations.size() * sizeof(float));
        append(flags.data(), flags.size() * sizeof(int32_t));
        return out;
    }
};

void pushTransform(std::vector<float>& positions, std::vector<float>& rotations, const detail::Decomposition& d,
                   const dm::dquat& q) {
    positions.insert(positions.end(), {float(d.translation.x), float(d.translation.y), float(d.translation.z), 0.f});
    rotations.insert(rotations.end(), {float(q.x), float(q.y), float(q.z), float(q.w)});
}

}  // namespace

struct PhysSceneService::Impl {
    enum class SlotState : uint8_t { Free, Recorded, Submitted };
    struct Slot {
        PhysBufferHandle staging;
        Fence fence;
        SlotState state = SlotState::Free;
        uint64_t frame = 0;
    };

    struct DomainState {
        PhysDomainHandle domain;
        PhysSolverHandle solver;
        bool solverFailed = false;
        bool foreignReported = false;

        // This frame (SyncToPhysics).
        PhysDomainInstance* primary = nullptr;
        dm::daffine3 sceneToSim = dm::daffine3::identity();

        // The frame of the last step, and whether its bounds still need a copy: a frame whose readback
        // buffers are all in flight records none, and a later frame (stepped or not) records it instead.
        uint64_t lastStepFrame = 0;
        bool boundsCopyPending = false;

        bool uploaded = false;
        std::vector<uint8_t> lastUpload;
        uint32_t numShapes = 0;

        // The asset instances of the domain (PhysInstanceLeaf), created with the first one, over the solver; and
        // the force fields (PhysForceFieldLeaf), registered on the solver with the first one.
        PhysContainerHandle container;
        bool containerFailed = false;
        ForceFieldSetHandle forceFields;
        bool forceFieldsFailed = false;
        std::vector<uint8_t> lastFields;
        uint32_t numForceFields = 0;

        std::vector<Slot> slots;
        dm::box3 bounds = dm::box3::empty();
        uint64_t boundsFrame = 0;
    };

    nvrhi::AutoPtr<PhysSceneGraph> graph;
    PhysDeviceHandle device;
    nvrhi::IDevice* rhi = nullptr;
    uint32_t slotCount = 4;
    PhysSceneMessageCallback messageCallback;
    donut::engine::SceneResourceCallback<IPhysDomain> previousAdded, previousRemoved;

    std::unordered_map<const IPhysDomain*, DomainState> domains;
    uint64_t frame = 0;
    uint64_t copiesSkipped = 0;
    bool pendingRefreshReported = false;

    void report(PhysMessageSeverity severity, const char* format, ...) {
        char text[1024];
        va_list args;
        va_start(args, format);
        std::vsnprintf(text, sizeof(text), format, args);
        va_end(args);
        if (messageCallback) {
            messageCallback(severity, text);
            return;
        }
        switch (severity) {
            case PhysMessageSeverity::Info:
                donut::log::info("%s", text);
                break;
            case PhysMessageSeverity::Warning:
                donut::log::warning("%s", text);
                break;
            default:
                donut::log::error("%s", text);
                break;
        }
    }

    // A domain joins: its state, and its solver, created with the desc of the first instance that holds it.
    // The solver is created here, when the domain enters the graph, and not in a per-frame pass: creating it
    // records clears on the device's own command list, which on D3D11 (one immediate context) may not be
    // opened while a pass records on an application command list.
    void addDomain(const IPhysDomain* domain) {
        if (!domain || domains.count(domain)) return;
        DomainState& state = domains[domain];
        state.domain = const_cast<IPhysDomain*>(domain);
        createSolver(state);
    }

    void createSolver(DomainState& state) {
        if (state.solver || state.solverFailed || state.domain->getDevice() != device.Get()) return;
        PhysDomainInstance* first = nullptr;
        for (PhysDomainInstance* instance : graph->GetDomainInstances())
            if (instance->GetDomain() == state.domain.Get()) {
                first = instance;
                break;
            }
        if (!first) return;
        if (NVRHI_FAILED(device->createSolver(state.domain, first->GetSolverDesc(), &state.solver))) {
            report(PhysMessageSeverity::Error, "PhysDomainInstance '%s': the solver cannot be created",
                   nodePath(first).c_str());
            state.solverFailed = true;
        }
    }

    void removeDomain(const IPhysDomain* domain) {
        // The solver and the readback buffers go; nvrhi keeps what a command list in flight still uses. The
        // instances of the domain's container go first (they keep it, and through it the solver, alive); their
        // leaves spawn them again if the domain comes back.
        if (DomainState* state = find(domain)) releaseInstances(*state);
        domains.erase(domain);
    }

    void releaseInstances(DomainState& state) {
        if (!state.container) return;
        for (PhysInstanceLeaf* leaf : graph->GetInstanceLeaves())
            if (leaf->m_Instance && leaf->m_Instance->getContainer() == state.container.Get()) leaf->m_Instance = nullptr;
    }

    DomainState* find(const IPhysDomain* domain) {
        auto it = domains.find(domain);
        return it == domains.end() ? nullptr : &it->second;
    }

    // The domain a leaf names: its target, the domain of the instance at its path, or nullptr for none.
    // *unresolved: a path that names no domain instance (the leaf is left out).
    IPhysDomain* resolveTarget(IPhysDomain* target, const std::filesystem::path& path,
                               const donut::engine::SceneGraphLeaf* leaf, bool* unresolved) {
        *unresolved = false;
        if (target) return target;
        if (path.empty()) return nullptr;
        SceneGraphNode* node = graph->FindNode(path, leaf->GetNode());
        auto* instance = node ? dynamic_cast<PhysDomainInstance*>(node->GetLeaf()) : nullptr;
        if (!instance || !instance->GetDomain()) {
            *unresolved = true;
            return nullptr;
        }
        return instance->GetDomain();
    }

    // The domain a shape collides with: its target, or nullptr for every domain.
    IPhysDomain* targetOf(PhysShapeLeaf* shape, bool* unresolved) {
        return resolveTarget(shape->GetTargetDomain(), shape->GetTargetDomainPath(), shape, unresolved);
    }

    void syncInstances(bool& ok);
    void syncForceFields(bool& ok);

    // The geometry of a shape with the scale of its transform baked in; false and *problem when the
    // geometry cannot take that scale.
    static bool bake(const PhysShapeLeaf* shape, const dm::double3& s, CollisionGeometry& g, const char** problem) {
        g = shape->GetGeometry();
        switch (shape->GetType()) {
            case CollisionShapeType::Sphere:
                if (!detail::isUniform(s)) {
                    *problem = "a sphere takes a uniform scale only";
                    return false;
                }
                g.sphere.radius *= float(s.x);
                return true;
            case CollisionShapeType::Capsule:
                if (!detail::nearlyEqual(s.y, s.z)) {
                    *problem = "a capsule takes one scale for its radius (y and z must be equal)";
                    return false;
                }
                g.capsule.radius *= float(s.y);
                g.capsule.halfHeight *= float(s.x);
                return true;
            case CollisionShapeType::Box:
                for (int i = 0; i < 3; ++i) g.box.halfExtents[i] *= float(s[i]);
                return true;
            case CollisionShapeType::ConvexMesh:
            case CollisionShapeType::TriangleMesh:
                if (g.convexMesh.mesh == 0) {
                    *problem = "the shape has no asset";
                    return false;
                }
                for (int i = 0; i < 3; ++i) g.convexMesh.scale[i] *= float(s[i]);
                return true;
            case CollisionShapeType::Sdf:
                if (g.sdf.field == 0) {
                    *problem = "the shape has no asset";
                    return false;
                }
                if (!detail::isUniform(s)) {
                    *problem = "a distance field takes a uniform scale only";
                    return false;
                }
                g.sdf.scale *= float(s.x);
                return true;
        }
        *problem = "the shape type is not one";
        return false;
    }

    bool syncToPhysics(nvrhi::ICommandList* commandList);
    bool step(float dt, uint32_t substeps, nvrhi::ICommandList* commandList);
    uint64_t syncFromPhysics(nvrhi::ICommandList* commandList);
    void poll();
    void markSubmitted(nvrhi::CommandQueue queue);
};

// ---- SyncToPhysics -----------------------------------------------------------------------------------------

bool PhysSceneService::Impl::syncToPhysics(nvrhi::ICommandList* commandList) {
    ++frame;
    bool ok = true;

    // After a Refresh the root keeps SubgraphPrevTransforms (the next Refresh updates the previous transforms),
    // so that one is not a pending change.
    const SceneGraphNode* root = graph->GetRootNode();
    const bool pending = root && (root->GetDirtyFlags() & (SceneGraphNode::DirtyFlags::LocalTransform |
                                                            SceneGraphNode::DirtyFlags::SubgraphStructure |
                                                            SceneGraphNode::DirtyFlags::SubgraphTransforms)) != 0;
    if (pending && !pendingRefreshReported) {
        report(PhysMessageSeverity::Warning,
               "PhysSceneService::SyncToPhysics: the scene graph has changes that SceneGraph::Refresh has not "
               "processed yet; the transforms sent to the physics are those of the last Refresh");
        pendingRefreshReported = true;
    }

    // Domains the service has not seen (the callbacks keep the map in step; this covers a graph whose
    // callbacks were replaced by the application).
    for (IPhysDomain* domain : graph->GetDomains()) addDomain(domain);
    for (auto& [key, state] : domains) state.primary = nullptr;

    // Placements: rigid with a uniform scale, else the instance is rejected. The first valid instance of a
    // domain, in registration order, is its primary: the shapes are expressed in its space.
    for (PhysDomainInstance* instance : graph->GetDomainInstances()) {
        IPhysDomain* domain = instance->GetDomain();
        DomainState* state = domain ? find(domain) : nullptr;
        if (!state) continue;
        if (domain->getDevice() != device.Get()) {
            if (!state->foreignReported)
                report(PhysMessageSeverity::Error,
                       "PhysDomainInstance '%s': its domain was created by another physics device than the "
                       "service's; it is not simulated",
                       nodePath(instance).c_str());
            state->foreignReported = true;
            ok = false;
            continue;
        }
        const dm::daffine3 domainToScene = instance->GetDomainToScene();
        const detail::Decomposition d = detail::decompose(domainToScene);
        const bool valid = d.positive && !d.mirrored && d.orthogonal && detail::isUniform(d.scale);
        if (!valid) {
            if (instance->m_Placement != PhysPlacement::Rejected) {
                const char* reason = !d.positive    ? "a zero or non-finite scale"
                                     : d.mirrored   ? "a mirroring (a negative scale)"
                                     : !d.orthogonal ? "a shear"
                                                     : "a non-uniform scale";
                report(PhysMessageSeverity::Error,
                       "PhysDomainInstance '%s': the node-to-world transform has %s (scale %.6g, %.6g, %.6g). A "
                       "domain is placed by a rigid transform with a uniform scale only: FleX simulates in one "
                       "isotropic space, and any other placement would make the particle radius anisotropic. The "
                       "instance is ignored until its transform is fixed.",
                       nodePath(instance).c_str(), reason, d.scale.x, d.scale.y, d.scale.z);
            }
            instance->m_Placement = PhysPlacement::Rejected;
            ok = false;
            continue;
        }
        if (instance->m_Placement == PhysPlacement::Rejected)
            report(PhysMessageSeverity::Info, "PhysDomainInstance '%s': the placement is valid again",
                   nodePath(instance).c_str());
        instance->m_Placement = PhysPlacement::Valid;
        if (!state->primary) {
            state->primary = instance;
            state->sceneToSim = dm::inverse(domainToScene);
        }
    }

    // A domain whose solver could not be created when it joined (the graph's callbacks were replaced): create
    // it now, unless that would open the device's own command list beside the application's on D3D11.
    if (!commandList || rhi->getGraphicsAPI() != nvrhi::GraphicsAPI::D3D11)
        for (auto& [key, state] : domains)
            if (!state.solver && !state.solverFailed && state.primary) {
                createSolver(state);
                ok &= state.solver != nullptr;
            }

    // Shapes. A shape not synchronised in the previous frame sweeps from its current transform: its previous
    // one is not a position it was simulated at.
    struct ShapeEntry {
        PhysShapeLeaf* leaf;
        bool fresh;
        bool wasRejected;
        IPhysDomain* target;  // nullptr: every domain
    };
    auto reject = [&](ShapeEntry& entry, const char* problem) {
        if (!entry.wasRejected && !entry.leaf->m_Rejected)
            report(PhysMessageSeverity::Error, "PhysShapeLeaf '%s' (%s): %s; the shape is left out",
                   nodePath(entry.leaf).c_str(), shapeTypeName(entry.leaf->GetType()), problem);
        entry.leaf->m_Rejected = true;
        ok = false;
    };
    std::vector<ShapeEntry> shapes;
    shapes.reserve(graph->GetShapeLeaves().size());
    for (PhysShapeLeaf* leaf : graph->GetShapeLeaves()) {
        // m_LastSyncFrame 0: never synchronised (frames count from 1).
        ShapeEntry entry{leaf, leaf->m_LastSyncFrame == 0 || leaf->m_LastSyncFrame + 1 != frame, leaf->m_Rejected,
                         nullptr};
        leaf->m_LastSyncFrame = frame;
        leaf->m_Rejected = false;
        bool unresolved = false;
        entry.target = targetOf(leaf, &unresolved);
        if (unresolved) {
            reject(entry, "its domain path names no node with a PhysDomainInstance that has a domain");
            continue;
        }
        if (!leaf->GetNode()) continue;
        shapes.push_back(entry);
    }

    ShapeArrays arrays;
    for (auto& [key, state] : domains) {
        if (!state.primary) continue;  // not simulated this frame: its shape set stays as it is
        arrays.clear();
        for (ShapeEntry& entry : shapes) {
            if (entry.target && entry.target != state.domain.Get()) continue;
            SceneGraphNode* node = entry.leaf->GetNode();

            const dm::daffine3 current = node->GetLocalToWorldTransform() * state.sceneToSim;
            const dm::daffine3 previous =
                entry.fresh ? current : node->GetPrevLocalToWorldTransform() * state.sceneToSim;
            const detail::Decomposition d = detail::decompose(current);
            const detail::Decomposition p = detail::decompose(previous);
            if (!d.positive || d.mirrored || !d.orthogonal || !p.positive || p.mirrored || !p.orthogonal) {
                reject(entry, "its transform in the domain's space is sheared, mirrored or degenerate");
                continue;
            }
            CollisionGeometry geometry;
            const char* problem = nullptr;
            if (!bake(entry.leaf, d.scale, geometry, &problem)) {
                reject(entry, problem);
                continue;
            }
            // q and -q are one rotation; keep the two on the same side so that a sweep between them is the
            // short one.
            dm::dquat prevRotation = p.rotation;
            const dm::dquat& q = d.rotation;
            if (q.w * prevRotation.w + q.x * prevRotation.x + q.y * prevRotation.y + q.z * prevRotation.z < 0.0)
                prevRotation = -prevRotation;

            arrays.geometry.push_back(geometry);
            pushTransform(arrays.positions, arrays.rotations, d, q);
            pushTransform(arrays.prevPositions, arrays.prevRotations, p, prevRotation);
            arrays.flags.push_back(entry.leaf->GetFlags());
        }

        std::vector<uint8_t> bytes = arrays.bytes();
        if (state.uploaded && bytes == state.lastUpload) continue;
        CollisionShapeData data;
        data.geometry = arrays.geometry.data();
        data.positions = arrays.positions.data();
        data.rotations = arrays.rotations.data();
        data.prevPositions = arrays.prevPositions.data();
        data.prevRotations = arrays.prevRotations.data();
        data.flags = arrays.flags.data();
        data.numShapes = arrays.size();
        if (NVRHI_FAILED(state.domain->getShapes()->upload(commandList, data))) {
            report(PhysMessageSeverity::Error, "PhysDomainInstance '%s': the shapes cannot be uploaded",
                   nodePath(state.primary).c_str());
            state.uploaded = false;
            ok = false;
            continue;
        }
        state.uploaded = true;
        state.lastUpload = std::move(bytes);
        state.numShapes = arrays.size();
    }

    syncInstances(ok);
    syncForceFields(ok);
    return ok;
}

// Asset instances: each leaf is spawned once, into the container of its domain, at its node's world transform in
// the domain's simulation space; from then on the particles are the solver's and the node is not looked at.
void PhysSceneService::Impl::syncInstances(bool& ok) {
    for (PhysInstanceLeaf* leaf : graph->GetInstanceLeaves()) {
        SceneGraphNode* node = leaf->GetNode();
        if (!node || !leaf->GetAsset()) continue;
        auto reject = [&](const char* problem) {
            if (!leaf->m_Rejected)
                report(PhysMessageSeverity::Error, "PhysInstanceLeaf '%s': %s; the instance is not spawned",
                       nodePath(leaf).c_str(), problem);
            leaf->m_Rejected = true;
            ok = false;
        };

        bool unresolved = false;
        IPhysDomain* target = resolveTarget(leaf->GetTargetDomain(), leaf->GetTargetDomainPath(), leaf, &unresolved);
        if (unresolved) {
            reject("its domain path names no node with a PhysDomainInstance that has a domain");
            continue;
        }
        if (!target) {
            // The only domain of the graph.
            uint32_t count = 0;
            for (IPhysDomain* domain : graph->GetDomains()) {
                target = domain;
                ++count;
            }
            if (count != 1) {
                target = nullptr;
                reject(count ? "it names no domain, and the graph has several" : "the graph has no domain");
                continue;
            }
        }
        DomainState* state = find(target);
        if (!state) {
            reject("its domain is not in the graph");
            continue;
        }

        if (leaf->m_Instance) {
            if (state->container && leaf->m_Instance->getContainer() == state->container.Get()) continue;  // spawned
            leaf->m_Instance = nullptr;  // in another container: spawned again here
        }
        if (!state->primary || !state->solver) continue;  // not simulated this frame; spawned when it is

        if (!state->container) {
            if (state->containerFailed) continue;
            const uint32_t capacity = state->domain->getParticles()->getMaxParticles();
            if (NVRHI_FAILED(device->createContainer(state->solver, capacity, &state->container))) {
                report(PhysMessageSeverity::Error, "PhysDomainInstance '%s': the container cannot be created",
                       nodePath(state->primary).c_str());
                state->containerFailed = true;
                ok = false;
                continue;
            }
        }

        // The spawn transform, rigid in the domain's space: the asset's rest shape is not scaled.
        const dm::daffine3 spawn = node->GetLocalToWorldTransform() * state->sceneToSim;
        const detail::Decomposition d = detail::decompose(spawn);
        if (!d.positive || d.mirrored || !d.orthogonal || !detail::nearlyEqual(d.scale.x, 1.0, 1e-4) ||
            !detail::nearlyEqual(d.scale.y, 1.0, 1e-4) || !detail::nearlyEqual(d.scale.z, 1.0, 1e-4)) {
            reject("its transform in the domain's space is not rigid (a scale, a shear or a mirroring)");
            continue;
        }
        // FleX's column-major matrix for column vectors: column i is the image of axis i, which is row i of
        // Donut's row-vector linear part.
        float transform[16] = {};
        const dm::double3 rows[3] = {spawn.m_linear.row0, spawn.m_linear.row1, spawn.m_linear.row2};
        for (int i = 0; i < 3; ++i)
            for (int k = 0; k < 3; ++k) transform[i * 4 + k] = float(rows[i][k] / d.scale[i]);
        for (int k = 0; k < 3; ++k) transform[12 + k] = float(spawn.m_translation[k]);
        transform[15] = 1.0f;
        const float velocity[3] = {leaf->m_Velocity.x, leaf->m_Velocity.y, leaf->m_Velocity.z};
        if (NVRHI_FAILED(state->container->createInstance(nullptr, leaf->m_Asset, transform, velocity, leaf->m_Phase,
                                                          leaf->m_InvMassScale, &leaf->m_Instance))) {
            reject("the container cannot take it (see the engine's message)");
            continue;
        }
        if (leaf->m_Rejected)
            report(PhysMessageSeverity::Info, "PhysInstanceLeaf '%s': spawned", nodePath(leaf).c_str());
        leaf->m_Rejected = false;
    }
}

// Force fields: every frame, the node's world translation in the domain's space is the field's position.
void PhysSceneService::Impl::syncForceFields(bool& ok) {
    struct Entry {
        PhysForceFieldLeaf* leaf;
        IPhysDomain* target;  // nullptr: every domain
    };
    std::vector<Entry> entries;
    for (PhysForceFieldLeaf* leaf : graph->GetForceFieldLeaves()) {
        if (!leaf->GetNode()) continue;
        bool unresolved = false;
        IPhysDomain* target = resolveTarget(leaf->GetTargetDomain(), leaf->GetTargetDomainPath(), leaf, &unresolved);
        if (unresolved) {
            if (!leaf->m_Rejected)
                report(PhysMessageSeverity::Error,
                       "PhysForceFieldLeaf '%s': its domain path names no node with a PhysDomainInstance that has a "
                       "domain; the field is left out",
                       nodePath(leaf).c_str());
            leaf->m_Rejected = true;
            ok = false;
            continue;
        }
        leaf->m_Rejected = false;
        entries.push_back({leaf, target});
    }

    for (auto& [key, state] : domains) {
        if (!state.primary || !state.solver) continue;  // not simulated this frame: its fields stay as they are
        std::vector<ForceField> fields;
        for (const Entry& entry : entries) {
            if (entry.target && entry.target != state.domain.Get()) continue;
            const dm::daffine3 placed = entry.leaf->GetNode()->GetLocalToWorldTransform() * state.sceneToSim;
            ForceField field = entry.leaf->GetField();
            for (int k = 0; k < 3; ++k) field.position[k] = float(placed.m_translation[k]);
            fields.push_back(field);
        }
        if (fields.empty() && !state.forceFields) continue;
        if (!state.forceFields) {
            if (state.forceFieldsFailed) continue;
            if (NVRHI_FAILED(device->createForceFieldSet(state.solver, &state.forceFields))) {
                report(PhysMessageSeverity::Error, "PhysDomainInstance '%s': the force fields cannot be created",
                       nodePath(state.primary).c_str());
                state.forceFieldsFailed = true;
                ok = false;
                continue;
            }
        }
        std::vector<uint8_t> bytes(fields.size() * sizeof(ForceField));
        if (!fields.empty()) std::memcpy(bytes.data(), fields.data(), bytes.size());
        if (state.numForceFields == fields.size() && bytes == state.lastFields) continue;
        if (NVRHI_FAILED(state.forceFields->setForceFields(fields.data(), uint32_t(fields.size())))) {
            report(PhysMessageSeverity::Error, "PhysDomainInstance '%s': the force fields cannot be set",
                   nodePath(state.primary).c_str());
            ok = false;
            continue;
        }
        state.lastFields = std::move(bytes);
        state.numForceFields = uint32_t(fields.size());
    }
}

// ---- Step --------------------------------------------------------------------------------------------------

bool PhysSceneService::Impl::step(float dt, uint32_t substeps, nvrhi::ICommandList* commandList) {
    bool ok = true;
    for (auto& [key, state] : domains) {
        if (!state.primary || !state.solver) continue;
        // The container's particles and constraints that changed (instances spawned or released) go first.
        if (state.container && NVRHI_FAILED(state.container->pushToDevice(commandList))) {
            report(PhysMessageSeverity::Error, "PhysDomainInstance '%s': the container cannot push its changes",
                   nodePath(state.primary).c_str());
            ok = false;
            continue;
        }
        if (NVRHI_FAILED(state.solver->step(commandList, dt, substeps))) {
            report(PhysMessageSeverity::Error, "PhysDomainInstance '%s': the step failed",
                   nodePath(state.primary).c_str());
            ok = false;
            continue;
        }
        state.lastStepFrame = frame;
        state.boundsCopyPending = true;
    }
    return ok;
}

// ---- SyncFromPhysics ---------------------------------------------------------------------------------------

void PhysSceneService::Impl::poll() {
    bool unfinished = false;
    for (auto& [key, state] : domains) {
        for (Slot& slot : state.slots) {
            if (slot.state != SlotState::Submitted) continue;
            if (!slot.fence.poll(&unfinished)) continue;
            float values[6] = {};
            void* memory = nullptr;
            if (NVRHI_SUCCEEDED(slot.staging->map(&memory)) && memory) {
                std::memcpy(values, memory, sizeof(values));
                slot.staging->unmap();
                if (slot.frame > state.boundsFrame) {
                    const dm::float3 lower(values[0], values[1], values[2]);
                    const dm::float3 upper(values[3], values[4], values[5]);
                    state.bounds = dm::any(lower > upper) || !dm::all(dm::isfinite(lower)) ||
                                           !dm::all(dm::isfinite(upper))
                                       ? dm::box3::empty()
                                       : dm::box3(lower, upper);
                    state.boundsFrame = slot.frame;
                }
            }
            slot.state = SlotState::Free;
        }
    }
    if (unfinished) flushD3D11(rhi);
}

void PhysSceneService::Impl::markSubmitted(nvrhi::CommandQueue queue) {
    for (auto& [key, state] : domains)
        for (Slot& slot : state.slots)
            if (slot.state == SlotState::Recorded) {
                slot.fence.signal(queue);
                slot.state = SlotState::Submitted;
            }
}

uint64_t PhysSceneService::Impl::syncFromPhysics(nvrhi::ICommandList* commandList) {
    poll();

    // The newest bounds to every instance of their domain (an instance's local space is the simulation space).
    for (PhysDomainInstance* instance : graph->GetDomainInstances()) {
        DomainState* state = instance->GetDomain() ? find(instance->GetDomain()) : nullptr;
        if (!state || state->boundsFrame == 0 || instance->m_Bounds == state->bounds) continue;
        instance->m_Bounds = state->bounds;
        if (SceneGraphNode* node = instance->GetNode()) detail::refreshNodeBounds(node);
    }

    // This frame's copies.
    for (auto& [key, state] : domains) {
        if (!state.boundsCopyPending) continue;
        IPhysBuffer* bounds = state.domain->getParticles()->getBounds();
        Slot* free = nullptr;
        for (Slot& slot : state.slots)
            if (slot.state == SlotState::Free) {
                free = &slot;
                break;
            }
        if (!free && state.slots.size() < slotCount) {
            Slot slot;
            PhysBufferDesc desc;
            desc.elementCount = 2;
            desc.elementStride = bounds->getElementStride();
            desc.use = BufferUse::Readback;
            desc.debugName = "ephys scene bounds readback";
            if (NVRHI_FAILED(device->createBuffer(desc, &slot.staging)) ||
                !slot.fence.create(rhi)) {
                report(PhysMessageSeverity::Error, "PhysSceneService: a bounds readback buffer cannot be created");
                continue;
            }
            state.slots.push_back(std::move(slot));
            free = &state.slots.back();
        }
        if (!free) {
            ++copiesSkipped;
            continue;
        }
        CopyRegion region;
        region.elementCount = 2;
        if (NVRHI_FAILED(device->copyBuffer(commandList, free->staging, bounds, region))) {
            report(PhysMessageSeverity::Error, "PhysSceneService: the bounds copy cannot be recorded");
            continue;
        }
        free->state = SlotState::Recorded;
        free->frame = state.lastStepFrame;
        state.boundsCopyPending = false;
    }

    if (commandList) return 0;
    const uint64_t instance = device->flush();
    markSubmitted(device->getInfo().executionQueue);
    return instance;
}

// ---- PhysSceneService --------------------------------------------------------------------------------------

PhysSceneService::PhysSceneService(PhysSceneGraph* graph, const PhysSceneServiceDesc& desc)
    : m_Impl(std::make_unique<Impl>()) {
    Impl& s = *m_Impl;
    s.graph = graph;
    s.device = desc.device;
    s.rhi = desc.device ? desc.device->getRHIDevice() : nullptr;
    s.slotCount = desc.boundsReadbackSlots ? desc.boundsReadbackSlots : 1;
    s.messageCallback = desc.messageCallback;
    if (!graph || !desc.device) {
        s.report(PhysMessageSeverity::Error, "PhysSceneService: a graph and a device are required");
        return;
    }

    s.previousAdded = graph->OnDomainAdded;
    s.previousRemoved = graph->OnDomainRemoved;
    Impl* impl = m_Impl.get();
    graph->OnDomainAdded = [impl](const IPhysDomain* domain) {
        impl->addDomain(domain);
        if (impl->previousAdded) impl->previousAdded(domain);
    };
    graph->OnDomainRemoved = [impl](const IPhysDomain* domain) {
        impl->removeDomain(domain);
        if (impl->previousRemoved) impl->previousRemoved(domain);
    };
    for (IPhysDomain* domain : graph->GetDomains()) s.addDomain(domain);
}

PhysSceneService::~PhysSceneService() {
    Impl& s = *m_Impl;
    if (s.graph && s.device) {
        s.graph->OnDomainAdded = s.previousAdded;
        s.graph->OnDomainRemoved = s.previousRemoved;
        // The containers are the service's: their instances go with them (a later service spawns them again).
        for (auto& [key, state] : s.domains) s.releaseInstances(state);
    }
    s.domains.clear();
}

bool PhysSceneService::SyncToPhysics(nvrhi::ICommandList* commandList) {
    if (!m_Impl->graph || !m_Impl->device) return false;
    return m_Impl->syncToPhysics(commandList);
}

bool PhysSceneService::Step(float dt, uint32_t substeps, nvrhi::ICommandList* commandList) {
    if (!m_Impl->graph || !m_Impl->device) return false;
    return m_Impl->step(dt, substeps, commandList);
}

uint64_t PhysSceneService::SyncFromPhysics(nvrhi::ICommandList* commandList) {
    if (!m_Impl->graph || !m_Impl->device) return 0;
    return m_Impl->syncFromPhysics(commandList);
}

void PhysSceneService::NotifySubmitted(nvrhi::CommandQueue queue) {
    if (!m_Impl->graph || !m_Impl->device) return;
    m_Impl->markSubmitted(queue);
}

IPhysSolver* PhysSceneService::GetSolver(const IPhysDomain* domain) const {
    const Impl::DomainState* state = m_Impl->find(domain);
    return state ? state->solver.Get() : nullptr;
}

PhysDomainInstance* PhysSceneService::GetPrimaryInstance(const IPhysDomain* domain) const {
    const Impl::DomainState* state = m_Impl->find(domain);
    return state ? state->primary : nullptr;
}

uint32_t PhysSceneService::GetNumShapes(const IPhysDomain* domain) const {
    const Impl::DomainState* state = m_Impl->find(domain);
    return state ? state->numShapes : 0;
}

uint64_t PhysSceneService::GetBoundsFrame(const IPhysDomain* domain) const {
    const Impl::DomainState* state = m_Impl->find(domain);
    return state ? state->boundsFrame : 0;
}

IPhysContainer* PhysSceneService::GetContainer(const IPhysDomain* domain) const {
    const Impl::DomainState* state = m_Impl->find(domain);
    return state ? state->container.Get() : nullptr;
}

IForceFieldSet* PhysSceneService::GetForceFieldSet(const IPhysDomain* domain) const {
    const Impl::DomainState* state = m_Impl->find(domain);
    return state ? state->forceFields.Get() : nullptr;
}

uint64_t PhysSceneService::GetFrame() const { return m_Impl->frame; }
uint64_t PhysSceneService::GetBoundsCopiesSkipped() const { return m_Impl->copiesSkipped; }
PhysSceneGraph* PhysSceneService::GetGraph() const { return m_Impl->graph; }
IPhysDevice* PhysSceneService::GetDevice() const { return m_Impl->device; }

}  // namespace ephys::scene
