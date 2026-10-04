// Tests of ethereal_phys_scene: the physics leaves of Donut's scene graph and PhysSceneService.
//
//   scene_test graph                      the graph alone, no device: leaf tracking, the domain tracker and
//                                         its callbacks, Clone, the content flag, detaching, the JSON factory
//   scene_test <d3d11|d3d12|vulkan>       the service on a real, headless device (solver_support.h): shape
//                                         transforms in a domain's space, collision with an animated shape,
//                                         placement rejection, bounds without blocking, an application
//                                         command list, JSON domains, reference counts
//
// As in the other device tests, every nvrhi, debug-layer, Donut log or engine message the test did not
// provoke is a failure; the service's own messages go to a sink that counts them, and each test checks that
// it saw exactly the ones it provoked.

#include <ethereal/phys_scene/scene_graph.h>
#include <ethereal/phys_scene/scene_service.h>

#include "solver_support.h"

#include <donut/core/json.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace {

using namespace support;
using donut::engine::SceneContentFlags;
using donut::engine::SceneGraphNode;
using ephys::scene::PhysDomainInstance;
using ephys::scene::PhysForceFieldLeaf;
using ephys::scene::PhysInstanceLeaf;
using ephys::scene::PhysPlacement;
using ephys::scene::PhysSceneGraph;
using ephys::scene::PhysSceneService;
using ephys::scene::PhysSceneServiceDesc;
using ephys::scene::PhysSceneTypeFactory;
using ephys::scene::PhysShapeLeaf;
using ephys::scene::SceneContentFlags_Physics;

bool hasPhysics(SceneContentFlags flags) { return (flags & SceneContentFlags_Physics) != 0; }

nvrhi::AutoPtr<PhysSceneGraph> makeGraph() {
    auto graph = MAKE_RC_OBJ_PTR(PhysSceneGraph);
    graph->SetRootNode(MAKE_RC_OBJ_PTR(SceneGraphNode));
    graph->GetRootNode()->SetName("root");
    return graph;
}

nvrhi::AutoPtr<SceneGraphNode> addNode(PhysSceneGraph* graph, SceneGraphNode* parent, const char* name,
                                       donut::engine::SceneGraphLeaf* leaf = nullptr) {
    auto node = MAKE_RC_OBJ_PTR(SceneGraphNode);
    node->SetName(name);
    if (leaf) node->SetLeaf(leaf);
    return graph->Attach(parent, node);
}

Json::Value parseJson(const char* text) {
    Json::Value value;
    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    std::string errors;
    CHECK(reader->parse(text, text + std::strlen(text), &value, &errors));
    return value;
}

// ---- The graph alone ------------------------------------------------------------------------------------------

// A domain that is nothing but an identity: the graph tests need no device.
class FakeDomain final : public nvrhi::ObjectImpl<ephys::IPhysDomain> {
 public:
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(FakeDomain)
    NVRHI_IMPLEMENTS_INTERFACE(ephys::IPhysDomain)
    NVRHI_END_INTERFACE_TABLE()

    const ephys::DomainDesc& getDesc() const noexcept override { return m_Desc; }
    ephys::IPhysDevice* getDevice() const noexcept override { return nullptr; }
    ephys::IParticleState* getParticles() const noexcept override { return nullptr; }
    ephys::IDiffuseParticleState* getDiffuseParticles() const noexcept override { return nullptr; }
    ephys::ISpringSet* getSprings() const noexcept override { return nullptr; }
    ephys::IShapeMatchSet* getShapeMatches() const noexcept override { return nullptr; }
    ephys::ITriangleSet* getTriangles() const noexcept override { return nullptr; }
    ephys::IInflatableSet* getInflatables() const noexcept override { return nullptr; }
    ephys::ICollisionShapeSet* getShapes() const noexcept override { return nullptr; }
    void setParams(const ephys::PhysParams& params) noexcept override { m_Params = params; }
    const ephys::PhysParams& getParams() const noexcept override { return m_Params; }
    uint64_t getParamsRevision() const noexcept override { return 0; }

 private:
    ephys::DomainDesc m_Desc;
    ephys::PhysParams m_Params;
};

struct DomainEvents {
    std::vector<const ephys::IPhysDomain*> added, removed;
    void attach(PhysSceneGraph* graph) {
        graph->OnDomainAdded = [this](const ephys::IPhysDomain* d) { added.push_back(d); };
        graph->OnDomainRemoved = [this](const ephys::IPhysDomain* d) { removed.push_back(d); };
    }
};

void testGraphTracking() {
    std::printf("graph: tracking\n");
    auto domainA = MAKE_RC_OBJ_PTR(FakeDomain);
    auto domainB = MAKE_RC_OBJ_PTR(FakeDomain);
    const nvrhi::FLONG refsA = refCount(domainA.Get());
    {
        auto graph = makeGraph();
        DomainEvents events;
        events.attach(graph);
        SceneGraphNode* root = graph->GetRootNode();

        // Two instances of one domain: one domain, added once.
        ephys::SolverDesc xpbd;
        xpbd.mode = ephys::SolverMode::XPBD;
        auto instance1 = MAKE_RC_OBJ_PTR(PhysDomainInstance, domainA.Get(), xpbd);
        auto node1 = addNode(graph, root, "fluid1", instance1);
        auto group = addNode(graph, root, "group");
        auto instance2 = MAKE_RC_OBJ_PTR(PhysDomainInstance, domainA.Get());
        auto node2 = addNode(graph, group, "fluid2", instance2);
        CHECK(graph->GetDomainInstances().size() == 2);
        CHECK(graph->GetDomains().size() == 1);
        CHECK(events.added.size() == 1 && events.added[0] == domainA.Get());
        CHECK(events.removed.empty());

        // Clone shares the domain (as MeshInstance::Clone shares the MeshInfo) and copies the solver desc.
        auto clone = instance1->Clone();
        auto* cloned = dynamic_cast<PhysDomainInstance*>(clone.Get());
        CHECK(cloned && cloned->GetDomain() == domainA.Get());
        CHECK(cloned && cloned->GetSolverDesc().mode == ephys::SolverMode::XPBD);
        clone = nullptr;

        // Attaching a node that is in the graph copies it, and the copy's leaf is a clone: a third instance.
        auto copy = graph->Attach(group, node1);
        CHECK(copy.Get() != node1.Get());
        CHECK(graph->GetDomainInstances().size() == 3);
        CHECK(graph->GetDomains().size() == 1);
        CHECK(events.added.size() == 1);
        auto* copiedInstance = dynamic_cast<PhysDomainInstance*>(copy->GetLeaf());
        CHECK(copiedInstance && copiedInstance != instance1.Get() && copiedInstance->GetDomain() == domainA.Get());

        // A shape leaf, and a node of another kind beside them.
        auto shape = MAKE_RC_OBJ_PTR(PhysShapeLeaf);
        shape->SetBox(dm::float3(1.f, 2.f, 3.f));
        auto shapeNode = addNode(graph, group, "shape", shape);
        auto camera = MAKE_RC_OBJ_PTR(donut::engine::PerspectiveCamera);
        auto cameraBranch = addNode(graph, root, "cameras");
        auto cameraNode = addNode(graph, cameraBranch, "camera", camera);
        CHECK(graph->GetShapeLeaves().size() == 1);
        CHECK(graph->GetCameras().size() == 1);

        // The content flag 0x400 reaches every ancestor of a physics leaf, and only them.
        graph->Refresh(0);
        CHECK(SceneContentFlags_Physics == SceneContentFlags(0x400));
        CHECK(hasPhysics(node1->GetLeafContentFlags()));
        CHECK(hasPhysics(shapeNode->GetLeafContentFlags()));
        CHECK(hasPhysics(group->GetSubgraphContentFlags()));
        CHECK(hasPhysics(root->GetSubgraphContentFlags()));
        CHECK(!hasPhysics(cameraBranch->GetSubgraphContentFlags()));
        CHECK((cameraBranch->GetSubgraphContentFlags() & SceneContentFlags::Cameras) != 0);
        // The shape's box reaches the node's global bounding box.
        CHECK(shapeNode->GetGlobalBoundingBox() == dm::box3(dm::float3(-1, -2, -3), dm::float3(1, 2, 3)));

        // Detaching a subtree unregisters every leaf in it: two instances and the shape go, the domain stays
        // (instance1 still holds it), so no callback.
        graph->Detach(group);
        CHECK(graph->GetDomainInstances().size() == 1);
        CHECK(graph->GetShapeLeaves().empty());
        CHECK(graph->GetDomains().size() == 1);
        CHECK(events.removed.empty());
        graph->Refresh(1);
        CHECK(hasPhysics(root->GetSubgraphContentFlags()));

        // SetDomain on a registered instance moves the tracker: A leaves (removed once), B joins (added once).
        instance1->SetDomain(domainB);
        CHECK(graph->GetDomains().size() == 1);
        CHECK(events.removed.size() == 1 && events.removed[0] == domainA.Get());
        CHECK(events.added.size() == 2 && events.added[1] == domainB.Get());

        // Replacing the leaf of a node unregisters the instance; the domain leaves with its last instance.
        node1->SetLeaf(MAKE_RC_OBJ_PTR(donut::engine::PerspectiveCamera));
        CHECK(graph->GetDomainInstances().empty());
        CHECK(graph->GetDomains().empty());
        CHECK(events.removed.size() == 2 && events.removed[1] == domainB.Get());
        graph->Refresh(2);
        CHECK(!hasPhysics(root->GetSubgraphContentFlags()));

        // An instance without a domain is tracked as an instance, not as a domain; SetDomain fixes that.
        auto empty = MAKE_RC_OBJ_PTR(PhysDomainInstance);
        auto emptyNode = addNode(graph, root, "empty", empty);
        CHECK(graph->GetDomainInstances().size() == 1 && graph->GetDomains().empty());
        empty->SetDomain(domainA);
        CHECK(graph->GetDomains().size() == 1 && events.added.size() == 3);
        graph->Detach(emptyNode);
        CHECK(events.removed.size() == 3 && graph->GetDomains().empty());

        // Re-attaching the detached subtree (now graph-less) registers its leaves again, without a copy.
        auto reattached = graph->Attach(root, group);
        CHECK(reattached.Get() == group.Get());
        CHECK(graph->GetDomainInstances().size() == 2 && graph->GetShapeLeaves().size() == 1);
        CHECK(graph->GetDomains().size() == 1 && events.added.size() == 4);
        graph->Detach(group);
        CHECK(events.removed.size() == 4);
    }
    // Every reference the graph and the leaves took is back.
    CHECK(refCount(domainA.Get()) == refsA);
}

void testGraphFactory() {
    std::printf("graph: factory and JSON\n");
    auto factory = MAKE_RC_OBJ_PTR(PhysSceneTypeFactory);
    auto graph = factory->CreateGraph();
    auto* physGraph = dynamic_cast<PhysSceneGraph*>(graph.Get());
    CHECK(physGraph != nullptr);
    if (!physGraph) return;
    graph->SetRootNode(MAKE_RC_OBJ_PTR(SceneGraphNode));

    // As Donut's scene loader does it: create by type, SetLeaf (registers), then Load.
    auto leaf = factory->CreateLeaf("PhysShape");
    auto* shape = dynamic_cast<PhysShapeLeaf*>(leaf.Get());
    CHECK(shape != nullptr);
    if (!shape) return;
    auto node = addNode(physGraph, graph->GetRootNode(), "ground");
    node->SetLeaf(leaf);
    shape->Load(parseJson(R"({"type": "PhysShape", "shape": "box", "halfExtents": [2, 0.5, 3],
                              "dynamic": true, "channels": [0, 2], "domain": "/fluid"})"));
    CHECK(shape->GetType() == ephys::CollisionShapeType::Box);
    CHECK(shape->GetGeometry().box.halfExtents[0] == 2.f && shape->GetGeometry().box.halfExtents[1] == 0.5f &&
          shape->GetGeometry().box.halfExtents[2] == 3.f);
    CHECK(shape->IsDynamic() && !shape->IsTrigger());
    CHECK(shape->GetChannels() == (ephys::PhaseShapeChannel0 | ephys::PhaseShapeChannel2));
    CHECK(shape->GetFlags() == ephys::makeShapeFlagsWithChannels(ephys::CollisionShapeType::Box, true,
                                                                  ephys::PhaseShapeChannel0 | ephys::PhaseShapeChannel2));
    CHECK(shape->GetTargetDomainPath() == std::filesystem::path("/fluid"));
    CHECK(physGraph->GetShapeLeaves().size() == 1);

    auto capsule = factory->CreateLeaf("PhysShape");
    capsule->Load(parseJson(R"({"shape": "capsule", "radius": 0.2, "halfHeight": 0.7, "trigger": true,
                                "channels": 16777216})"));
    auto* capsuleShape = dynamic_cast<PhysShapeLeaf*>(capsule.Get());
    CHECK(capsuleShape && capsuleShape->GetType() == ephys::CollisionShapeType::Capsule);
    CHECK(capsuleShape && capsuleShape->GetGeometry().capsule.halfHeight == 0.7f && capsuleShape->IsTrigger());
    CHECK(capsuleShape && capsuleShape->GetChannels() == ephys::PhaseShapeChannel0);
    CHECK(capsuleShape && (capsuleShape->GetFlags() & ephys::ShapeFlagTrigger) != 0);

    // Donut's own leaf types still come from Donut's factory; unknown ones are nullptr.
    CHECK(dynamic_cast<donut::engine::PerspectiveCamera*>(factory->CreateLeaf("PerspectiveCamera").Get()) != nullptr);
    CHECK(factory->CreateLeaf("NoSuchLeaf") == nullptr);
    CHECK(dynamic_cast<PhysDomainInstance*>(factory->CreateLeaf("PhysDomain").Get()) != nullptr);
}

void testGraphExtensionLeaves() {
    std::printf("graph: instance and force field leaves\n");
    auto domain = MAKE_RC_OBJ_PTR(FakeDomain);
    const nvrhi::FLONG refs = refCount(domain.Get());
    {
        auto graph = makeGraph();
        SceneGraphNode* root = graph->GetRootNode();
        auto instance = MAKE_RC_OBJ_PTR(PhysInstanceLeaf);
        instance->SetVelocity(dm::float3(1.f, 2.f, 3.f));
        instance->SetPhase(ephys::makePhase(4, ephys::PhaseSelfCollide));
        instance->SetInvMassScale(0.5f);
        instance->SetTargetDomain(domain);
        auto field = MAKE_RC_OBJ_PTR(PhysForceFieldLeaf);
        field->SetRadius(0.5f);
        field->SetStrength(-2.f);
        field->SetMode(ephys::ForceMode::Impulse);
        field->SetLinearFalloff(true);
        auto branch = addNode(graph, root, "fx");
        auto instanceNode = addNode(graph, branch, "instance", instance);
        auto fieldNode = addNode(graph, branch, "field", field);
        auto other = addNode(graph, root, "other");
        CHECK(graph->GetInstanceLeaves().size() == 1 && graph->GetForceFieldLeaves().size() == 1);
        CHECK(graph->GetInstanceLeaves()[0] == instance.Get() && graph->GetForceFieldLeaves()[0] == field.Get());

        // The content flag on both, and on their ancestors only; the field's sphere is its node's box.
        graph->Refresh(0);
        CHECK(hasPhysics(instanceNode->GetLeafContentFlags()) && hasPhysics(fieldNode->GetLeafContentFlags()));
        CHECK(hasPhysics(branch->GetSubgraphContentFlags()) && !hasPhysics(other->GetSubgraphContentFlags()));
        CHECK(fieldNode->GetGlobalBoundingBox() == dm::box3(dm::float3(-0.5f), dm::float3(0.5f)));
        CHECK(instance->GetLocalBoundingBox().isempty());  // no asset yet

        // Clone copies the settings; an instance clone spawns its own instance (none here, without a service).
        auto instanceClone = instance->Clone();
        auto* ic = dynamic_cast<PhysInstanceLeaf*>(instanceClone.Get());
        CHECK(ic && ic->GetVelocity().y == 2.f && ic->GetPhase() == instance->GetPhase() &&
              ic->GetInvMassScale() == 0.5f && ic->GetTargetDomain() == domain.Get() && !ic->GetInstance());
        auto fieldClone = field->Clone();
        auto* fc = dynamic_cast<PhysForceFieldLeaf*>(fieldClone.Get());
        CHECK(fc && fc->GetField().radius == 0.5f && fc->GetField().strength == -2.f &&
              fc->GetField().mode == ephys::ForceMode::Impulse && fc->GetField().linearFalloff);
        instanceClone = nullptr;
        fieldClone = nullptr;

        // Detaching unregisters both; re-attaching registers them again.
        graph->Detach(branch);
        CHECK(graph->GetInstanceLeaves().empty() && graph->GetForceFieldLeaves().empty());
        graph->Attach(root, branch);
        CHECK(graph->GetInstanceLeaves().size() == 1 && graph->GetForceFieldLeaves().size() == 1);
    }
    CHECK(refCount(domain.Get()) == refs);

    // JSON, as Donut's loader does it.
    auto factory = MAKE_RC_OBJ_PTR(PhysSceneTypeFactory);
    auto graph = factory->CreateGraph();
    auto* physGraph = dynamic_cast<PhysSceneGraph*>(graph.Get());
    CHECK(physGraph != nullptr);
    if (!physGraph) return;
    graph->SetRootNode(MAKE_RC_OBJ_PTR(SceneGraphNode));
    auto instanceLeaf = factory->CreateLeaf("PhysInstance");
    auto fieldLeaf = factory->CreateLeaf("PhysForceField");
    auto* instance = dynamic_cast<PhysInstanceLeaf*>(instanceLeaf.Get());
    auto* field = dynamic_cast<PhysForceFieldLeaf*>(fieldLeaf.Get());
    CHECK(instance && field);
    if (!instance || !field) return;
    addNode(physGraph, graph->GetRootNode(), "box")->SetLeaf(instanceLeaf);
    addNode(physGraph, graph->GetRootNode(), "fan")->SetLeaf(fieldLeaf);
    instance->Load(parseJson(R"({"type": "PhysInstance", "velocity": [1, 0, -2], "group": 5, "selfCollide": true,
                                 "fluid": true, "invMassScale": 0.25, "domain": "/water"})"));
    CHECK(instance->GetVelocity().x == 1.f && instance->GetVelocity().z == -2.f);
    CHECK(instance->GetPhase() == ephys::makePhase(5, ephys::PhaseSelfCollide | ephys::PhaseFluid));
    CHECK(instance->GetInvMassScale() == 0.25f && instance->GetTargetDomainPath() == std::filesystem::path("/water"));
    instance->Load(parseJson(R"({"phase": 123})"));
    CHECK(instance->GetPhase() == 123);
    field->Load(parseJson(R"({"type": "PhysForceField", "radius": 2, "strength": -4, "mode": "velocityChange",
                              "linearFalloff": true, "domain": "../water"})"));
    CHECK(field->GetField().radius == 2.f && field->GetField().strength == -4.f);
    CHECK(field->GetField().mode == ephys::ForceMode::VelocityChange && field->GetField().linearFalloff);
    CHECK(field->GetTargetDomainPath() == std::filesystem::path("../water"));
    CHECK(physGraph->GetInstanceLeaves().size() == 1 && physGraph->GetForceFieldLeaves().size() == 1);
}

int runGraphTests() {
    donut::log::ConsoleApplicationMode();
    donut::log::SetMinSeverity(donut::log::Severity::Warning);
    donut::log::SetCallback(logCallback);
    testGraphTracking();
    testGraphFactory();
    testGraphExtensionLeaves();
    CHECK(g_LogProblems.load() == 0);
    if (g_Failures) {
        std::printf("FAILED: %d check(s)\n", g_Failures);
        return 1;
    }
    std::printf("PASSED\n");
    return 0;
}

// ---- The service on a device -----------------------------------------------------------------------------------

// The service's messages: counted, and printed.
struct ServiceMessages {
    int errors = 0, warnings = 0;
    std::string last;
    PhysSceneServiceDesc desc(Env& env) {
        PhysSceneServiceDesc d;
        d.device = env.phys;
        d.messageCallback = [this](ephys::PhysMessageSeverity severity, const char* text) {
            if (severity == ephys::PhysMessageSeverity::Info) return;
            (severity == ephys::PhysMessageSeverity::Warning ? warnings : errors)++;
            last = text;
            std::printf("  (service %s: %s)\n", severity == ephys::PhysMessageSeverity::Warning ? "warning" : "error",
                        text);
        };
        return d;
    }
};

// A granular block in the plane box of scenes::granular(): dimx^3 particles from `lower`, spacing 0.1.
scenes::Scene block(int dim, scenes::Vec3 lower) {
    scenes::Scene s = scenes::granular();
    s.positions.clear();
    s.velocities.clear();
    s.phases.clear();
    scenes::Jitter random(7);
    scenes::addParticleGrid(s, lower, dim, dim, dim, 0.1f, {0, 0, 0}, 1.0f,
                            ephys::makePhase(0, ephys::PhaseSelfCollide), 0.002f, random);
    return s;
}

// A domain loaded with a scene; the rig's own solver is dropped (the service makes one).
ephys::PhysDomainHandle loadDomain(Env& env, const scenes::Scene& scene) {
    Rig rig = loadScene(env, scene);
    CHECK(rig.domain != nullptr);
    rig.solver = nullptr;
    env.phys->flush();
    return rig.domain;
}

// FleX's quaternion rotation (x y z imaginary, w real).
dm::double3 rotate(const float q[4], const dm::double3& v) {
    const dm::double3 u(q[0], q[1], q[2]);
    return v + 2.0 * dm::cross(u, dm::cross(u, v) + double(q[3]) * v);
}

// Checks one shape of the domain's set against a transform in the domain's space: the translation, and the
// rotation through the images of the three axes (rows of the linear part, normalised).
void checkTransform(const float position[4], const float rotation[4], const dm::daffine3& expected, const char* what) {
    bool ok = true;
    for (int k = 0; k < 3; ++k) ok &= std::abs(double(position[k]) - expected.m_translation[k]) < 1e-4;
    const dm::double3 rows[3] = {expected.m_linear.row0, expected.m_linear.row1, expected.m_linear.row2};
    for (int i = 0; i < 3; ++i) {
        dm::double3 axis(0.0);
        axis[i] = 1.0;
        const dm::double3 image = rotate(rotation, axis);
        ok &= dm::length(image - dm::normalize(rows[i])) < 1e-4;
    }
    if (!ok)
        std::printf("  %s: got p (%g %g %g) q (%g %g %g %g), expected p (%g %g %g)\n", what, position[0], position[1],
                    position[2], rotation[0], rotation[1], rotation[2], rotation[3], expected.m_translation.x,
                    expected.m_translation.y, expected.m_translation.z);
    CHECK(ok);
}

void testShapeTransforms(Env& env, D3DMessages& d3d) {
    std::printf("shape transforms\n");
    ephys::PhysDomainHandle placed = loadDomain(env, block(2, {0, 1, 0}));
    ephys::PhysDomainHandle identity = loadDomain(env, block(2, {0, 1, 0}));
    if (!placed || !identity) return;
    {
        ServiceMessages messages;
        auto graph = makeGraph();
        PhysSceneService service(graph, messages.desc(env));
        SceneGraphNode* root = graph->GetRootNode();

        // Domain "placed": rotated, translated, scale 2 (rigid with a uniform scale). Domain "identity": none.
        auto placedNode = addNode(graph, root, "placed", MAKE_RC_OBJ_PTR(PhysDomainInstance, placed.Get()));
        const dm::double3 placedTranslation(3.0, -1.0, 0.5);
        const dm::dquat placedRotation = dm::rotationQuat(dm::normalize(dm::double3(1, 2, 0.5)), 0.7);
        const dm::double3 placedScale(2.0);
        placedNode->SetTransform(&placedTranslation, &placedRotation, &placedScale);
        addNode(graph, root, "identity", MAKE_RC_OBJ_PTR(PhysDomainInstance, identity.Get()));

        // An animated sphere under an animated parent, colliding with every domain; a box under a non-uniform
        // scale (a box takes one per axis), colliding with "identity" only.
        auto arm = addNode(graph, root, "arm");
        auto sphere = MAKE_RC_OBJ_PTR(PhysShapeLeaf);
        sphere->SetSphere(0.25f);
        auto sphereNode = addNode(graph, arm, "sphere", sphere);
        const dm::double3 sphereScale(1.5);
        sphereNode->SetScaling(sphereScale);
        auto box = MAKE_RC_OBJ_PTR(PhysShapeLeaf);
        box->SetBox(dm::float3(0.5f, 0.25f, 1.0f));
        box->SetTargetDomainPath("/identity");
        box->SetDynamic(true);
        auto boxNode = addNode(graph, root, "box", box);
        const dm::double3 boxScale(2.0, 3.0, 0.5);
        const dm::dquat boxRotation = dm::rotationQuat(dm::double3(0, 1, 0), 0.4);
        boxNode->SetTransform(nullptr, &boxRotation, &boxScale);

        uint64_t lastRevision = 0;
        for (uint32_t frame = 0; frame < 4; ++frame) {
            const dm::dquat armRotation = dm::rotationQuat(dm::double3(0, 0, 1), 0.3 * frame);
            const dm::double3 armTranslation(0.1 * frame, 1.0, -0.2 * frame);
            arm->SetTransform(&armTranslation, &armRotation, nullptr);
            const dm::double3 sphereTranslation(0.5, 0.05 * frame * frame, 0.0);
            sphereNode->SetTranslation(sphereTranslation);
            graph->Refresh(frame);
            CHECK(service.SyncToPhysics());
            CHECK(service.GetPrimaryInstance(placed) == placedNode->GetLeaf());
            CHECK(service.GetNumShapes(placed) == 1);
            CHECK(service.GetNumShapes(identity) == 2);

            for (int which = 0; which < 2; ++which) {
                ephys::IPhysDomain* domain = which == 0 ? placed.Get() : identity.Get();
                const dm::daffine3 sceneToSim =
                    which == 0 ? dm::inverse(placedNode->GetLocalToWorldTransform()) : dm::daffine3::identity();
                ephys::ICollisionShapeSet* set = domain->getShapes();
                const uint32_t n = set->getNumShapes();
                CHECK(n == (which == 0 ? 1u : 2u));
                if (n == 0) continue;
                auto geometry = env.fetch<ephys::CollisionGeometry>(set->getGeometry(), 0, n);
                auto positions = env.fetch<float>(set->getPositions(), 0, n);
                auto rotations = env.fetch<float>(set->getRotations(), 0, n);
                auto prevPositions = env.fetch<float>(set->getPrevPositions(), 0, n);
                auto prevRotations = env.fetch<float>(set->getPrevRotations(), 0, n);
                auto flags = env.fetch<int32_t>(set->getFlags(), 0, n);

                // The sphere is the first shape leaf. Its current and previous world transforms, in the domain's
                // space; in the first frame the previous one is the current one (the node was just attached).
                const dm::daffine3 current = sphereNode->GetLocalToWorldTransform() * sceneToSim;
                const dm::daffine3 previous =
                    (frame == 0 ? sphereNode->GetLocalToWorldTransform() : sphereNode->GetPrevLocalToWorldTransform()) *
                    sceneToSim;
                checkTransform(&positions[0], &rotations[0], current, "sphere current");
                checkTransform(&prevPositions[0], &prevRotations[0], previous, "sphere previous");
                // Scale 1.5 in the scene; in the placed domain's space 1.5 / 2.
                const float expectedRadius = 0.25f * 1.5f / (which == 0 ? 2.0f : 1.0f);
                CHECK(std::abs(geometry[0].sphere.radius - expectedRadius) < 1e-5f);
                CHECK(flags[0] == ephys::makeShapeFlags(ephys::CollisionShapeType::Sphere, false));
                if (frame > 0) {
                    // It moved: the previous transform is not the current one.
                    CHECK(std::abs(positions[0] - prevPositions[0]) + std::abs(positions[1] - prevPositions[1]) > 1e-3f);
                }

                if (which == 1 && n == 2) {
                    // The box: world transform exactly (identity placement), half extents scaled per axis.
                    checkTransform(&positions[4], &rotations[4], boxNode->GetLocalToWorldTransform(), "box current");
                    checkTransform(&prevPositions[4], &prevRotations[4], boxNode->GetLocalToWorldTransform(), "box prev");
                    CHECK(std::abs(geometry[1].box.halfExtents[0] - 1.0f) < 1e-5f);
                    CHECK(std::abs(geometry[1].box.halfExtents[1] - 0.75f) < 1e-5f);
                    CHECK(std::abs(geometry[1].box.halfExtents[2] - 0.5f) < 1e-5f);
                    CHECK(flags[1] == ephys::makeShapeFlags(ephys::CollisionShapeType::Box, true));
                    // Exactly the node's world translation, as a float.
                    const dm::double3 t = boxNode->GetLocalToWorldTransform().m_translation;
                    CHECK(positions[4] == float(t.x) && positions[5] == float(t.y) && positions[6] == float(t.z));
                }
            }
            lastRevision = identity->getShapes()->getContentRevision();
            env.device->runGarbageCollection();
        }

        // Nothing moves for two frames: the first uploads (the previous transforms catch up), the second
        // finds the arrays unchanged and uploads nothing.
        graph->Refresh(4);
        CHECK(service.SyncToPhysics());
        CHECK(identity->getShapes()->getContentRevision() == lastRevision + 1);
        graph->Refresh(5);
        CHECK(service.SyncToPhysics());
        CHECK(identity->getShapes()->getContentRevision() == lastRevision + 1);

        // A sphere under a non-uniform scale is left out, with one message.
        const dm::double3 squash(1.0, 2.0, 1.0);
        sphereNode->SetScaling(squash);
        graph->Refresh(6);
        CHECK(!service.SyncToPhysics());
        CHECK(messages.errors == 1 && messages.last.find("uniform scale") != std::string::npos);
        CHECK(service.GetNumShapes(identity) == 1 && service.GetNumShapes(placed) == 0);
        graph->Refresh(7);
        CHECK(!service.SyncToPhysics());
        CHECK(messages.errors == 1);  // once, not every frame
        sphereNode->SetScaling(sphereScale);
        graph->Refresh(8);
        CHECK(service.SyncToPhysics());
        CHECK(service.GetNumShapes(identity) == 2);

        CHECK(messages.errors == 1 && messages.warnings == 0);
        CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
        env.device->runGarbageCollection();
    }
    CHECK(refCount(placed.Get()) == 1);
    CHECK(refCount(identity.Get()) == 1);
    CHECK(d3d.problems() == 0);
}

// One frame of the service on the device's own command list.
void frame(Env& env, PhysSceneGraph* graph, PhysSceneService& service, uint32_t index, bool step = true) {
    graph->Refresh(index);
    CHECK(service.SyncToPhysics());
    if (step) CHECK(service.Step(1.0f / 60.0f, 2));
    service.SyncFromPhysics();
    env.device->runGarbageCollection();
}

void testCollision(Env& env, D3DMessages& d3d) {
    std::printf("collision with an animated shape\n");
    // Two identical blocks, 6 x 6 x 6 particles from y = 1.2, in the plane box of the granular scene. "caught"
    // has a box shape below it that rises and turns; "free" has none and falls to the floor.
    ephys::PhysDomainHandle caught = loadDomain(env, block(6, {-0.25f, 1.2f, -0.25f}));
    ephys::PhysDomainHandle free = loadDomain(env, block(6, {-0.25f, 1.2f, -0.25f}));
    if (!caught || !free) return;
    const uint32_t n = caught->getParticles()->getActiveCount();
    {
        ServiceMessages messages;
        auto graph = makeGraph();
        PhysSceneService service(graph, messages.desc(env));
        SceneGraphNode* root = graph->GetRootNode();
        addNode(graph, root, "caught", MAKE_RC_OBJ_PTR(PhysDomainInstance, caught.Get()));
        addNode(graph, root, "free", MAKE_RC_OBJ_PTR(PhysDomainInstance, free.Get()));

        auto box = MAKE_RC_OBJ_PTR(PhysShapeLeaf);
        box->SetBox(dm::float3(0.8f, 0.1f, 0.8f));
        box->SetTargetDomain(caught);
        auto boxNode = addNode(graph, root, "platform", box);

        const uint32_t frames = 120;
        auto boxHeight = [](uint32_t f) { return 0.4 + 0.005 * f; };  // 0.3 m/s upwards
        for (uint32_t f = 0; f < frames; ++f) {
            const dm::double3 translation(0.0, boxHeight(f), 0.0);
            const dm::dquat rotation = dm::rotationQuat(dm::double3(0, 1, 0), 0.01 * f);
            boxNode->SetTransform(&translation, &rotation, nullptr);
            frame(env, graph, service, f);
        }
        CHECK(service.GetNumShapes(caught) == 1 && service.GetNumShapes(free) == 0);

        // The particles in the box's footprint (in its local space, less a particle radius at the rim) and
        // those beside it; particles that slid off the rim of the platform fall to the floor, which is fine.
        const float boxTop = float(boxHeight(frames - 1)) + 0.1f;
        const dm::daffine3 worldToBox = dm::inverse(boxNode->GetLocalToWorldTransform());
        struct Stats {
            uint32_t onFootprint = 0, belowTop = 0;
            float minFootprintY = 1e9f, minY = 1e9f, meanY = 0.f, meanFootprintVy = 0.f;
        };
        auto stats = [&](ephys::IPhysDomain* domain) {
            Stats s;
            auto p = env.fetch<float>(domain->getParticles()->getArray(ParticleArray::Position), 0, n);
            auto v = env.fetch<float>(domain->getParticles()->getArray(ParticleArray::Velocity), 0, n);
            if (p.size() < 4 * n || v.size() < 3 * n) return s;
            for (uint32_t i = 0; i < n; ++i) {
                const float y = p[4 * i + 1];
                s.minY = std::min(s.minY, y);
                s.meanY += y / float(n);
                const dm::double3 local = worldToBox.transformPoint(dm::double3(p[4 * i], y, p[4 * i + 2]));
                if (std::abs(local.x) > 0.7 || std::abs(local.z) > 0.7) continue;
                ++s.onFootprint;
                s.minFootprintY = std::min(s.minFootprintY, y);
                s.meanFootprintVy += v[3 * i + 1];
                if (y < boxTop - 0.02f) ++s.belowTop;
            }
            if (s.onFootprint) s.meanFootprintVy /= float(s.onFootprint);
            return s;
        };
        const Stats c = stats(caught), f = stats(free);
        std::printf("  box top %.3f; caught: %u of %u over the box, min y there %.3f, mean vy there %.3f, mean y %.3f; "
                    "free: min y %.3f, mean y %.3f\n",
                    boxTop, c.onFootprint, n, c.minFootprintY, c.meanFootprintVy, c.meanY, f.minY, f.meanY);
        // On the box, not through it: every particle over the footprint is above the box top (less a little
        // penetration), so it was carried 0.6 m up with the box; most of the block is there. The free block
        // lies on the floor.
        CHECK(c.onFootprint > n * 3 / 4);
        CHECK(c.belowTop == 0 && c.minFootprintY > boxTop - 0.02f);
        CHECK(c.meanFootprintVy > 0.05f);  // moving up with the box (the solver's velocity damps contacts)
        CHECK(f.minY < 0.1f && f.meanY < 0.2f);
        CHECK(c.meanY > f.meanY + 0.8f);
        CHECK(messages.errors == 0 && messages.warnings == 0);
        CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
        env.device->runGarbageCollection();
    }
    CHECK(refCount(caught.Get()) == 1);
    CHECK(refCount(free.Get()) == 1);
    CHECK(d3d.problems() == 0);
}

void testPlacementRejected(Env& env, D3DMessages& d3d) {
    std::printf("non-uniform placement\n");
    ephys::PhysDomainHandle domain = loadDomain(env, block(3, {0, 1, 0}));
    if (!domain) return;
    {
        ServiceMessages messages;
        auto graph = makeGraph();
        PhysSceneService service(graph, messages.desc(env));
        auto parent = addNode(graph, graph->GetRootNode(), "stretched");
        const dm::double3 stretch(1.0, 2.0, 1.0);
        parent->SetScaling(stretch);
        auto instance = MAKE_RC_OBJ_PTR(PhysDomainInstance, domain.Get());
        addNode(graph, parent, "fluid", instance);

        graph->Refresh(0);
        CHECK(!service.SyncToPhysics());
        CHECK(messages.errors == 1);
        CHECK(messages.last.find("non-uniform scale") != std::string::npos);
        CHECK(messages.last.find("/root/stretched/fluid") != std::string::npos);
        CHECK(instance->GetPlacement() == PhysPlacement::Rejected);
        CHECK(service.GetPrimaryInstance(domain) == nullptr);
        CHECK(service.Step(1.0f / 60.0f, 2));  // nothing to step: not a failure
        // The solver exists (it was created when the domain joined the graph) but does not step.
        CHECK(service.GetSolver(domain) && service.GetSolver(domain)->getSubstepCount() == 0);
        service.SyncFromPhysics();
        graph->Refresh(1);
        CHECK(!service.SyncToPhysics());
        CHECK(messages.errors == 1);  // reported once

        // A shear (a rotated child under the stretched parent) is rejected too; a mirror as well.
        const dm::double3 uniform(2.0);
        parent->SetScaling(uniform);
        const dm::double3 mirror(-1.0, 1.0, 1.0);
        auto mirrored = MAKE_RC_OBJ_PTR(PhysDomainInstance, domain.Get());
        auto mirroredNode = addNode(graph, graph->GetRootNode(), "mirrored", mirrored);
        mirroredNode->SetScaling(mirror);

        // Uniform now: accepted, solved, stepped. The mirrored instance is still rejected, with its own message.
        graph->Refresh(2);
        CHECK(!service.SyncToPhysics());
        CHECK(messages.errors == 2 && messages.last.find("mirroring (a negative scale)") != std::string::npos);
        CHECK(instance->GetPlacement() == PhysPlacement::Valid);
        CHECK(mirrored->GetPlacement() == PhysPlacement::Rejected);
        CHECK(service.GetPrimaryInstance(domain) == instance.Get());
        CHECK(service.GetSolver(domain) != nullptr);
        CHECK(service.Step(1.0f / 60.0f, 2));
        service.SyncFromPhysics();
        CHECK(service.GetSolver(domain) && service.GetSolver(domain)->getSubstepCount() == 2);

        CHECK(messages.warnings == 0);
        CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
        env.device->runGarbageCollection();
    }
    CHECK(refCount(domain.Get()) == 1);
    CHECK(d3d.problems() == 0);
}

dm::box3 fetchBounds(Env& env, ephys::IPhysDomain* domain) {
    auto b = env.fetch<float>(domain->getParticles()->getBounds(), 0, 2);
    if (b.size() < 6) return dm::box3::empty();
    return dm::box3(dm::float3(b[0], b[1], b[2]), dm::float3(b[3], b[4], b[5]));
}

void testBounds(Env& env, D3DMessages& d3d) {
    std::printf("bounds\n");
    ephys::PhysDomainHandle domain = loadDomain(env, block(4, {-0.2f, 1.0f, -0.2f}));
    if (!domain) return;
    {
        ServiceMessages messages;
        auto graph = makeGraph();
        PhysSceneService service(graph, messages.desc(env));
        auto instance = MAKE_RC_OBJ_PTR(PhysDomainInstance, domain.Get());
        auto node = addNode(graph, graph->GetRootNode(), "fluid", instance);
        const dm::double3 translation(3.0, 0.0, 0.0);
        const dm::double3 scale(2.0);
        node->SetTransform(&translation, nullptr, &scale);

        // Stepping: the bounds arrive some frames later, never blocking. Record the lag.
        uint32_t firstSeen = 0, maxLag = 0;
        const uint32_t frames = 30;
        for (uint32_t f = 0; f < frames; ++f) {
            frame(env, graph, service, f);
            const uint64_t boundsFrame = service.GetBoundsFrame(domain);
            if (boundsFrame) {
                if (!firstSeen) firstSeen = f + 1;
                maxLag = std::max(maxLag, uint32_t(service.GetFrame() - boundsFrame));
                CHECK(!instance->GetLocalBoundingBox().isempty());
            } else {
                CHECK(instance->GetLocalBoundingBox().isempty());
            }
        }
        std::printf("  first bounds at frame %u, lag up to %u frame(s), copies skipped %llu\n", firstSeen, maxLag,
                    (unsigned long long)service.GetBoundsCopiesSkipped());
        CHECK(firstSeen >= 2);  // the copy of frame N is polled from frame N + 1 on
        CHECK(service.GetBoundsFrame(domain) > 0);

        // Stop stepping and let the GPU finish: the bounds catch up with the last step exactly (a frame whose
        // readback buffers were all in flight left its copy to a later frame: at most two more).
        uint32_t extra = 0;
        for (; extra < 3 && service.GetBoundsFrame(domain) != frames; ++extra) {
            CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
            frame(env, graph, service, frames + extra, false);
        }
        std::printf("  caught up after %u frame(s) without a step\n", extra);
        CHECK(service.GetBoundsFrame(domain) == frames);
        const dm::box3 gpu = fetchBounds(env, domain);
        CHECK(instance->GetLocalBoundingBox() == gpu);
        CHECK(!gpu.isempty());
        // The box is in the domain's space: the particles fell, within the plane box of the scene.
        CHECK(gpu.m_mins.y > -0.2f && gpu.m_maxs.y < 1.6f);

        // Donut puts it in the scene: the node's global box is the local box placed by the node transform.
        graph->Refresh(frames + 10);
        const dm::box3 global = node->GetGlobalBoundingBox();
        const dm::box3 expected = gpu * node->GetLocalToWorldTransformFloat();
        CHECK(dm::all(dm::abs(global.m_mins - expected.m_mins) < 1e-5f) &&
              dm::all(dm::abs(global.m_maxs - expected.m_maxs) < 1e-5f));
        CHECK(dm::all(dm::abs(global.m_mins - (gpu.m_mins * 2.f + dm::float3(3, 0, 0))) < 1e-5f));
        CHECK(hasPhysics(graph->GetRootNode()->GetSubgraphContentFlags()));
        // The root's box includes it.
        CHECK(graph->GetRootNode()->GetGlobalBoundingBox().contains(global));

        CHECK(messages.errors == 0 && messages.warnings == 0);
        CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
        env.device->runGarbageCollection();
    }
    CHECK(refCount(domain.Get()) == 1);
    CHECK(d3d.problems() == 0);
}

void testApplicationCommandList(Env& env, D3DMessages& d3d) {
    std::printf("application command list\n");
    ephys::PhysDomainHandle domain = loadDomain(env, block(4, {-0.2f, 1.0f, -0.2f}));
    if (!domain) return;
    const uint32_t n = domain->getParticles()->getActiveCount();
    const std::vector<float> start = readPositions(env, Rig{domain, nullptr}, n);
    {
        ServiceMessages messages;
        auto graph = makeGraph();
        PhysSceneService service(graph, messages.desc(env));
        auto instance = MAKE_RC_OBJ_PTR(PhysDomainInstance, domain.Get());
        addNode(graph, graph->GetRootNode(), "fluid", instance);
        auto shape = MAKE_RC_OBJ_PTR(PhysShapeLeaf);
        shape->SetSphere(0.2f);
        auto shapeNode = addNode(graph, graph->GetRootNode(), "ball", shape);

        nvrhi::CommandListHandle commandList;
        nvrhi::CommandListParameters listParams;
        listParams.queueType = nvrhi::CommandQueue::Graphics;
        CHECK(env.device->createCommandList(listParams, &commandList) == nvrhi::FS_OK);
        if (!commandList) return;

        const uint32_t frames = 6;
        for (uint32_t f = 0; f < frames; ++f) {
            const dm::double3 translation(1.0, 0.5 + 0.05 * f, 0.0);
            shapeNode->SetTranslation(translation);
            graph->Refresh(f);
            env.phys->flush();  // D3D11: one open command list at a time
            commandList->open();
            CHECK(service.SyncToPhysics(commandList));
            CHECK(service.Step(1.0f / 60.0f, 2, commandList));
            CHECK(service.SyncFromPhysics(commandList) == 0);
            commandList->close();
            nvrhi::ICommandList* list = commandList;
            env.device->executeCommandLists(&list, 1, nvrhi::CommandQueue::Graphics);
            service.NotifySubmitted(nvrhi::CommandQueue::Graphics);
            env.device->runGarbageCollection();
        }
        ephys::IPhysSolver* solver = service.GetSolver(domain);
        CHECK(solver && solver->getSubstepCount() == 2 * frames);
        CHECK(service.GetNumShapes(domain) == 1);

        // The particles fell; the shape set holds the ball where its node is.
        const std::vector<float> now = readPositions(env, Rig{domain, nullptr}, n);
        CHECK(now.size() == start.size() && !now.empty() && now[1] < start[1] - 0.01f);
        auto positions = env.fetch<float>(domain->getShapes()->getPositions(), 0, 1);
        CHECK(positions.size() == 4 && std::abs(positions[1] - float(0.5 + 0.05 * (frames - 1))) < 1e-5f);

        // The bounds of the last step arrive once the GPU is done, through the application's list as well.
        for (uint32_t extra = 0; extra < 3 && service.GetBoundsFrame(domain) != frames; ++extra) {
            CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
            graph->Refresh(frames + extra);
            commandList->open();
            CHECK(service.SyncToPhysics(commandList));
            service.SyncFromPhysics(commandList);
            commandList->close();
            nvrhi::ICommandList* list = commandList;
            env.device->executeCommandLists(&list, 1, nvrhi::CommandQueue::Graphics);
            service.NotifySubmitted(nvrhi::CommandQueue::Graphics);
        }
        CHECK(service.GetBoundsFrame(domain) == frames);
        CHECK(instance->GetLocalBoundingBox() == fetchBounds(env, domain));

        // And a frame on the device's own list after that.
        frame(env, graph, service, frames + 3);
        CHECK(solver && solver->getSubstepCount() == 2 * frames + 2);

        CHECK(messages.errors == 0 && messages.warnings == 0);
        CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
        env.device->runGarbageCollection();
    }
    CHECK(refCount(domain.Get()) == 1);
    CHECK(d3d.problems() == 0);
}

void testJson(Env& env, D3DMessages& d3d) {
    std::printf("JSON domains and shapes\n");
    {
        ServiceMessages messages;
        auto factory = MAKE_RC_OBJ_PTR(PhysSceneTypeFactory, env.phys);
        auto base = factory->CreateGraph();
        nvrhi::AutoPtr<PhysSceneGraph> graph = dynamic_cast<PhysSceneGraph*>(base.Get());
        CHECK(graph != nullptr);
        if (!graph) return;
        graph->SetRootNode(MAKE_RC_OBJ_PTR(SceneGraphNode));
        DomainEvents events;
        events.attach(graph);
        PhysSceneService service(graph, messages.desc(env));

        // The loader's order: the shape first (its domain path does not resolve yet), SetLeaf, then Load.
        auto shapeNode = addNode(graph, graph->GetRootNode(), "ground");
        auto shapeLeaf = factory->CreateLeaf("PhysShape");
        shapeNode->SetLeaf(shapeLeaf);
        shapeLeaf->Load(parseJson(R"({"shape": "sphere", "radius": 0.3, "domain": "../fluid"})"));

        auto domainNode = addNode(graph, graph->GetRootNode(), "fluid");
        auto leaf = factory->CreateLeaf("PhysDomain");
        domainNode->SetLeaf(leaf);
        CHECK(graph->GetDomains().empty());  // registered without a domain
        leaf->Load(parseJson(R"({"type": "PhysDomain", "maxParticles": 128, "solver": "xpbd", "radius": 0.05,
                                 "gravity": [0, -5, 0], "numIterations": 4, "dynamicFriction": 0.25,
                                 "planes": [[0, 1, 0, 0]]})"));
        auto* instance = dynamic_cast<PhysDomainInstance*>(leaf.Get());
        ephys::IPhysDomain* domain = instance ? instance->GetDomain() : nullptr;
        CHECK(domain != nullptr);
        if (!domain) return;
        // Load gave it a domain and the tracker followed (the service's callback chained to the test's).
        CHECK(graph->GetDomains().size() == 1 && events.added.size() == 1 && events.added[0] == domain);
        CHECK(domain->getDesc().maxParticles == 128);
        CHECK(domain->getParams().radius == 0.05f && domain->getParams().gravity[1] == -5.f);
        CHECK(domain->getParams().numIterations == 4 && domain->getParams().dynamicFriction == 0.25f);
        CHECK(domain->getParams().numPlanes == 1 && domain->getParams().planes[0][1] == 1.f);

        graph->Refresh(0);
        CHECK(service.SyncToPhysics());
        ephys::IPhysSolver* solver = service.GetSolver(domain);
        CHECK(solver != nullptr);
        ephys::PhysSolverXPBDHandle xpbd;
        CHECK(solver && solver->QueryInterface(NVRHI_IID_PPV_ARGS(&xpbd)) == nvrhi::FS_OK);
        CHECK(service.GetNumShapes(domain) == 1);  // "../fluid" resolved relative to the shape's node

        // Removing the domain node: the service drops its solver.
        graph->Detach(domainNode);
        CHECK(events.removed.size() == 1);
        CHECK(service.GetSolver(domain) == nullptr);
        graph->Refresh(1);
        CHECK(!service.SyncToPhysics());  // the shape's path names nothing now
        CHECK(messages.errors == 1 && messages.last.find("domain path") != std::string::npos);
        CHECK(messages.warnings == 0);
        CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
        env.device->runGarbageCollection();
    }
    CHECK(d3d.problems() == 0);
}

// ---- Asset instances and force fields ---------------------------------------------------------------------------

// A rigid cube asset of half size 0.2 at spacing 0.1 (64 particles), centred at the origin.
ephys::PhysAssetHandle cubeAsset(Env& env) {
    ephys::PhysAssetBuilderHandle builder;
    CHECK(env.phys->createAssetBuilder(&builder) == nvrhi::FS_OK);
    if (!builder) return nullptr;
    std::vector<float> vertices;
    for (int i = 0; i < 8; ++i) vertices.insert(vertices.end(), {(i & 1) ? 0.2f : -0.2f, (i & 2) ? 0.2f : -0.2f, (i & 4) ? 0.2f : -0.2f});
    const int32_t indices[36] = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4,
                                 2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
    ephys::PhysAssetHandle asset;
    CHECK(builder->createRigidFromMesh(vertices.data(), 8, indices, 36, 0.1f, 0.0f, &asset) == nvrhi::FS_OK);
    return asset;
}

// A domain without gravity or planes, its particles to be filled by a container.
ephys::PhysDomainHandle emptyDomain(Env& env, uint32_t maxParticles) {
    ephys::DomainDesc desc;
    desc.maxParticles = maxParticles;
    ephys::PhysDomainHandle domain;
    CHECK(env.phys->createDomain(desc, &domain) == nvrhi::FS_OK);
    if (!domain) return domain;
    ephys::PhysParams params;
    params.radius = 0.1f;
    params.numIterations = 3;
    scenes::finishParams(params);
    domain->setParams(params);
    env.phys->flush();
    return domain;
}

// The largest distance of the instance's particles (in the domain) from the asset's particles moved by `offset`.
double spawnError(Env& env, ephys::IPhysDomain* domain, ephys::IPhysInstance* instance, ephys::IPhysAsset* asset,
                  const dm::float3& offset) {
    const std::vector<float> p = env.fetch<float>(domain->getParticles()->getArray(ParticleArray::Position), 0,
                                                  domain->getDesc().maxParticles);
    double worst = 0.0;
    for (uint32_t i = 0; i < instance->getParticleCount(); ++i) {
        const size_t k = size_t(instance->getParticleIndices()[i]);
        for (int j = 0; j < 3; ++j)
            worst = std::max(worst, std::fabs(double(p[k * 4 + j]) - (asset->getParticles()[i * 4 + j] + offset[j])));
    }
    return worst;
}

void testInstanceLeaves(Env& env, D3DMessages& d3d) {
    std::printf("asset instances\n");
    ephys::PhysAssetHandle cube = cubeAsset(env);
    ephys::PhysDomainHandle domain = emptyDomain(env, 256);
    if (!cube || !domain) return;
    const uint32_t n = cube->getParticleCount();
    {
        ServiceMessages messages;
        auto graph = makeGraph();
        PhysSceneService service(graph, messages.desc(env));
        SceneGraphNode* root = graph->GetRootNode();

        // The domain placed with a rotation and a scale of 2; the instance's node is a child of the domain's, so
        // its local transform is its transform in the domain's simulation space.
        auto domainNode = addNode(graph, root, "domain", MAKE_RC_OBJ_PTR(PhysDomainInstance, domain.Get()));
        const dm::double3 placement(0.0, 0.5, 0.0), scale2(2.0);
        const dm::dquat turn = dm::rotationQuat(dm::double3(0, 1, 0), 0.3);
        domainNode->SetTransform(&placement, &turn, &scale2);
        auto leaf = MAKE_RC_OBJ_PTR(PhysInstanceLeaf, cube.Get());
        leaf->SetPhase(ephys::makePhase(1, 0));  // no target: the only domain of the graph
        auto node = addNode(graph, domainNode, "box", leaf);
        const dm::double3 spawnAt(0.2, 0.3, 0.0);
        node->SetTransform(&spawnAt, nullptr, nullptr);

        frame(env, graph, service, 0);
        ephys::IPhysContainer* container = service.GetContainer(domain);
        CHECK(container && leaf->GetInstance() && leaf->GetInstance()->getContainer() == container);
        if (!container || !leaf->GetInstance()) return;
        CHECK(container->getActiveCount() == n && domain->getParticles()->getActiveCount() == n);
        const double spawned = spawnError(env, domain, leaf->GetInstance(), cube, dm::float3(0.2f, 0.3f, 0.0f));

        // Moving the node later moves nothing, and the service does not write the node.
        const dm::double3 elsewhere(1.2, 0.3, 0.0);
        node->SetTransform(&elsewhere, nullptr, nullptr);
        for (uint32_t f = 1; f < 6; ++f) frame(env, graph, service, f);
        const double stayed = spawnError(env, domain, leaf->GetInstance(), cube, dm::float3(0.2f, 0.3f, 0.0f));
        CHECK(node->GetTranslation().x == 1.2);

        // Respawn: at the node's transform then; the old particles are given back.
        leaf->Respawn();
        frame(env, graph, service, 6);
        CHECK(leaf->GetInstance() != nullptr && container->getActiveCount() == n);
        const double respawned =
            leaf->GetInstance() ? spawnError(env, domain, leaf->GetInstance(), cube, dm::float3(1.2f, 0.3f, 0.0f)) : 1e9;
        std::printf("  spawned within %.2e of the node's transform, %.2e after the node moved away, respawned within "
                    "%.2e of its new transform\n",
                    spawned, stayed, respawned);
        CHECK(spawned < 1e-5 && stayed < 1e-5 && respawned < 1e-5);

        // A node with a scale in the domain's space is not a spawn transform: rejected once.
        auto scaled = MAKE_RC_OBJ_PTR(PhysInstanceLeaf, cube.Get());
        auto scaledNode = addNode(graph, domainNode, "scaled", scaled);
        const dm::double3 scale15(1.5);
        scaledNode->SetTransform(nullptr, nullptr, &scale15);
        graph->Refresh(7);
        CHECK(!service.SyncToPhysics());
        CHECK(messages.errors == 1 && messages.last.find("not rigid") != std::string::npos);
        CHECK(!scaled->GetInstance());
        graph->Refresh(8);
        CHECK(!service.SyncToPhysics());
        CHECK(messages.errors == 1);  // reported once
        graph->Detach(scaledNode);

        // Out of the graph, out of the simulation.
        graph->Detach(node);
        CHECK(!leaf->GetInstance() && container->getActiveCount() == 0);
        frame(env, graph, service, 9);
        CHECK(domain->getParticles()->getActiveCount() == 0);
        CHECK(messages.warnings == 0);
        CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
        env.device->runGarbageCollection();
    }
    CHECK(refCount(domain.Get()) == 1);
    CHECK(refCount(cube.Get()) == 1);
    CHECK(d3d.problems() == 0);
}

void testForceFieldLeaves(Env& env, D3DMessages& d3d) {
    std::printf("force fields\n");
    ephys::PhysAssetHandle cube = cubeAsset(env);
    ephys::PhysDomainHandle domain = emptyDomain(env, 256);
    if (!cube || !domain) return;
    {
        ServiceMessages messages;
        auto graph = makeGraph();
        PhysSceneService service(graph, messages.desc(env));
        SceneGraphNode* root = graph->GetRootNode();
        auto domainNode = addNode(graph, root, "domain", MAKE_RC_OBJ_PTR(PhysDomainInstance, domain.Get()));
        const dm::double3 placement(0.0, 1.0, 0.0);
        const dm::dquat turn = dm::rotationQuat(dm::double3(0, 1, 0), 0.5);
        domainNode->SetTransform(&placement, &turn, nullptr);
        // A cube at the origin of the domain, and a field beside it in the scene, not under the domain's node.
        auto instance = MAKE_RC_OBJ_PTR(PhysInstanceLeaf, cube.Get());
        instance->SetPhase(ephys::makePhase(1, 0));
        addNode(graph, domainNode, "box", instance);
        auto field = MAKE_RC_OBJ_PTR(PhysForceFieldLeaf);
        field->SetRadius(3.0f);
        field->SetStrength(2.0f);
        field->SetMode(ephys::ForceMode::VelocityChange);
        auto fieldNode = addNode(graph, root, "fan", field);

        double worst = 0.0;
        for (uint32_t f = 0; f < 8; ++f) {
            const dm::double3 at(std::sin(0.4 * f), -0.2 + 0.05 * f, 0.3);  // animated in scene space
            fieldNode->SetTransform(&at, nullptr, nullptr);
            frame(env, graph, service, f);
            ephys::IForceFieldSet* set = service.GetForceFieldSet(domain);
            CHECK(set && set->getForceFieldCount() == 1);
            if (!set || set->getForceFieldCount() != 1) return;
            const dm::daffine3 sceneToSim = dm::inverse(domainNode->GetLocalToWorldTransform());  // after Refresh
            const dm::double3 expected = (dm::translation(at) * sceneToSim).m_translation;
            for (int k = 0; k < 3; ++k) worst = std::max(worst, std::fabs(set->getForceFields()[0].position[k] - expected[k]));
            CHECK(set->getForceFields()[0].radius == 3.0f && set->getForceFields()[0].mode == ephys::ForceMode::VelocityChange);
        }
        std::printf("  the field followed its node within %.2e (in the domain's space)\n", worst);
        CHECK(worst < 1e-5);

        // It acts: the cube was pushed away from the field, which sat below it in the domain's space.
        const std::vector<float> v =
            env.fetch<float>(domain->getParticles()->getArray(ParticleArray::Velocity), 0, domain->getDesc().maxParticles);
        double vy = 0.0;
        ephys::IPhysInstance* inst = instance->GetInstance();
        CHECK(inst != nullptr);
        if (inst) {
            for (uint32_t i = 0; i < inst->getParticleCount(); ++i) vy += v[size_t(inst->getParticleIndices()[i]) * 3 + 1];
            vy /= inst->getParticleCount();
        }
        std::printf("  the cube's mean vertical velocity: %.3f\n", vy);
        CHECK(vy > 0.5);

        // A field whose domain path names nothing is left out, once.
        auto lost = MAKE_RC_OBJ_PTR(PhysForceFieldLeaf);
        lost->SetTargetDomainPath("/nowhere");
        addNode(graph, root, "lost", lost);
        graph->Refresh(8);
        CHECK(!service.SyncToPhysics());
        CHECK(messages.errors == 1 && messages.last.find("domain path") != std::string::npos);
        CHECK(service.GetForceFieldSet(domain)->getForceFieldCount() == 1);
        CHECK(messages.warnings == 0);
        CHECK(env.phys->waitForIdle() == nvrhi::FS_OK);
        env.device->runGarbageCollection();
    }
    CHECK(refCount(domain.Get()) == 1);
    CHECK(refCount(cube.Get()) == 1);
    CHECK(d3d.problems() == 0);
}

}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc >= 2 && std::strcmp(argv[1], "graph") == 0) return runGraphTests();

    Options options;
    parse(argc, argv, options);
    if (!options.rest.empty()) {
        std::printf("usage: %s graph | <d3d11|d3d12|vulkan> [--adapter <n>] [--shaders <dir>]\n", argv[0]);
        return 1;
    }
    return runWithDevice(options, [&](Env& env, D3DMessages& d3d) {
        testShapeTransforms(env, d3d);
        testCollision(env, d3d);
        testPlacementRejected(env, d3d);
        testBounds(env, d3d);
        testApplicationCommandList(env, d3d);
        testJson(env, d3d);
        testInstanceLeaves(env, d3d);
        testForceFieldLeaves(env, d3d);
    });
}
