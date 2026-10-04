// Physics leaves of Donut's scene graph (scene_graph.h).

#include <ethereal/phys_scene/scene_graph.h>

#include "scene_util.h"

#include <donut/core/json.h>
#include <donut/core/log.h>

#include <algorithm>
#include <cstddef>
#include <cstring>

namespace ephys::scene {

using donut::engine::SceneGraphLeaf;
using donut::engine::SceneGraphNode;

// ---- PhysDomainInstance ----------------------------------------------------------------------------------

PhysDomainInstance::PhysDomainInstance(IPhysDomain* domain, const SolverDesc& solverDesc)
    : m_Domain(domain), m_SolverDesc(solverDesc) {}

void PhysDomainInstance::SetDomain(IPhysDomain* domain) {
    if (domain == m_Domain.Get()) return;
    nvrhi::AutoPtr<IPhysDomain> previous = m_Domain;
    m_Domain = domain;
    m_Bounds = dm::box3::empty();
    m_Placement = PhysPlacement::Unchecked;

    // Registered with a graph exactly when the node is in one (Donut registers a leaf in SetLeaf and Attach,
    // and unregisters it in SetLeaf and Detach).
    SceneGraphNode* node = GetNode();
    nvrhi::AutoPtr<donut::engine::SceneGraph> graph = node ? node->GetGraph() : nullptr;
    if (auto* physGraph = dynamic_cast<PhysSceneGraph*>(graph.Get())) {
        physGraph->addDomainRef(domain);  // first, so that a domain shared by both ends never leaves
        physGraph->releaseDomainRef(previous);
    }
    if (node) detail::refreshNodeBounds(node);
}

nvrhi::AutoPtr<SceneGraphLeaf> PhysDomainInstance::Clone() {
    auto copy = MAKE_RC_OBJ_PTR(PhysDomainInstance, m_Domain.Get(), m_SolverDesc);  // shares the domain
    copy->m_LoadDevice = m_LoadDevice;
    copy->m_Bounds = m_Bounds;
    return copy;
}

dm::daffine3 PhysDomainInstance::GetDomainToScene() const {
    SceneGraphNode* node = GetNode();
    return node ? node->GetLocalToWorldTransform() : dm::daffine3::identity();
}

namespace {

// The float members of PhysParams a scene file may set, by name.
struct ParamField {
    const char* name;
    size_t offset;
};

#define EPHYS_PARAM(field) {#field, offsetof(PhysParams, field)}
const ParamField c_ParamFields[] = {
    EPHYS_PARAM(radius),
    EPHYS_PARAM(solidRestDistance),
    EPHYS_PARAM(fluidRestDistance),
    EPHYS_PARAM(dynamicFriction),
    EPHYS_PARAM(staticFriction),
    EPHYS_PARAM(particleFriction),
    EPHYS_PARAM(restitution),
    EPHYS_PARAM(adhesion),
    EPHYS_PARAM(sleepThreshold),
    EPHYS_PARAM(maxSpeed),
    EPHYS_PARAM(maxAcceleration),
    EPHYS_PARAM(shockPropagation),
    EPHYS_PARAM(dissipation),
    EPHYS_PARAM(damping),
    EPHYS_PARAM(drag),
    EPHYS_PARAM(lift),
    EPHYS_PARAM(cohesion),
    EPHYS_PARAM(surfaceTension),
    EPHYS_PARAM(viscosity),
    EPHYS_PARAM(vorticityConfinement),
    EPHYS_PARAM(anisotropyScale),
    EPHYS_PARAM(anisotropyMin),
    EPHYS_PARAM(anisotropyMax),
    EPHYS_PARAM(smoothing),
    EPHYS_PARAM(solidPressure),
    EPHYS_PARAM(freeSurfaceDrag),
    EPHYS_PARAM(buoyancy),
    EPHYS_PARAM(diffuseThreshold),
    EPHYS_PARAM(diffuseBuoyancy),
    EPHYS_PARAM(diffuseDrag),
    EPHYS_PARAM(diffuseLifetime),
    EPHYS_PARAM(collisionDistance),
    EPHYS_PARAM(particleCollisionMargin),
    EPHYS_PARAM(shapeCollisionMargin),
    EPHYS_PARAM(relaxationFactor),
};
#undef EPHYS_PARAM

void readParams(const Json::Value& node, PhysParams& params) {
    for (const ParamField& field : c_ParamFields) {
        const Json::Value& value = node[field.name];
        if (value.isNumeric()) {
            const float f = value.asFloat();
            std::memcpy(reinterpret_cast<uint8_t*>(&params) + field.offset, &f, sizeof(f));
        }
    }
    node["numIterations"] >> params.numIterations;
    node["diffuseBallistic"] >> params.diffuseBallistic;
    dm::float3 vec(params.gravity[0], params.gravity[1], params.gravity[2]);
    node["gravity"] >> vec;
    params.gravity[0] = vec.x, params.gravity[1] = vec.y, params.gravity[2] = vec.z;
    vec = dm::float3(params.wind[0], params.wind[1], params.wind[2]);
    node["wind"] >> vec;
    params.wind[0] = vec.x, params.wind[1] = vec.y, params.wind[2] = vec.z;
    const Json::Value& relaxation = node["relaxationMode"];
    if (relaxation.isString())
        params.relaxationMode = relaxation.asString() == "global" ? RelaxationMode::Global : RelaxationMode::Local;
    const Json::Value& planes = node["planes"];
    if (planes.isArray()) {
        const int count = std::min(int(planes.size()), 8);
        for (int i = 0; i < count; ++i) {
            dm::float4 plane = dm::float4::zero();
            planes[i] >> plane;
            params.planes[i][0] = plane.x, params.planes[i][1] = plane.y;
            params.planes[i][2] = plane.z, params.planes[i][3] = plane.w;
        }
        params.numPlanes = count;
    }
}

}  // namespace

void PhysDomainInstance::Load(const Json::Value& node) {
    const Json::Value& solver = node["solver"];
    if (solver.isString()) m_SolverDesc.mode = solver.asString() == "xpbd" ? SolverMode::XPBD : SolverMode::PBD;
    node["maxNeighborsPerParticle"] >> m_SolverDesc.maxNeighborsPerParticle;
    node["maxContactsPerParticle"] >> m_SolverDesc.maxContactsPerParticle;

    if (!m_Domain) {
        if (!m_LoadDevice) {
            donut::log::error("PhysDomain '%s': no physics device to create the domain on (create the leaf with "
                              "an ephys::scene::PhysSceneTypeFactory that has one)",
                              GetNode() ? GetNode()->GetName().c_str() : "");
            return;
        }
        DomainDesc desc;
        node["maxParticles"] >> desc.maxParticles;
        node["maxDiffuseParticles"] >> desc.maxDiffuseParticles;
        nvrhi::AutoPtr<IPhysDomain> domain;
        if (NVRHI_FAILED(m_LoadDevice->createDomain(desc, &domain))) {
            donut::log::error("PhysDomain '%s': the domain cannot be created",
                              GetNode() ? GetNode()->GetName().c_str() : "");
            return;
        }
        PhysParams params = domain->getParams();
        readParams(node, params);
        domain->setParams(params);
        SetDomain(domain);
        return;
    }

    PhysParams params = m_Domain->getParams();
    readParams(node, params);
    m_Domain->setParams(params);
}

// ---- PhysShapeLeaf -----------------------------------------------------------------------------------------

PhysShapeLeaf::PhysShapeLeaf() { std::memset(&m_Geometry, 0, sizeof(m_Geometry)); }

void PhysShapeLeaf::SetSphere(float radius) {
    std::memset(&m_Geometry, 0, sizeof(m_Geometry));
    m_Type = CollisionShapeType::Sphere;
    m_Geometry.sphere.radius = radius;
    m_Asset = nullptr;
    notifyGeometryChanged();
}

void PhysShapeLeaf::SetCapsule(float radius, float halfHeight) {
    std::memset(&m_Geometry, 0, sizeof(m_Geometry));
    m_Type = CollisionShapeType::Capsule;
    m_Geometry.capsule.radius = radius;
    m_Geometry.capsule.halfHeight = halfHeight;
    m_Asset = nullptr;
    notifyGeometryChanged();
}

void PhysShapeLeaf::SetBox(const dm::float3& halfExtents) {
    std::memset(&m_Geometry, 0, sizeof(m_Geometry));
    m_Type = CollisionShapeType::Box;
    m_Geometry.box.halfExtents[0] = halfExtents.x;
    m_Geometry.box.halfExtents[1] = halfExtents.y;
    m_Geometry.box.halfExtents[2] = halfExtents.z;
    m_Asset = nullptr;
    notifyGeometryChanged();
}

void PhysShapeLeaf::SetConvexMesh(IConvexAsset* asset, const dm::float3& scale) {
    std::memset(&m_Geometry, 0, sizeof(m_Geometry));
    m_Type = CollisionShapeType::ConvexMesh;
    m_Geometry.convexMesh.scale[0] = scale.x;
    m_Geometry.convexMesh.scale[1] = scale.y;
    m_Geometry.convexMesh.scale[2] = scale.z;
    m_Geometry.convexMesh.mesh = asset ? asset->getId() : 0;
    m_Asset = asset;
    notifyGeometryChanged();
}

void PhysShapeLeaf::SetTriangleMesh(ITriangleMeshAsset* asset, const dm::float3& scale) {
    std::memset(&m_Geometry, 0, sizeof(m_Geometry));
    m_Type = CollisionShapeType::TriangleMesh;
    m_Geometry.triMesh.scale[0] = scale.x;
    m_Geometry.triMesh.scale[1] = scale.y;
    m_Geometry.triMesh.scale[2] = scale.z;
    m_Geometry.triMesh.mesh = asset ? asset->getId() : 0;
    m_Asset = asset;
    notifyGeometryChanged();
}

void PhysShapeLeaf::SetSdf(ISdfAsset* asset, float width) {
    std::memset(&m_Geometry, 0, sizeof(m_Geometry));
    m_Type = CollisionShapeType::Sdf;
    m_Geometry.sdf.scale = width;
    m_Geometry.sdf.field = asset ? asset->getId() : 0;
    m_Asset = asset;
    notifyGeometryChanged();
}

int32_t PhysShapeLeaf::GetFlags() const {
    return makeShapeFlagsWithChannels(m_Type, m_Dynamic, m_Channels) | (m_Trigger ? int32_t(ShapeFlagTrigger) : 0);
}

void PhysShapeLeaf::notifyGeometryChanged() {
    if (SceneGraphNode* node = GetNode()) detail::refreshNodeBounds(node);
}

dm::box3 PhysShapeLeaf::GetLocalBoundingBox() {
    const CollisionGeometry& g = m_Geometry;
    switch (m_Type) {
        case CollisionShapeType::Sphere:
            return dm::box3(dm::float3(-g.sphere.radius), dm::float3(g.sphere.radius));
        case CollisionShapeType::Capsule: {
            const dm::float3 extent(g.capsule.halfHeight + g.capsule.radius, g.capsule.radius, g.capsule.radius);
            return dm::box3(-extent, extent);
        }
        case CollisionShapeType::Box: {
            const dm::float3 extent(g.box.halfExtents[0], g.box.halfExtents[1], g.box.halfExtents[2]);
            return dm::box3(-extent, extent);
        }
        case CollisionShapeType::ConvexMesh:
        case CollisionShapeType::TriangleMesh: {
            const Bounds3* bounds = nullptr;
            nvrhi::AutoPtr<IConvexAsset> convex;
            nvrhi::AutoPtr<ITriangleMeshAsset> mesh;
            if (m_Asset && m_Type == CollisionShapeType::ConvexMesh &&
                NVRHI_SUCCEEDED(m_Asset->QueryInterface(NVRHI_IID_PPV_ARGS(&convex))))
                bounds = &convex->getBounds();
            if (m_Asset && m_Type == CollisionShapeType::TriangleMesh &&
                NVRHI_SUCCEEDED(m_Asset->QueryInterface(NVRHI_IID_PPV_ARGS(&mesh))))
                bounds = &mesh->getBounds();
            if (!bounds) return dm::box3::empty();
            const dm::float3 scale(g.convexMesh.scale[0], g.convexMesh.scale[1], g.convexMesh.scale[2]);
            const dm::float3 a = dm::float3(bounds->lower[0], bounds->lower[1], bounds->lower[2]) * scale;
            const dm::float3 b = dm::float3(bounds->upper[0], bounds->upper[1], bounds->upper[2]) * scale;
            return dm::box3(dm::min(a, b), dm::max(a, b));
        }
        case CollisionShapeType::Sdf:
            return dm::box3(dm::float3(0.f), dm::float3(g.sdf.scale));
    }
    return dm::box3::empty();
}

nvrhi::AutoPtr<SceneGraphLeaf> PhysShapeLeaf::Clone() {
    auto copy = MAKE_RC_OBJ_PTR(PhysShapeLeaf);
    copy->m_Type = m_Type;
    copy->m_Geometry = m_Geometry;
    copy->m_Asset = m_Asset;  // shares the asset
    copy->m_Dynamic = m_Dynamic;
    copy->m_Trigger = m_Trigger;
    copy->m_Channels = m_Channels;
    copy->m_TargetDomain = m_TargetDomain;
    copy->m_TargetDomainPath = m_TargetDomainPath;
    return copy;
}

void PhysShapeLeaf::Load(const Json::Value& node) {
    const Json::Value& shape = node["shape"];
    const std::string type = shape.isString() ? shape.asString() : std::string("sphere");
    if (type == "sphere") {
        float radius = m_Type == CollisionShapeType::Sphere ? m_Geometry.sphere.radius : 0.5f;
        node["radius"] >> radius;
        SetSphere(radius);
    } else if (type == "capsule") {
        float radius = 0.25f, halfHeight = 0.5f;
        node["radius"] >> radius;
        node["halfHeight"] >> halfHeight;
        SetCapsule(radius, halfHeight);
    } else if (type == "box") {
        dm::float3 halfExtents(0.5f);
        node["halfExtents"] >> halfExtents;
        SetBox(halfExtents);
    } else {
        donut::log::warning("PhysShape '%s': shape type '%s' cannot be loaded from a scene file (sphere, capsule "
                            "and box can; an application sets the asset shapes)",
                            GetNode() ? GetNode()->GetName().c_str() : "", type.c_str());
    }

    node["dynamic"] >> m_Dynamic;
    node["trigger"] >> m_Trigger;
    const Json::Value& channels = node["channels"];
    if (channels.isIntegral()) {
        SetChannels(channels.asInt());
    } else if (channels.isArray()) {
        int32_t bits = 0;
        for (const Json::Value& channel : channels)
            if (channel.isIntegral() && channel.asInt() >= 0 && channel.asInt() < 7)
                bits |= int32_t(PhaseShapeChannel0) << channel.asInt();
        SetChannels(bits);
    }
    const Json::Value& domain = node["domain"];
    if (domain.isString()) SetTargetDomainPath(domain.asString());
}

// ---- PhysSceneGraph ----------------------------------------------------------------------------------------

void PhysSceneGraph::addDomainRef(IPhysDomain* domain) {
    if (domain && m_Domains.AddRef(domain) && OnDomainAdded) OnDomainAdded(domain);
}

void PhysSceneGraph::releaseDomainRef(IPhysDomain* domain) {
    if (domain && m_Domains.Release(domain) && OnDomainRemoved) OnDomainRemoved(domain);
}

void PhysSceneGraph::RegisterLeaf(SceneGraphLeaf* leaf) {
    SceneGraph::RegisterLeaf(leaf);
    if (!leaf) return;

    if (auto* instance = dynamic_cast<PhysDomainInstance*>(leaf)) {
        m_DomainInstances.push_back(instance);
        addDomainRef(instance->GetDomain());
        return;
    }
    if (auto* shape = dynamic_cast<PhysShapeLeaf*>(leaf)) {
        m_ShapeLeaves.push_back(shape);
        return;
    }
}

void PhysSceneGraph::UnregisterLeaf(SceneGraphLeaf* leaf) {
    SceneGraph::UnregisterLeaf(leaf);
    if (!leaf) return;

    if (auto* instance = dynamic_cast<PhysDomainInstance*>(leaf)) {
        // The instance keeps its reference while the callback runs: erase it from the list afterwards.
        nvrhi::AutoPtr<PhysDomainInstance> keep = instance;
        auto it = std::find(m_DomainInstances.begin(), m_DomainInstances.end(), instance);
        if (it == m_DomainInstances.end()) return;
        m_DomainInstances.erase(it);  // keeps the registration order of the others
        releaseDomainRef(instance->GetDomain());
        return;
    }
    if (auto* shape = dynamic_cast<PhysShapeLeaf*>(leaf)) {
        auto it = std::find(m_ShapeLeaves.begin(), m_ShapeLeaves.end(), shape);
        if (it != m_ShapeLeaves.end()) m_ShapeLeaves.erase(it);
        return;
    }
}

// ---- PhysSceneTypeFactory -----------------------------------------------------------------------------------

nvrhi::AutoPtr<donut::engine::SceneGraph> PhysSceneTypeFactory::CreateGraph() {
    return MAKE_RC_OBJ_PTR(PhysSceneGraph);
}

nvrhi::AutoPtr<SceneGraphLeaf> PhysSceneTypeFactory::CreateLeaf(const std::string& type) {
    if (type == "PhysDomain") {
        auto instance = MAKE_RC_OBJ_PTR(PhysDomainInstance);
        instance->SetDeviceForLoad(m_Device);
        return instance;
    }
    if (type == "PhysShape") return MAKE_RC_OBJ_PTR(PhysShapeLeaf);
    return SceneTypeFactory::CreateLeaf(type);
}

}  // namespace ephys::scene
