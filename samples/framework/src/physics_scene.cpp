#include "physics_scene.h"

#include <ethereal/phys_framework/demo_context.h>

#include <donut/core/log.h>
#include <nvrhi/core/foundation.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace ephys::framework {

using helpers::ShapeDesc;

struct PhysicsScene::ShapeNode {
    nvrhi::AutoPtr<donut::engine::SceneGraphNode> node;
    nvrhi::AutoPtr<scene::PhysShapeLeaf> leaf;
    ConvexAssetHandle convex;
    std::shared_ptr<GpuMesh> renderMesh;
    CollisionShapeType type = CollisionShapeType::Sphere;
};

namespace {

dm::dquat toQuat(const dm::float4& q) { return dm::dquat(double(q.w), double(q.x), double(q.y), double(q.z)); }

}  // namespace

PhysicsScene::PhysicsScene(nvrhi::IDevice* device, IPhysDevice* phys, DemoContext* context, MessageCallback messages)
    : m_Device(device), m_Phys(phys), m_Context(context), m_Messages(std::move(messages)) {}

PhysicsScene::~PhysicsScene() {
    // everything the GPU may still read goes after it finished
    if (m_Phys)
        m_Phys->waitForIdle();
    m_Service.reset();
    m_Shapes.clear();
    m_ShapesNode = nullptr;
    m_DomainInstance = nullptr;
    m_Graph = nullptr;
    m_Readback = nullptr;
    m_Timers = nullptr;
    m_Domain = nullptr;
}

void PhysicsScene::report(PhysMessageSeverity severity, const char* format, ...) {
    char text[1024];
    va_list args;
    va_start(args, format);
    std::vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (m_Messages)
        m_Messages(severity, text);
}

PhysBufferHandle PhysicsScene::uploadBuffer(const void* data, uint32_t count, uint32_t stride, const char* name) {
    PhysBufferDesc desc;
    desc.elementCount = count ? count : 1;
    desc.elementStride = stride;
    desc.use = BufferUse::Upload;
    desc.debugName = name;
    PhysBufferHandle buffer;
    if (NVRHI_FAILED(m_Phys->createBuffer(desc, &buffer)) || !buffer)
        return nullptr;
    if (data && count) {
        void* mapped = nullptr;
        if (NVRHI_SUCCEEDED(buffer->map(&mapped)) && mapped) {
            std::memcpy(mapped, data, size_t(count) * stride);
            buffer->unmap();
        }
    }
    return buffer;
}

PhysBufferHandle PhysicsScene::readbackBuffer(uint32_t count, uint32_t stride, const char* name) {
    PhysBufferDesc desc;
    desc.elementCount = count ? count : 1;
    desc.elementStride = stride;
    desc.use = BufferUse::Readback;
    desc.debugName = name;
    PhysBufferHandle buffer;
    m_Phys->createBuffer(desc, &buffer);
    return buffer;
}

bool PhysicsScene::Create(DemoState& g, uint32_t maxParticles, uint32_t numParticles) {
    helpers::SimBuffers& b = g.buffers;
    m_MaxParticles = maxParticles;
    m_MaxDiffuse = uint32_t(std::max(0, g.maxDiffuseParticles));

    // ---- the domain (FleX: NvFlexCreateSolver with the solver description of the scene) ----
    DomainDesc domainDesc;
    domainDesc.maxParticles = maxParticles;
    domainDesc.maxDiffuseParticles = m_MaxDiffuse;
    if (NVRHI_FAILED(m_Phys->createDomain(domainDesc, &m_Domain)) || !m_Domain) {
        report(PhysMessageSeverity::Error, "cannot create a domain of %u particles", maxParticles);
        return false;
    }
    IParticleState* particles = m_Domain->getParticles();

    // ---- the render arrays: before the uploads, so that the uploads write into them ----
    if (!createRenderBuffers(g))
        return false;

    // ---- the data (FleX: NvFlexSetParams, SetParticles, ... of Init) ----
    m_Domain->setParams(g.params);

    // the host arrays have maxParticles elements (Init resized them); the copies take all of them, the active
    // count says which are simulated
    m_UploadPositions = uploadBuffer(b.positions.data(), maxParticles, 16, "host positions");
    m_UploadVelocities = uploadBuffer(b.velocities.data(), maxParticles, 12, "host velocities");
    m_UploadPhases = uploadBuffer(b.phases.data(), maxParticles, 4, "host phases");
    m_UploadActive = uploadBuffer(nullptr, maxParticles, 4, "host active indices");
    PhysBufferHandle normals = uploadBuffer(b.normals.data(), maxParticles, 16, "host normals");
    PhysBufferHandle rest = uploadBuffer(b.restPositions.data(), maxParticles, 16, "host rest positions");
    {
        void* mapped = nullptr;
        if (NVRHI_SUCCEEDED(m_UploadActive->map(&mapped)) && mapped) {
            std::memcpy(mapped, b.activeIndices.data(), sizeof(int) * b.activeIndices.size());
            m_UploadActive->unmap();
        }
    }
    if (!m_UploadPositions || !m_UploadVelocities || !m_UploadPhases || !m_UploadActive || !normals || !rest)
        return false;

    const CopyRegion all{0, 0, maxParticles};
    bool ok = true;
    ok &= NVRHI_SUCCEEDED(particles->upload(nullptr, ParticleArray::Position, m_UploadPositions, &all));
    ok &= NVRHI_SUCCEEDED(particles->upload(nullptr, ParticleArray::Velocity, m_UploadVelocities, &all));
    ok &= NVRHI_SUCCEEDED(particles->upload(nullptr, ParticleArray::Normal, normals, &all));
    ok &= NVRHI_SUCCEEDED(particles->upload(nullptr, ParticleArray::Phase, m_UploadPhases, &all));
    ok &= NVRHI_SUCCEEDED(particles->upload(nullptr, ParticleArray::RestPosition, rest, &all));
    if (numParticles) {
        const CopyRegion active{0, 0, uint32_t(b.activeIndices.size())};
        ok &= NVRHI_SUCCEEDED(particles->upload(nullptr, ParticleArray::ActiveIndex, m_UploadActive, &active));
    }
    ok &= NVRHI_SUCCEEDED(particles->setActiveCount(uint32_t(b.activeIndices.size())));

    // springs
    if (!b.springLengths.empty()) {
        const uint32_t n = uint32_t(b.springLengths.size());
        PhysBufferHandle indices = uploadBuffer(b.springIndices.data(), n * 2, 4, "host spring indices");
        PhysBufferHandle lengths = uploadBuffer(b.springLengths.data(), n, 4, "host spring lengths");
        PhysBufferHandle stiffness = uploadBuffer(b.springStiffness.data(), n, 4, "host spring stiffness");
        ok &= NVRHI_SUCCEEDED(m_Domain->getSprings()->upload(nullptr, indices, lengths, stiffness, n));
    }

    // rigids
    if (b.rigidOffsets.size() > 1) {
        const uint32_t numRigids = uint32_t(b.rigidOffsets.size() - 1);
        const uint32_t numIndices = uint32_t(b.rigidIndices.size());
        PhysBufferHandle offsets = uploadBuffer(b.rigidOffsets.data(), numRigids + 1, 4, "host rigid offsets");
        PhysBufferHandle indices = uploadBuffer(b.rigidIndices.data(), numIndices, 4, "host rigid indices");
        PhysBufferHandle local = uploadBuffer(b.rigidLocalPositions.data(), numIndices, 12, "host rigid local positions");
        PhysBufferHandle localNormals;
        if (b.rigidLocalNormals.size() >= numIndices)
            localNormals = uploadBuffer(b.rigidLocalNormals.data(), numIndices, 16, "host rigid local normals");
        PhysBufferHandle coefficients = uploadBuffer(b.rigidCoefficients.data(), numRigids, 4, "host rigid stiffness");
        PhysBufferHandle thresholds, creeps;
        if (b.rigidPlasticThresholds.size() >= numRigids && b.rigidPlasticCreeps.size() >= numRigids) {
            thresholds = uploadBuffer(b.rigidPlasticThresholds.data(), numRigids, 4, "host plastic thresholds");
            creeps = uploadBuffer(b.rigidPlasticCreeps.data(), numRigids, 4, "host plastic creeps");
        }
        PhysBufferHandle rotations = uploadBuffer(b.rigidRotations.data(), numRigids, 16, "host rigid rotations");
        PhysBufferHandle translations = uploadBuffer(b.rigidTranslations.data(), numRigids, 12, "host rigid translations");

        ShapeMatchBuffers buffers;
        buffers.offsets = offsets;
        buffers.indices = indices;
        buffers.restPositions = local;
        buffers.restNormals = localNormals;
        buffers.stiffness = coefficients;
        buffers.plasticThresholds = thresholds;
        buffers.plasticCreeps = creeps;
        buffers.rotations = rotations;
        buffers.translations = translations;
        ok &= NVRHI_SUCCEEDED(m_Domain->getShapeMatches()->upload(nullptr, buffers, numRigids, numIndices));

        m_ReadRotations = readbackBuffer(numRigids, 16, "rigid rotations readback");
        m_ReadTranslations = readbackBuffer(numRigids, 12, "rigid translations readback");
    }

    // inflatables
    if (!b.inflatableTriOffsets.empty()) {
        std::vector<InflatableDesc> inflatables(b.inflatableTriOffsets.size());
        for (size_t i = 0; i < inflatables.size(); ++i) {
            inflatables[i].startTri = b.inflatableTriOffsets[i];
            inflatables[i].numTris = b.inflatableTriCounts[i];
            inflatables[i].restVolume = b.inflatableVolumes[i];
            inflatables[i].overPressure = b.inflatablePressures[i];
            inflatables[i].constraintScale = b.inflatableCoefficients[i];
        }
        ok &= NVRHI_SUCCEEDED(
            m_Domain->getInflatables()->upload(nullptr, inflatables.data(), uint32_t(inflatables.size())));
    }

    // dynamic triangles
    if (!b.triangles.empty()) {
        const uint32_t n = uint32_t(b.triangles.size() / 3);
        PhysBufferHandle indices = uploadBuffer(b.triangles.data(), n * 3, 4, "host triangles");
        PhysBufferHandle triNormals;
        if (b.triangleNormals.size() >= n)
            triNormals = uploadBuffer(b.triangleNormals.data(), n, 12, "host triangle normals");
        ok &= NVRHI_SUCCEEDED(m_Domain->getTriangles()->upload(nullptr, indices, triNormals, n));

        // the cloth's index buffer for the renderer (the triangles do not change in these scenes)
        nvrhi::BufferDesc d;
        d.byteSize = sizeof(int) * b.triangles.size();
        d.isIndexBuffer = true;
        d.debugName = "cloth indices";
        d.initialState = nvrhi::ResourceStates::IndexBuffer;
        d.keepInitialState = true;
        m_Device->createBuffer(d, &m_ClothIndices);
        m_NumTriangles = n;
        PhysBufferHandle adopted;
        if (m_ClothIndices && NVRHI_SUCCEEDED(m_Phys->adoptBuffer(m_ClothIndices, n * 3, 4, &adopted)))
            ok &= NVRHI_SUCCEEDED(m_Phys->copyBuffer(nullptr, adopted, indices, CopyRegion{0, 0, n * 3}));
    }

    if (!ok) {
        report(PhysMessageSeverity::Error, "the scene's data could not be sent to the domain");
        return false;
    }

    // ---- the scene graph: the domain and the shapes as leaves (ethereal_phys_scene) ----
    m_Graph = MAKE_RC_OBJ_PTR(scene::PhysSceneGraph);
    auto root = MAKE_RC_OBJ_PTR(donut::engine::SceneGraphNode);
    root->SetName("scene");
    m_Graph->SetRootNode(root);

    SolverDesc solverDesc;
    solverDesc.mode = g.solverMode;
    solverDesc.maxNeighborsPerParticle = uint32_t(g.maxNeighborsPerParticle);
    solverDesc.maxContactsPerParticle = uint32_t(g.maxContactsPerParticle);
    m_DomainInstance = MAKE_RC_OBJ_PTR(scene::PhysDomainInstance, m_Domain.Get(), solverDesc);
    m_Graph->AttachLeafNode(root, m_DomainInstance)->SetName("domain");

    auto shapesNode = MAKE_RC_OBJ_PTR(donut::engine::SceneGraphNode);
    shapesNode->SetName("shapes");
    m_ShapesNode = m_Graph->Attach(root, shapesNode);
    createShapeNodes(g, nullptr);

    scene::PhysSceneServiceDesc serviceDesc;
    serviceDesc.device = m_Phys;
    serviceDesc.messageCallback = m_Messages;
    m_Service = std::make_unique<scene::PhysSceneService>(m_Graph.Get(), serviceDesc);

    IPhysSolver* solver = m_Service->GetSolver(m_Domain);
    if (!solver) {
        report(PhysMessageSeverity::Error, "the scene service created no solver");
        return false;
    }
    if (g.solverMode == SolverMode::XPBD) {
        nvrhi::AutoPtr<IPhysSolverXPBD> xpbd;
        if (NVRHI_SUCCEEDED(solver->QueryInterface(NVRHI_IID_PPV_ARGS(&xpbd))))
            xpbd->setCompliance(g.xpbd);
    }
    if (NVRHI_FAILED(m_Phys->createReadback(solver, &m_Readback)) ||
        NVRHI_FAILED(m_Phys->createTimers(solver, &m_Timers)))
        return false;

    // ---- host mirror readbacks ----
    m_ReadPositions = readbackBuffer(maxParticles, 16, "positions readback");
    m_ReadVelocities = readbackBuffer(maxParticles, 12, "velocities readback");
    m_ReadNormals = readbackBuffer(maxParticles, 16, "normals readback");
    for (CountSlot& slot : m_CountSlots) {
        if (m_MaxDiffuse) {
            slot.buffer = readbackBuffer(1, 4, "diffuse count readback");
        }
    }

    // the renderer reads the arrays from the first frame on (before the first step)
    particles->updateRenderArrays(nullptr);
    m_HostValid = true;
    m_Mapped = false;

    // everything recorded on the device's own command list (the uploads, the solver's clears) runs before the
    // first frame; on D3D11 the frame's command list may only be opened after this
    return NVRHI_SUCCEEDED(m_Phys->waitForIdle());
}

bool PhysicsScene::createRenderBuffers(DemoState& g) {
    m_Shared = m_Phys->getInfo().supportsRenderSharedBuffers;
    IParticleState* particles = m_Domain->getParticles();
    const uint32_t maxParticles = m_MaxParticles;

    // A particle array as a renderer buffer: shared (a structured buffer with UAVs the solver writes, and a vertex
    // or index buffer the renderer binds) where the device can, a plain vertex or index buffer that
    // updateRenderArrays copies into where it cannot.
    auto renderArray = [&](ParticleArray array, bool index, const char* name, nvrhi::BufferHandle& buffer,
                           PhysBufferHandle& adopted) -> bool {
        const uint32_t stride = particleArrayStride(array);
        nvrhi::BufferDesc d;
        d.byteSize = uint64_t(maxParticles) * stride;
        d.isVertexBuffer = !index;
        d.isIndexBuffer = index;
        d.debugName = name;
        d.keepInitialState = true;
        if (m_Shared) {
            d.structStride = stride;
            d.canHaveUAVs = true;
            d.initialState = nvrhi::ResourceStates::UnorderedAccess;
        } else {
            d.initialState = index ? nvrhi::ResourceStates::IndexBuffer : nvrhi::ResourceStates::VertexBuffer;
        }
        m_Device->createBuffer(d, &buffer);
        if (!buffer || NVRHI_FAILED(m_Phys->adoptBuffer(buffer, maxParticles, stride, &adopted)))
            return false;
        const nvrhi::FRESULT r = m_Shared ? particles->setArray(nullptr, array, adopted)
                                          : particles->setRenderArray(array, adopted);
        return NVRHI_SUCCEEDED(r);
    };

    bool ok = renderArray(ParticleArray::Position, false, "particle positions", m_PositionsVB, m_PositionsRA);
    ok = ok && renderArray(ParticleArray::Normal, false, "particle normals", m_NormalsVB, m_NormalsRA);
    ok = ok && renderArray(ParticleArray::Phase, false, "particle phases", m_PhasesVB, m_PhasesRA);
    ok = ok && renderArray(ParticleArray::ActiveIndex, true, "active particles", m_ActiveIB, m_ActiveRA);
    if (!ok) {
        report(PhysMessageSeverity::Error, "cannot create the render arrays");
        return false;
    }

    // A renderer buffer a solver output (or the diffuse state) is copied into.
    auto copyTarget = [&](uint32_t count, uint32_t stride, const char* name, nvrhi::BufferHandle& buffer,
                          PhysBufferHandle& adopted) -> bool {
        nvrhi::BufferDesc d;
        d.byteSize = uint64_t(count) * stride;
        d.isVertexBuffer = true;
        d.debugName = name;
        d.initialState = nvrhi::ResourceStates::VertexBuffer;
        d.keepInitialState = true;
        m_Device->createBuffer(d, &buffer);
        return buffer && NVRHI_SUCCEEDED(m_Phys->adoptBuffer(buffer, count, stride, &adopted));
    };

    ok = copyTarget(maxParticles, 16, "smoothed positions", m_SmoothVB, m_SmoothDst);
    ok = ok && copyTarget(maxParticles, 4, "densities", m_DensityVB, m_DensityDst);
    for (int i = 0; i < 3 && ok; ++i)
        ok = copyTarget(maxParticles, 16, "anisotropy", m_AnisotropyVB[i], m_AnisotropyDst[i]);

    if (ok && m_MaxDiffuse) {
        ok = copyTarget(m_MaxDiffuse, 16, "diffuse positions", m_DiffusePositionsVB, m_DiffusePositionsDst);
        ok = ok && copyTarget(m_MaxDiffuse, 16, "diffuse velocities", m_DiffuseVelocitiesVB, m_DiffuseVelocitiesDst);
        if (ok) {
            // the indirect arguments {count, 1, 0, 0}: the count copied from the domain every frame
            nvrhi::BufferDesc d;
            d.byteSize = 16;
            d.isDrawIndirectArgs = true;
            d.debugName = "diffuse draw arguments";
            d.initialState = nvrhi::ResourceStates::IndirectArgument;
            d.keepInitialState = true;
            m_Device->createBuffer(d, &m_DiffuseArgs);
            ok = m_DiffuseArgs && NVRHI_SUCCEEDED(m_Phys->adoptBuffer(m_DiffuseArgs, 4, 4, &m_DiffuseArgsDst));
            const uint32_t args[4] = {0, 1, 0, 0};
            ok = ok && NVRHI_SUCCEEDED(m_Phys->writeBuffer(nullptr, m_DiffuseArgsDst, args, 0, 4));
        }
    }
    if (!ok)
        report(PhysMessageSeverity::Error, "cannot create the renderer's copies of the solver outputs");
    (void)g;
    return ok;
}

// commandList: the frame's, or null for the device's own (outside a frame).
void PhysicsScene::createShapeNodes(DemoState& g, nvrhi::ICommandList* commandList) {
    IGeometryLibrary* geometry = m_Phys->getGeometry();
    for (const ShapeDesc& shape : g.buffers.shapes) {
        ShapeNode entry;
        entry.type = shape.type;
        entry.leaf = MAKE_RC_OBJ_PTR(scene::PhysShapeLeaf);
        switch (shape.type) {
            case CollisionShapeType::Sphere:
                entry.leaf->SetSphere(shape.radius);
                break;
            case CollisionShapeType::Capsule:
                entry.leaf->SetCapsule(shape.radius, shape.halfHeight);
                break;
            case CollisionShapeType::Box:
                entry.leaf->SetBox(shape.halfExtents);
                break;
            case CollisionShapeType::ConvexMesh: {
                // FleX: NvFlexCreateConvexMesh + NvFlexUpdateConvexMesh in AddRandomConvex
                const uint32_t n = uint32_t(shape.convexPlanes.size());
                PhysBufferHandle planes = uploadBuffer(shape.convexPlanes.data(), n, 16, "convex planes");
                if (planes && NVRHI_SUCCEEDED(geometry->createConvex(&entry.convex))) {
                    Bounds3 bounds;
                    std::memcpy(bounds.lower, &shape.convexLower.x, sizeof(bounds.lower));
                    std::memcpy(bounds.upper, &shape.convexUpper.x, sizeof(bounds.upper));
                    entry.convex->update(commandList, planes, n, bounds);
                    entry.leaf->SetConvexMesh(entry.convex, shape.scale);
                }
                if (shape.renderMesh) {
                    // without a frame's command list the mesh is uploaded on a list of the renderer: the device's
                    // own is executed first (D3D11 has one immediate context, one open list at a time)
                    if (!commandList)
                        m_Phys->flush();
                    entry.renderMesh = m_Context->CreateGpuMesh(shape.renderMesh.get(), commandList);
                }
                break;
            }
            default:
                // triangle meshes and distance fields come with the scenes of step 10b; the entry keeps the
                // indices of m_Shapes aligned with the shapes of g.buffers
                report(PhysMessageSeverity::Warning, "a shape of type %d is not supported yet; left out",
                       int(shape.type));
                entry.leaf = nullptr;
                m_Shapes.push_back(std::move(entry));
                continue;
        }
        entry.leaf->SetDynamic(shape.dynamic);
        entry.leaf->SetTrigger(shape.trigger);
        entry.leaf->SetChannels(shape.channels);
        entry.leaf->SetTargetDomain(m_Domain);

        entry.node = m_Graph->AttachLeafNode(m_ShapesNode, entry.leaf);
        const dm::double3 translation(shape.position);
        const dm::dquat rotation = toQuat(shape.rotation);
        entry.node->SetTransform(&translation, &rotation, nullptr);
        m_Shapes.push_back(std::move(entry));
    }
    g.buffers.shapesChanged = false;
}

void PhysicsScene::updateShapeNodes(DemoState& g, nvrhi::ICommandList* commandList) {
    // same shapes: new transforms (the node keeps the previous one, which the service sweeps from); otherwise the
    // shape nodes are made again
    bool same = m_Shapes.size() == g.buffers.shapes.size();
    for (size_t i = 0; same && i < m_Shapes.size(); ++i)
        same = m_Shapes[i].type == g.buffers.shapes[i].type;

    if (!same) {
        for (ShapeNode& entry : m_Shapes)
            if (entry.node)
                m_Graph->Detach(entry.node);
        m_Shapes.clear();
        createShapeNodes(g, commandList);
        return;
    }

    for (size_t i = 0; i < m_Shapes.size(); ++i) {
        const ShapeDesc& shape = g.buffers.shapes[i];
        ShapeNode& entry = m_Shapes[i];
        if (!entry.node)
            continue;
        switch (shape.type) {
            case CollisionShapeType::Sphere: entry.leaf->SetSphere(shape.radius); break;
            case CollisionShapeType::Capsule: entry.leaf->SetCapsule(shape.radius, shape.halfHeight); break;
            case CollisionShapeType::Box: entry.leaf->SetBox(shape.halfExtents); break;
            default: break;
        }
        const dm::double3 translation(shape.position);
        const dm::dquat rotation = toQuat(shape.rotation);
        entry.node->SetTransform(&translation, &rotation, nullptr);
    }
    g.buffers.shapesChanged = false;
}

void PhysicsScene::Warmup(DemoState& g) {
    IPhysSolver* solver = GetSolver();
    if (!solver)
        return;

    // the shapes the warm-up collides with
    m_Graph->Refresh(0);
    m_Service->SyncToPhysics(nullptr);

    // relax the positions to the rest density without changing the velocities
    PhysParams copy = g.params;
    copy.numIterations = 4;
    m_Domain->setParams(copy);

    const int kWarmupIterations = 100;
    IParticleState* particles = m_Domain->getParticles();
    for (int i = 0; i < kWarmupIterations; ++i) {
        solver->step(nullptr, 0.0001f, 1);
        particles->upload(nullptr, ParticleArray::Velocity, m_UploadVelocities, nullptr);
    }
    m_Domain->setParams(g.params);
    m_Phys->waitForIdle();
    m_HostValid = false;
}

void PhysicsScene::MapBuffers(DemoState& g) {
    if (m_Mapped)
        return;
    m_Mapped = true;
    if (m_HostValid)
        return;

    helpers::SimBuffers& b = g.buffers;
    IParticleState* particles = m_Domain->getParticles();
    particles->download(nullptr, ParticleArray::Position, m_ReadPositions, nullptr);
    particles->download(nullptr, ParticleArray::Velocity, m_ReadVelocities, nullptr);
    particles->download(nullptr, ParticleArray::Normal, m_ReadNormals, nullptr);
    const bool rigids = m_ReadRotations && m_Domain->getShapeMatches()->getNumRigids() > 0;
    if (rigids) {
        ShapeMatchBuffers buffers;
        buffers.rotations = m_ReadRotations;
        buffers.translations = m_ReadTranslations;
        m_Domain->getShapeMatches()->download(nullptr, buffers);
    }
    m_Phys->waitForIdle();

    auto fetch = [](IPhysBuffer* buffer, void* dst, size_t bytes) {
        void* mapped = nullptr;
        if (NVRHI_SUCCEEDED(buffer->map(&mapped)) && mapped) {
            std::memcpy(dst, mapped, bytes);
            buffer->unmap();
        }
    };
    b.positions.resize(m_MaxParticles);
    b.velocities.resize(m_MaxParticles);
    b.normals.resize(m_MaxParticles);
    fetch(m_ReadPositions, b.positions.data(), sizeof(dm::float4) * m_MaxParticles);
    fetch(m_ReadVelocities, b.velocities.data(), sizeof(dm::float3) * m_MaxParticles);
    fetch(m_ReadNormals, b.normals.data(), sizeof(dm::float4) * m_MaxParticles);
    if (rigids) {
        const size_t n = b.rigidRotations.size();
        fetch(m_ReadRotations, b.rigidRotations.data(), sizeof(dm::float4) * n);
        fetch(m_ReadTranslations, b.rigidTranslations.data(), sizeof(dm::float3) * n);
    }
    m_HostValid = true;
}

void PhysicsScene::BeginFrame(nvrhi::ICommandList* commandList, DemoState& g) {
    helpers::SimBuffers& b = g.buffers;
    IParticleState* particles = m_Domain->getParticles();

    if (m_Mapped) {
        // FleX's NvFlexSetParticles / SetVelocities / SetPhases / SetActive / SetActiveCount of every frame. The
        // GPU is idle here (MapBuffers waited for it this frame), so the upload buffers may be rewritten.
        auto put = [](IPhysBuffer* buffer, const void* src, size_t bytes) {
            void* mapped = nullptr;
            if (NVRHI_SUCCEEDED(buffer->map(&mapped)) && mapped) {
                std::memcpy(mapped, src, bytes);
                buffer->unmap();
            }
        };
        const uint32_t numActive = uint32_t(std::min<size_t>(b.activeIndices.size(), m_MaxParticles));
        put(m_UploadPositions, b.positions.data(), sizeof(dm::float4) * m_MaxParticles);
        put(m_UploadVelocities, b.velocities.data(), sizeof(dm::float3) * m_MaxParticles);
        put(m_UploadPhases, b.phases.data(), sizeof(int) * m_MaxParticles);
        put(m_UploadActive, b.activeIndices.data(), sizeof(int) * numActive);
        particles->upload(commandList, ParticleArray::Position, m_UploadPositions, nullptr);
        particles->upload(commandList, ParticleArray::Velocity, m_UploadVelocities, nullptr);
        particles->upload(commandList, ParticleArray::Phase, m_UploadPhases, nullptr);
        if (numActive) {
            const CopyRegion active{0, 0, numActive};
            particles->upload(commandList, ParticleArray::ActiveIndex, m_UploadActive, &active);
        }
        particles->setActiveCount(numActive);
    }
    m_Mapped = false;

    // what the renderer reads this frame
    particles->updateRenderArrays(commandList);
    if (g.drawEllipsoids) {
        m_Readback->readSmoothedPositions(commandList, m_SmoothDst, nullptr);
        m_Readback->readAnisotropy(commandList, m_AnisotropyDst[0], m_AnisotropyDst[1], m_AnisotropyDst[2], nullptr);
    }
    if (g.drawDensity)
        m_Readback->readDensities(commandList, m_DensityDst, nullptr);
    if (m_MaxDiffuse)
        m_Readback->readDiffuseParticles(commandList, m_DiffusePositionsDst, m_DiffuseVelocitiesDst, m_DiffuseArgsDst,
                                         nullptr);
}

bool PhysicsScene::Simulate(nvrhi::ICommandList* commandList, DemoState& g, bool advance, uint32_t frameIndex) {
    // FleX set the parameters every frame; the domain is told only when they changed (a change makes the solver
    // recompute what it derives from them)
    if (std::memcmp(&m_Domain->getParams(), &g.params, sizeof(PhysParams)) != 0)
        m_Domain->setParams(g.params);

    if (g.buffers.shapesChanged)
        updateShapeNodes(g, commandList);

    m_Graph->Refresh(frameIndex);
    bool ok = m_Service->SyncToPhysics(commandList);
    if (advance) {
        ok &= m_Service->Step(g.dt, uint32_t(std::max(1, g.numSubsteps)), commandList);
        m_HostValid = false;
    }
    m_Service->SyncFromPhysics(commandList);

    // the diffuse count for the UI, without waiting: each of four readback buffers is copied into every fourth
    // frame and read just before it is copied into again. Donut's device manager keeps at most two frames in
    // flight, so the copy of four frames ago has finished (on D3D11 the map would wait for it anyway). No event
    // query: nvrhi's D3D11 pollEventQuery takes S_FALSE for done, and the next End abandons an unread result (a
    // debug-layer warning; scene_service.cpp has the same note).
    if (m_MaxDiffuse) {
        CountSlot& slot = m_CountSlots[m_CountSlot];
        if (slot.recorded) {
            void* mapped = nullptr;
            if (NVRHI_SUCCEEDED(slot.buffer->map(&mapped)) && mapped) {
                m_DiffuseCount = *static_cast<const uint32_t*>(mapped);
                slot.buffer->unmap();
            }
        }
        m_Readback->readDiffuseParticles(commandList, nullptr, nullptr, slot.buffer, nullptr);
        slot.recorded = true;
        m_CountSlot = (m_CountSlot + 1) % 4;
    }
    return ok;
}

void PhysicsScene::Submitted() { m_Service->NotifySubmitted(nvrhi::CommandQueue::Graphics); }

FluidRenderBuffers PhysicsScene::GetFluidBuffers(bool smoothed, bool density) const {
    FluidRenderBuffers r;
    r.positions = smoothed ? m_SmoothVB.Get() : m_PositionsVB.Get();
    r.densities = density ? m_DensityVB.Get() : m_PhasesVB.Get();
    for (int i = 0; i < 3; ++i)
        r.anisotropy[i] = m_AnisotropyVB[i];
    r.indices = m_ActiveIB;
    return r;
}

DiffuseRenderBuffers PhysicsScene::GetDiffuseBuffers() const {
    DiffuseRenderBuffers r;
    r.positions = m_DiffusePositionsVB;
    r.velocities = m_DiffuseVelocitiesVB;
    r.drawArgs = m_DiffuseArgs;
    r.maxParticles = m_MaxDiffuse;
    return r;
}

ClothRenderBuffers PhysicsScene::GetClothBuffers() const {
    ClothRenderBuffers r;
    r.positions = m_PositionsVB;
    r.normals = m_NormalsVB;
    r.indices = m_ClothIndices;
    r.numTriangles = m_NumTriangles;
    r.numPositions = m_MaxParticles;
    return r;
}

GpuMesh* PhysicsScene::GetConvexMesh(size_t shapeIndex) const {
    return shapeIndex < m_Shapes.size() ? m_Shapes[shapeIndex].renderMesh.get() : nullptr;
}

uint32_t PhysicsScene::GetActiveCount() const { return m_Domain ? m_Domain->getParticles()->getActiveCount() : 0; }

IPhysSolver* PhysicsScene::GetSolver() const {
    return m_Service && m_Domain ? m_Service->GetSolver(m_Domain) : nullptr;
}

}  // namespace ephys::framework
