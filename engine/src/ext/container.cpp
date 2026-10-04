// IPhysContainer, IPhysInstance and IPhysSoftJoint (ext/container.h): FleX's container
// (reference/flex/extensions/flexExtContainer.cpp) over a domain and its solver, through their public interfaces.
//
// The allocator, the compaction, the instance spawn, the soft joints and updateInstances follow FleX statement by
// statement (the parity test compares the active lists, the mirrors and the compacted transforms with FleX's
// extension library). What differs is the transport: FleX's host buffers that the solver copied from and to
// became mirrors in the container's memory, written to the domain's arrays with IPhysDevice::writeBuffer and
// filled from Readback buffers when they are read (ext/container.h, "Pull and resolve" and "Push").

#include "ext/ext_objects.h"
#include "ext/flex_math.h"

#include "phys_messages.h"

#include <nvrhi/core/foundation.h>
#include <nvrhi/core/memory.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <new>
#include <utility>
#include <vector>

namespace ephys {
namespace {

using namespace detail::flex;
using detail::report;

// The mirrors, what the application sees through the map views: memory of nvrhi's process-wide allocator
// (com-interface-abi.md, section 6), so that no block of this module's CRT heap is ever handed out. Plain
// arrays of a trivial type; growing keeps the contents and zero-fills the rest (FleX left it uninitialised).
template <typename T>
class HostArray {
 public:
    HostArray() = default;
    HostArray(const HostArray&) = delete;
    HostArray& operator=(const HostArray&) = delete;
    ~HostArray() { reset(); }

    void resize(size_t count) {
        if (count == m_Size) return;
        T* data = nullptr;
        if (count) {
            data = static_cast<T*>(nvrhi::GetDefaultMemAllocator()->AllocateAligned(count * sizeof(T), alignof(T) < 16 ? 16 : alignof(T)));
            if (!data) throw std::bad_alloc();
            const size_t kept = std::min(count, m_Size);
            if (kept) std::memcpy(data, m_Data, kept * sizeof(T));
            std::memset(data + kept, 0, (count - kept) * sizeof(T));
        }
        reset();
        m_Data = data;
        m_Size = count;
    }

    [[nodiscard]] T* data() { return m_Data; }
    [[nodiscard]] const T* data() const { return m_Data; }
    [[nodiscard]] size_t size() const { return m_Size; }
    [[nodiscard]] bool empty() const { return m_Size == 0; }
    T& operator[](size_t i) { return m_Data[i]; }
    const T& operator[](size_t i) const { return m_Data[i]; }

 private:
    void reset() {
        if (m_Data) nvrhi::GetDefaultMemAllocator()->FreeAligned(m_Data);
        m_Data = nullptr;
        m_Size = 0;
    }

    T* m_Data = nullptr;
    size_t m_Size = 0;
};

template <typename T>
T* dataOf(HostArray<T>& v) {
    return v.empty() ? nullptr : v.data();
}

class PhysContainer;

// ---- IPhysInstance -----------------------------------------------------------------------------------------

NVRHI_CLASS_CLSID(PhysInstance, "e6e88f07-f87e-4495-87c2-9f4fadb80892")
class PhysInstance final : public nvrhi::ObjectImpl<IPhysInstance> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(PhysInstance)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysInstance)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysInstance)
    NVRHI_IMPLEMENTS_CLASS(PhysInstance)
    NVRHI_END_INTERFACE_TABLE()

    PhysInstance(PhysContainer* container, IPhysAsset* asset);
    ~PhysInstance();

    IPhysContainer* getContainer() const noexcept override;
    IPhysAsset* getAsset() const noexcept override { return m_Asset; }
    const int32_t* getParticleIndices() const noexcept override {
        return particleIndices.empty() ? nullptr : particleIndices.data();
    }
    uint32_t getParticleCount() const noexcept override { return numParticles; }
    int32_t getTriangleIndex() const noexcept override { return triangleIndex; }
    int32_t getShapeIndex() const noexcept override { return shapeIndex; }
    int32_t getInflatableIndex() const noexcept override { return inflatableIndex; }
    const float* getShapeTranslations() const noexcept override {
        return shapeTranslations.empty() ? nullptr : shapeTranslations.data();
    }
    const float* getShapeRotations() const noexcept override {
        return shapeRotations.empty() ? nullptr : shapeRotations.data();
    }
    void setUserData(void* data) noexcept override { userData = data; }
    void* getUserData() const noexcept override { return userData; }

    // NvFlexExtInstance, for the container.
    std::vector<int32_t> particleIndices;
    uint32_t numParticles = 0;
    int32_t triangleIndex = -1;
    int32_t shapeIndex = -1;
    int32_t inflatableIndex = -1;
    std::vector<float> shapeTranslations;  // 3 per shape
    std::vector<float> shapeRotations;     // 4 per shape
    void* userData = nullptr;

 private:
    nvrhi::AutoPtr<PhysContainer> m_Container;
    PhysAssetHandle m_Asset;
};

// ---- IPhysSoftJoint ----------------------------------------------------------------------------------------

NVRHI_CLASS_CLSID(PhysSoftJoint, "693d2579-6f09-42c6-9489-08f43b5aeb4e")
class PhysSoftJoint final : public nvrhi::ObjectImpl<IPhysSoftJoint> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(PhysSoftJoint)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysSoftJoint)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysSoftJoint)
    NVRHI_IMPLEMENTS_CLASS(PhysSoftJoint)
    NVRHI_END_INTERFACE_TABLE()

    explicit PhysSoftJoint(PhysContainer* container);
    ~PhysSoftJoint();

    IPhysContainer* getContainer() const noexcept override;
    const int32_t* getParticleIndices() const noexcept override { return particleIndices.data(); }
    const float* getParticleLocalPositions() const noexcept override { return particleLocalPositions.data(); }
    uint32_t getParticleCount() const noexcept override { return uint32_t(particleIndices.size()); }
    int32_t getShapeIndex() const noexcept override { return shapeIndex; }
    const float* getShapeTranslation() const noexcept override { return shapeTranslations; }
    const float* getShapeRotation() const noexcept override { return shapeRotations; }
    float getStiffness() const noexcept override { return stiffness; }
    bool isInitialized() const noexcept override { return initialized; }
    nvrhi::FRESULT setTransform(const float* position, const float* rotation) noexcept override;

    // NvFlexExtSoftJoint, for the container.
    std::vector<int32_t> particleIndices;
    std::vector<float> particleLocalPositions;  // 3 per particle
    int32_t shapeIndex = -1;
    float shapeTranslations[3] = {0.0f, 0.0f, 0.0f};
    float shapeRotations[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    float stiffness = 0.0f;
    bool initialized = false;

 private:
    nvrhi::AutoPtr<PhysContainer> m_Container;
};

// ---- IPhysContainer ----------------------------------------------------------------------------------------

NVRHI_CLASS_CLSID(PhysContainer, "c2f67ead-eca9-465a-be5f-d7e83d6871ee")
class PhysContainer final : public nvrhi::ObjectImpl<IPhysContainer> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(PhysContainer)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysContainer)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysContainer)
    NVRHI_IMPLEMENTS_CLASS(PhysContainer)
    NVRHI_END_INTERFACE_TABLE()

    PhysContainer(IPhysDevice* device, IPhysSolver* solver, uint32_t maxParticles)
        : m_Device(device), m_Solver(solver), m_Domain(solver->getDomain()), m_MaxParticles(maxParticles) {
        // NvFlexExtCreateContainer: the free list is 0 .. max - 1.
        m_FreeList.resize(maxParticles);
        for (uint32_t i = 0; i < maxParticles; ++i) m_FreeList[i] = int32_t(i);
        m_IsFree.assign(maxParticles, 1);

        m_Particles.resize(size_t(maxParticles) * 4);
        m_ParticlesRest.resize(size_t(maxParticles) * 4);
        m_Velocities.resize(size_t(maxParticles) * 3);
        m_Phases.resize(maxParticles);
        m_Normals.resize(size_t(maxParticles) * 4);
        m_Bounds.resize(6);
        m_Dirty.assign(maxParticles, 0);
        m_ActiveList.reserve(maxParticles);
    }

    IPhysSolver* getSolver() const noexcept override { return m_Solver; }
    IPhysDomain* getDomain() const noexcept override { return m_Domain; }
    uint32_t getMaxParticles() const noexcept override { return m_MaxParticles; }

    // ---- The allocator ----

    uint32_t allocParticles(uint32_t n, int32_t* indices) noexcept override {
        if (n && !indices) {
            fail(nvrhi::FE_INVALID_ARGS, "IPhysContainer::allocParticles: a null array with a count");
            return 0;
        }
        const uint32_t numToAlloc = std::min(uint32_t(m_FreeList.size()), n);
        const size_t start = m_FreeList.size() - numToAlloc;
        if (numToAlloc) {
            std::memcpy(indices, &m_FreeList[start], numToAlloc * sizeof(int32_t));
            for (uint32_t i = 0; i < numToAlloc; ++i) m_IsFree[size_t(indices[i])] = 0;
            m_FreeList.resize(start);
        }
        m_NeedsActiveListRebuild = true;
        return numToAlloc;
    }

    nvrhi::FRESULT freeParticles(uint32_t n, const int32_t* indices) noexcept override {
        if (n && !indices) return fail(nvrhi::FE_INVALID_ARGS, "IPhysContainer::freeParticles: a null array with a count");
        // FleX's debug checks, always, and a duplicate within the call besides (marked 2 while checking).
        auto unmark = [&](uint32_t upTo) {
            for (uint32_t j = 0; j < upTo; ++j) m_IsFree[size_t(indices[j])] = 0;
        };
        for (uint32_t i = 0; i < n; ++i) {
            if (indices[i] < 0 || uint32_t(indices[i]) >= m_MaxParticles) {
                unmark(i);
                return fail(nvrhi::FE_INVALID_ARGS, "IPhysContainer::freeParticles: index %d is out of range", indices[i]);
            }
            const uint8_t state = m_IsFree[size_t(indices[i])];
            if (state) {
                unmark(i);
                return fail(nvrhi::FE_INVALID_ARGS,
                            state == 2 ? "IPhysContainer::freeParticles: particle %d is given twice"
                                       : "IPhysContainer::freeParticles: particle %d is free already",
                            indices[i]);
            }
            m_IsFree[size_t(indices[i])] = 2;
        }
        for (uint32_t i = 0; i < n; ++i) m_IsFree[size_t(indices[i])] = 1;
        try {
            m_FreeList.insert(m_FreeList.end(), indices, indices + n);
        } catch (const std::bad_alloc&) {
            return fail(nvrhi::FE_OUT_OF_MEMORY, "IPhysContainer::freeParticles: out of memory");
        }
        m_NeedsActiveListRebuild = true;
        return nvrhi::FS_OK;
    }

    // NvFlexExtGetActiveList: every particle that is not free, ascending (FleX's bitmap of the free list).
    uint32_t getActiveList(int32_t* indices, uint32_t capacity) noexcept override {
        uint32_t count = 0;
        for (uint32_t i = 0; i < m_MaxParticles; ++i) {
            if (m_IsFree[i]) continue;
            if (indices && count < capacity) indices[count] = int32_t(i);
            ++count;
        }
        return count;
    }

    uint32_t getActiveCount() const noexcept override { return m_MaxParticles - uint32_t(m_FreeList.size()); }

    // ---- The mirrors ----

    ContainerParticleData& mapParticleData(ContainerParticleData& retVal) noexcept override {
        resolvePull();
        m_ParticlesMapped = true;
        retVal.particles = m_Particles.data();
        retVal.restParticles = m_ParticlesRest.data();
        retVal.velocities = m_Velocities.data();
        retVal.phases = m_Phases.data();
        retVal.normals = m_Normals.data();
        retVal.lower = &m_Bounds[0];
        retVal.upper = &m_Bounds[3];
        return retVal;
    }

    void unmapParticleData() noexcept override {
        if (!m_ParticlesMapped) return;
        m_ParticlesMapped = false;
        m_AllDirty = true;  // the application may have written anything
    }

    ContainerTriangleData& mapTriangleData(ContainerTriangleData& retVal) noexcept override {
        resolvePull();
        m_TrianglesMapped = true;
        retVal.indices = dataOf(m_TriangleIndices);
        retVal.normals = dataOf(m_TriangleNormals);
        retVal.numTriangles = uint32_t(m_TriangleIndices.size() / 3);
        return retVal;
    }
    void unmapTriangleData() noexcept override { m_TrianglesMapped = false; }

    ContainerShapeData& mapShapeData(ContainerShapeData& retVal) noexcept override {
        resolvePull();
        m_ShapesMapped = true;
        retVal.rotations = dataOf(m_ShapeRotations);
        retVal.positions = dataOf(m_ShapeTranslations);
        retVal.n = uint32_t(m_ShapeRotations.size() / 4);
        return retVal;
    }
    void unmapShapeData() noexcept override { m_ShapesMapped = false; }

    // ---- Instances and joints ----

    nvrhi::FRESULT createInstance(const ContainerParticleData* particleData, IPhysAsset* asset, const float* transform,
                                  const float* velocity, int32_t phase, float invMassScale,
                                  IPhysInstance** ppInstance) noexcept override {
        if (!ppInstance) return fail(nvrhi::FE_INVALID_ARGS, "IPhysContainer::createInstance: ppInstance is null");
        *ppInstance = nullptr;
        if (!asset || !transform || !velocity)
            return fail(nvrhi::FE_INVALID_ARGS, "IPhysContainer::createInstance: a null asset, transform or velocity");
        if (particleData &&
            (!m_ParticlesMapped || particleData->particles != m_Particles.data() ||
             particleData->restParticles != m_ParticlesRest.data() || particleData->velocities != m_Velocities.data() ||
             particleData->phases != m_Phases.data() || particleData->normals != m_Normals.data()))
            return fail(nvrhi::FE_INVALID_ARGS,
                        "IPhysContainer::createInstance: the particle data is not this container's mapped data "
                        "(mapParticleData), nor null");

        const uint32_t numParticles = asset->getParticleCount();

        // check if asset will fit
        if (m_FreeList.size() < numParticles)
            return fail(nvrhi::FE_OUT_OF_MEMORY,
                        "IPhysContainer::createInstance: the asset has %u particles, the container has %u free",
                        numParticles, uint32_t(m_FreeList.size()));
        resolvePull();

        try {
            nvrhi::AutoPtr<PhysInstance> inst = MAKE_RC_OBJ_PTR(PhysInstance, this, asset);
            inst->particleIndices.resize(numParticles);
            const uint32_t numShapes = asset->getShapeCount();
            inst->shapeTranslations.resize(size_t(numShapes) * 3);
            inst->shapeRotations.resize(size_t(numShapes) * 4);
            m_Instances.reserve(m_Instances.size() + 1);

            // allocate particles for instance
            inst->numParticles = allocParticles(numParticles, inst->particleIndices.data());
            m_Instances.push_back(inst);

            const Matrix44 xform(transform);
            const float* particles = asset->getParticles();
            for (uint32_t i = 0; i < numParticles; ++i) {
                const size_t index = size_t(inst->particleIndices[i]);

                // add transformed particles to the locked particle data
                const Vec4 p = xform * Vec4(Vec3(&particles[i * 4]), 1.0f);
                store4(m_Particles, index, p.x, p.y, p.z, particles[i * 4 + 3] * invMassScale);
                store4(m_ParticlesRest, index, particles[i * 4 + 0], particles[i * 4 + 1], particles[i * 4 + 2],
                       particles[i * 4 + 3]);
                m_Velocities[index * 3 + 0] = velocity[0];
                m_Velocities[index * 3 + 1] = velocity[1];
                m_Velocities[index * 3 + 2] = velocity[2];
                m_Phases[index] = phase;
                store4(m_Normals, index, 0.0f, 0.0f, 0.0f, 0.0f);
                m_Dirty[index] = 1;
                m_AnyDirty = true;
            }

            // shape transforms
            const Quat rotation = QuatFromMatrix(Matrix33(xform.GetAxis(0), xform.GetAxis(1), xform.GetAxis(2)));
            const float* centers = asset->getShapeCenters();
            for (uint32_t i = 0; i < numShapes; ++i) {
                const Vec4 t = xform * Vec4(centers[i * 3 + 0], centers[i * 3 + 1], centers[i * 3 + 2], float(1.0));
                inst->shapeTranslations[i * 3 + 0] = t.x;
                inst->shapeTranslations[i * 3 + 1] = t.y;
                inst->shapeTranslations[i * 3 + 2] = t.z;
                inst->shapeRotations[i * 4 + 0] = rotation.x;
                inst->shapeRotations[i * 4 + 1] = rotation.y;
                inst->shapeRotations[i * 4 + 2] = rotation.z;
                inst->shapeRotations[i * 4 + 3] = rotation.w;
            }

            // mark container as dirty
            m_NeedsCompact = true;
            m_NeedsActiveListRebuild = true;
            *ppInstance = inst.Detach();
            return nvrhi::FS_OK;
        } catch (const std::bad_alloc&) {
            return fail(nvrhi::FE_OUT_OF_MEMORY, "IPhysContainer::createInstance: out of memory");
        }
    }

    void notifyAssetChanged(IPhysAsset*) noexcept override { m_NeedsCompact = true; }

    nvrhi::FRESULT createSoftJoint(const int32_t* indices, const float* localPositions, uint32_t n, float stiffness,
                                   IPhysSoftJoint** ppJoint) noexcept override {
        if (!ppJoint) return fail(nvrhi::FE_INVALID_ARGS, "IPhysContainer::createSoftJoint: ppJoint is null");
        *ppJoint = nullptr;
        if (n == 0 || !indices || !localPositions)
            return fail(nvrhi::FE_INVALID_ARGS, "IPhysContainer::createSoftJoint: no particles, or a null array");
        for (uint32_t i = 0; i < n; ++i)
            if (indices[i] < 0 || uint32_t(indices[i]) >= m_MaxParticles)
                return fail(nvrhi::FE_INVALID_ARGS, "IPhysContainer::createSoftJoint: index %d is out of range",
                            indices[i]);
        try {
            nvrhi::AutoPtr<PhysSoftJoint> joint = MAKE_RC_OBJ_PTR(PhysSoftJoint, this);
            joint->particleIndices.assign(indices, indices + n);
            joint->particleLocalPositions.assign(localPositions, localPositions + size_t(n) * 3);
            joint->stiffness = stiffness;
            m_SoftJoints.push_back(joint);
            m_NeedsCompact = true;
            *ppJoint = joint.Detach();
            return nvrhi::FS_OK;
        } catch (const std::bad_alloc&) {
            return fail(nvrhi::FE_OUT_OF_MEMORY, "IPhysContainer::createSoftJoint: out of memory");
        }
    }

    uint32_t getInstanceCount() const noexcept override { return uint32_t(m_Instances.size()); }
    uint32_t getSoftJointCount() const noexcept override { return uint32_t(m_SoftJoints.size()); }

    // ---- Device transfers ----

    nvrhi::FRESULT pushToDevice(nvrhi::ICommandList* commandList) noexcept override {
        if (m_NeedsCompact && (m_TrianglesMapped || m_ShapesMapped))
            return fail(nvrhi::FE_INVALID_ARGS,
                        "IPhysContainer::pushToDevice: the container has to compact, and its triangle or shape data "
                        "is mapped (unmap it first: compaction replaces those arrays)");
        try {
            IParticleState* particles = m_Domain->getParticles();
            bool wrote = false;
            nvrhi::FRESULT result = nvrhi::FS_OK;

            if (m_NeedsActiveListRebuild) {
                // update active list
                m_ActiveList.resize(m_MaxParticles);
                const uint32_t n = getActiveList(m_ActiveList.data(), m_MaxParticles);
                if (n) result = m_Device->writeBuffer(commandList, particles->getArray(ParticleArray::ActiveIndex),
                                                      m_ActiveList.data(), 0, n);
                if (NVRHI_FAILED(result)) return result;
                result = particles->setActiveCount(n);
                if (NVRHI_FAILED(result)) return result;
                m_NeedsActiveListRebuild = false;
                wrote = true;
            }

            // push any changes to solver
            if (m_AllDirty || m_ParticlesMapped) {
                result = writeParticles(commandList, 0, m_MaxParticles);
                if (NVRHI_FAILED(result)) return result;
                // While the data is mapped, the application may still write: keep pushing all of it.
                m_AllDirty = m_ParticlesMapped;
                std::fill(m_Dirty.begin(), m_Dirty.end(), uint8_t(0));
                m_AnyDirty = false;
                wrote = true;
            } else if (m_AnyDirty) {
                for (uint32_t i = 0; i < m_MaxParticles;) {
                    if (!m_Dirty[i]) {
                        ++i;
                        continue;
                    }
                    uint32_t end = i;
                    while (end < m_MaxParticles && m_Dirty[end]) m_Dirty[end++] = 0;
                    result = writeParticles(commandList, i, end - i);
                    if (NVRHI_FAILED(result)) return result;
                    i = end;
                }
                m_AnyDirty = false;
                wrote = true;
            }
            if (wrote) particles->markContentChanged();

            if (m_NeedsCompact) return compactObjects(commandList);
            return nvrhi::FS_OK;
        } catch (const std::bad_alloc&) {
            return fail(nvrhi::FE_OUT_OF_MEMORY, "IPhysContainer::pushToDevice: out of memory");
        }
    }

    nvrhi::FRESULT pullFromDevice(nvrhi::ICommandList* commandList) noexcept override {
        IParticleState* particles = m_Domain->getParticles();
        const uint32_t numShapes = uint32_t(m_ShapeRotations.size() / 4);
        const uint32_t numTriangles = uint32_t(m_TriangleIndices.size() / 3);
        nvrhi::FRESULT result = nvrhi::FS_OK;
        auto keep = [&result](nvrhi::FRESULT r) {
            if (NVRHI_SUCCEEDED(result) && NVRHI_FAILED(r)) result = r;
        };

        // read back particle data
        keep(readback(m_RbParticles, m_MaxParticles, 16, "ephys container particles"));
        keep(readback(m_RbVelocities, m_MaxParticles, 12, "ephys container velocities"));
        keep(readback(m_RbPhases, m_MaxParticles, 4, "ephys container phases"));
        keep(readback(m_RbNormals, m_MaxParticles, 16, "ephys container normals"));
        keep(readback(m_RbBounds, 2, 12, "ephys container bounds"));
        if (numShapes) {
            keep(readback(m_RbShapeRotations, numShapes, 16, "ephys container shape rotations"));
            keep(readback(m_RbShapeTranslations, numShapes, 12, "ephys container shape translations"));
        }
        if (numTriangles) keep(readback(m_RbTriangleNormals, numTriangles, 12, "ephys container triangle normals"));
        if (NVRHI_FAILED(result)) return result;

        CopyRegion region;
        region.elementCount = m_MaxParticles;
        keep(particles->download(commandList, ParticleArray::Position, m_RbParticles, &region));
        keep(particles->download(commandList, ParticleArray::Velocity, m_RbVelocities, &region));
        keep(particles->download(commandList, ParticleArray::Phase, m_RbPhases, &region));
        keep(particles->download(commandList, ParticleArray::Normal, m_RbNormals, &region));
        CopyRegion bounds;
        bounds.elementCount = 2;
        keep(m_Device->copyBuffer(commandList, m_RbBounds, particles->getBounds(), bounds));

        // read back shape transforms
        m_PulledShapes = 0;
        if (numShapes && m_Domain->getShapeMatches()->getNumRigids() == numShapes) {
            ShapeMatchBuffers buffers;
            buffers.rotations = m_RbShapeRotations;
            buffers.translations = m_RbShapeTranslations;
            keep(m_Domain->getShapeMatches()->download(commandList, buffers));
            m_PulledShapes = numShapes;
        }
        // and the triangle normals, which FleX did not read back
        m_PulledTriangles = 0;
        if (numTriangles && m_Domain->getTriangles()->getNumTriangles() == numTriangles) {
            keep(m_Domain->getTriangles()->download(commandList, nullptr, m_RbTriangleNormals, numTriangles));
            m_PulledTriangles = numTriangles;
        }
        if (NVRHI_SUCCEEDED(result)) m_PullPending = true;
        return result;
    }

    nvrhi::FRESULT tickContainer(nvrhi::ICommandList* commandList, float dt, uint32_t substeps) noexcept override {
        // update the device
        nvrhi::FRESULT result = pushToDevice(commandList);
        if (NVRHI_FAILED(result)) return result;
        // update solver
        result = m_Solver->step(commandList, dt, substeps);
        if (NVRHI_FAILED(result)) return result;
        // update host
        return pullFromDevice(commandList);
    }

    void updateInstances() noexcept override {
        resolvePull();
        const uint32_t numShapes = uint32_t(m_ShapeRotations.size() / 4);

        for (PhysInstance* inst : m_Instances) {
            // copy data back to per-instance memory from the container's memory
            const uint32_t instanceShapes = uint32_t(inst->shapeRotations.size() / 4);
            const int32_t shapeStart = inst->shapeIndex;
            if (shapeStart == -1) continue;
            for (uint32_t s = 0; s < instanceShapes && uint32_t(shapeStart) + s < numShapes; ++s) {
                std::memcpy(&inst->shapeTranslations[s * 3], &m_ShapeTranslations[(shapeStart + s) * 3], 12);
                std::memcpy(&inst->shapeRotations[s * 4], &m_ShapeRotations[(shapeStart + s) * 4], 16);
            }
        }

        for (PhysSoftJoint* joint : m_SoftJoints) {
            const int32_t shapeStart = joint->shapeIndex;
            const uint32_t count = uint32_t(joint->particleIndices.size());

            // Here we compute the COM only once instead of in NvFlexExtCreateSoftJoint() to avoid buffer mapping
            // issue
            if (!joint->initialized) {
                // To improve the accuracy of the result, first transform the particlePosition to relative coordinates
                // (by finding the mean and subtracting that from all positions)
                Vec3 shapeOffset(0.0f);
                for (uint32_t i = 0; i < count; ++i) shapeOffset += particleAt(joint->particleIndices[i]);
                shapeOffset /= float(count);

                Vec3 com;
                for (uint32_t i = 0; i < count; ++i) com += particleAt(joint->particleIndices[i]) - shapeOffset;
                com /= float(count);

                // Add the shapeOffset to switch back to absolute coordinates
                com += shapeOffset;

                joint->shapeTranslations[0] = com.x;
                joint->shapeTranslations[1] = com.y;
                joint->shapeTranslations[2] = com.z;
                joint->initialized = true;
            } else if (shapeStart >= 0 && uint32_t(shapeStart) < numShapes) {
                std::memcpy(joint->shapeTranslations, &m_ShapeTranslations[size_t(shapeStart) * 3], 12);
            }

            // copy data back to per-joint memory from the container's memory (FleX read shape 0 of the
            // container, or past its end, for a joint not compacted yet)
            if (shapeStart >= 0 && uint32_t(shapeStart) < numShapes)
                std::memcpy(joint->shapeRotations, &m_ShapeRotations[size_t(shapeStart) * 4], 16);
        }
    }

    // ---- For the instances and joints ----

    void removeInstance(PhysInstance* inst) noexcept {
        // NvFlexExtDestroyInstance: the particles go back to the free list.
        for (uint32_t i = 0; i < inst->numParticles; ++i) m_IsFree[size_t(inst->particleIndices[i])] = 1;
        m_FreeList.insert(m_FreeList.end(), inst->particleIndices.begin(),
                          inst->particleIndices.begin() + inst->numParticles);
        auto it = std::find(m_Instances.begin(), m_Instances.end(), inst);
        if (it != m_Instances.end()) m_Instances.erase(it);
        m_NeedsCompact = true;
        m_NeedsActiveListRebuild = true;
    }

    void removeSoftJoint(PhysSoftJoint* joint) noexcept {
        auto it = std::find(m_SoftJoints.begin(), m_SoftJoints.end(), joint);
        if (it != m_SoftJoints.end()) m_SoftJoints.erase(it);
        m_NeedsCompact = true;
    }

    // NvFlexExtSoftJointSetTransform, on the particle mirrors.
    nvrhi::FRESULT setJointTransform(PhysSoftJoint* joint, const float* newPosition, const float* newRotation) noexcept {
        if (!newPosition || !newRotation)
            return fail(nvrhi::FE_INVALID_ARGS, "IPhysSoftJoint::setTransform: a null pointer");
        resolvePull();

        // calculate transform from old position to new position
        const Matrix44 LocalFromOld = AffineInverse(TranslationMatrix(Vec3(joint->shapeTranslations)) *
                                                    RotationMatrix(Quat(joint->shapeRotations)));
        const Matrix44 NewFromLocal = TranslationMatrix(Vec3(newPosition)) * RotationMatrix(Quat(newRotation));
        const Matrix44 transform = NewFromLocal * LocalFromOld;

        // transform soft joint particles to new location
        for (int32_t index : joint->particleIndices) {
            const Vec3 particlePosition = particleAt(index);
            const Vec4 particleNewPostion = transform * Vec4(particlePosition, 1.0f);

            // update soft joint particles
            m_Particles[size_t(index) * 4 + 0] = particleNewPostion.x;
            m_Particles[size_t(index) * 4 + 1] = particleNewPostion.y;
            m_Particles[size_t(index) * 4 + 2] = particleNewPostion.z;
            m_Dirty[size_t(index)] = 1;
            m_AnyDirty = true;
        }

        std::memcpy(joint->shapeTranslations, newPosition, 12);
        std::memcpy(joint->shapeRotations, newRotation, 16);
        return nvrhi::FS_OK;
    }

 private:
    static void store4(HostArray<float>& v, size_t index, float x, float y, float z, float w) {
        v[index * 4 + 0] = x;
        v[index * 4 + 1] = y;
        v[index * 4 + 2] = z;
        v[index * 4 + 3] = w;
    }

    Vec3 particleAt(int32_t index) const { return Vec3(&m_Particles[size_t(index) * 4]); }

    nvrhi::FRESULT fail(nvrhi::FRESULT code, const char* format, ...) const noexcept {
        char text[512];
        va_list args;
        va_start(args, format);
        vsnprintf(text, sizeof(text), format, args);
        va_end(args);
        report(m_Device->getDesc().messageCallback, PhysMessageSeverity::Error, "%s", text);
        return code;
    }

    // The five particle arrays, particles [first, first + count).
    nvrhi::FRESULT writeParticles(nvrhi::ICommandList* commandList, uint32_t first, uint32_t count) {
        IParticleState* particles = m_Domain->getParticles();
        struct Array {
            ParticleArray array;
            const void* data;
            uint32_t stride;
        };
        const Array arrays[] = {
            {ParticleArray::Position, m_Particles.data(), 16},  {ParticleArray::RestPosition, m_ParticlesRest.data(), 16},
            {ParticleArray::Velocity, m_Velocities.data(), 12}, {ParticleArray::Phase, m_Phases.data(), 4},
            {ParticleArray::Normal, m_Normals.data(), 16},
        };
        for (const Array& a : arrays) {
            const nvrhi::FRESULT result =
                m_Device->writeBuffer(commandList, particles->getArray(a.array),
                                      static_cast<const uint8_t*>(a.data) + size_t(first) * a.stride, first, count);
            if (NVRHI_FAILED(result)) return result;
        }
        return nvrhi::FS_OK;
    }

    // A Readback buffer of at least count elements.
    nvrhi::FRESULT readback(PhysBufferHandle& buffer, uint32_t count, uint32_t stride, const char* name) {
        if (buffer && buffer->getElementCount() >= count) return nvrhi::FS_OK;
        PhysBufferDesc desc;
        desc.elementCount = std::max(count, 1u);
        desc.elementStride = stride;
        desc.use = BufferUse::Readback;
        desc.debugName = name;
        buffer = nullptr;
        return m_Device->createBuffer(desc, &buffer);
    }

    // An Upload buffer with the contents of an array (count elements of stride bytes); null for count 0.
    nvrhi::FRESULT upload(const void* data, uint32_t count, uint32_t stride, const char* name, PhysBufferHandle& out) {
        out = nullptr;
        if (count == 0) return nvrhi::FS_OK;
        PhysBufferDesc desc;
        desc.elementCount = count;
        desc.elementStride = stride;
        desc.use = BufferUse::Upload;
        desc.debugName = name;
        nvrhi::FRESULT result = m_Device->createBuffer(desc, &out);
        if (NVRHI_FAILED(result)) return result;
        void* memory = nullptr;
        result = out->map(&memory);
        if (NVRHI_FAILED(result) || !memory) return NVRHI_FAILED(result) ? result : nvrhi::FE_GENERIC_ERROR;
        std::memcpy(memory, data, size_t(count) * stride);
        return out->unmap();
    }

    // The copies of the last pull into the mirrors, after the GPU has made them.
    void resolvePull() noexcept {
        if (!m_PullPending) return;
        m_PullPending = false;
        if (NVRHI_FAILED(m_Device->waitForIdle())) return;
        auto copyFrom = [](IPhysBuffer* buffer, void* dst, size_t bytes) {
            void* memory = nullptr;
            if (!buffer || NVRHI_FAILED(buffer->map(&memory)) || !memory) return;
            std::memcpy(dst, memory, bytes);
            buffer->unmap();
        };
        copyFrom(m_RbParticles, m_Particles.data(), size_t(m_MaxParticles) * 16);
        copyFrom(m_RbVelocities, m_Velocities.data(), size_t(m_MaxParticles) * 12);
        copyFrom(m_RbPhases, m_Phases.data(), size_t(m_MaxParticles) * 4);
        copyFrom(m_RbNormals, m_Normals.data(), size_t(m_MaxParticles) * 16);
        copyFrom(m_RbBounds, m_Bounds.data(), 24);
        // A compaction since the pull has replaced the layout of these: the pulled values are stale then.
        if (m_PulledShapes && m_PulledShapes * 4 == m_ShapeRotations.size()) {
            copyFrom(m_RbShapeRotations, m_ShapeRotations.data(), size_t(m_PulledShapes) * 16);
            copyFrom(m_RbShapeTranslations, m_ShapeTranslations.data(), size_t(m_PulledShapes) * 12);
        }
        if (m_PulledTriangles && size_t(m_PulledTriangles) * 3 == m_TriangleNormals.size())
            copyFrom(m_RbTriangleNormals, m_TriangleNormals.data(), size_t(m_PulledTriangles) * 12);
    }

    // FleX's CompactObjects: the constraints of every instance (remapped to its particles) and joint, flattened,
    // and uploaded to the domain's constraint sets.
    nvrhi::FRESULT compactObjects(nvrhi::ICommandList* commandList) {
        resolvePull();
        m_PulledShapes = 0;
        m_PulledTriangles = 0;

        // An asset whose indices reach beyond its instance's particles (a tearing cloth that grew) is left out.
        std::vector<uint8_t> usable(m_Instances.size(), 1);
        for (size_t k = 0; k < m_Instances.size(); ++k) {
            const PhysInstance* inst = m_Instances[k];
            IPhysAsset* asset = inst->getAsset();
            auto inRange = [&](const int32_t* indices, size_t count) {
                for (size_t i = 0; i < count; ++i)
                    if (indices[i] < 0 || uint32_t(indices[i]) >= inst->numParticles) return false;
                return true;
            };
            const bool ok = inRange(asset->getSpringIndices(), size_t(asset->getSpringCount()) * 2) &&
                            inRange(asset->getShapeIndices(), asset->getShapeIndexCount()) &&
                            inRange(asset->getTriangleIndices(), size_t(asset->getTriangleCount()) * 3) &&
                            (asset->getShapeCount() == 0 ||
                             uint32_t(asset->getShapeOffsets()[asset->getShapeCount() - 1]) <= asset->getShapeIndexCount());
            if (!ok) {
                usable[k] = 0;
                fail(nvrhi::FE_INVALID_ARGS,
                     "IPhysContainer: an asset of an instance refers to particles beyond the %u the instance was "
                     "created with; its constraints are left out",
                     inst->numParticles);
            }
        }

        uint32_t totalNumSprings = 0, totalNumTris = 0, totalNumShapes = 0, totalNumShapeIndices = 0;
        bool plasticDeformation = false;

        // pre-calculate array sizes
        for (size_t k = 0; k < m_Instances.size(); ++k) {
            PhysInstance* inst = m_Instances[k];
            IPhysAsset* asset = inst->getAsset();

            // index into the triangle array for this instance
            inst->triangleIndex = int32_t(totalNumTris);
            if (!usable[k]) continue;

            totalNumSprings += asset->getSpringCount();
            totalNumTris += asset->getTriangleCount();
            totalNumShapeIndices += asset->getShapeIndexCount();
            totalNumShapes += asset->getShapeCount();

            if (asset->getShapePlasticThresholds() && asset->getShapePlasticCreeps()) plasticDeformation = true;
        }

        // each joint corresponds to one shape matching constraint
        for (const PhysSoftJoint* joint : m_SoftJoints) {
            totalNumShapeIndices += uint32_t(joint->particleIndices.size());
            ++totalNumShapes;
        }

        // springs
        std::vector<int32_t> springIndices(size_t(totalNumSprings) * 2);
        std::vector<float> springLengths(totalNumSprings), springCoefficients(totalNumSprings);
        // cloth
        m_TriangleIndices.resize(size_t(totalNumTris) * 3);
        m_TriangleNormals.resize(size_t(totalNumTris) * 3);
        // inflatables
        std::vector<InflatableDesc> inflatables;
        // shapes
        std::vector<int32_t> shapeIndices(totalNumShapeIndices);
        std::vector<float> shapeRestPositions(size_t(totalNumShapeIndices) * 3);
        std::vector<int32_t> shapeOffsets(size_t(totalNumShapes) + 1, 0);
        std::vector<float> shapeCoefficients(totalNumShapes);
        std::vector<float> shapePlasticThresholds(plasticDeformation ? totalNumShapes : 0);
        std::vector<float> shapePlasticCreeps(plasticDeformation ? totalNumShapes : 0);
        m_ShapeTranslations.resize(size_t(totalNumShapes) * 3);
        m_ShapeRotations.resize(size_t(totalNumShapes) * 4);

        size_t springOut = 0, triOut = 0;
        uint32_t shapeIndexOffset = 0;
        uint32_t shapeIndex = 0;

        // go through each instance and update springs, shapes, etc
        for (size_t k = 0; k < m_Instances.size(); ++k) {
            PhysInstance* inst = m_Instances[k];
            inst->shapeIndex = -1;
            inst->inflatableIndex = -1;
            if (!usable[k]) continue;
            IPhysAsset* asset = inst->getAsset();

            // map indices from the asset to the instance
            const int32_t* remap = inst->particleIndices.data();

            // flatten spring data
            const uint32_t numSprings = asset->getSpringCount();
            const int32_t* srcSpringIndices = asset->getSpringIndices();
            for (uint32_t i = 0; i < numSprings * 2; ++i) springIndices[springOut * 2 + i] = remap[srcSpringIndices[i]];
            if (numSprings) {
                std::memcpy(&springLengths[springOut], asset->getSpringRestLengths(), numSprings * sizeof(float));
                std::memcpy(&springCoefficients[springOut], asset->getSpringCoefficients(), numSprings * sizeof(float));
            }
            springOut += numSprings;

            // shapes
            const uint32_t numShapes = asset->getShapeCount();
            if (numShapes) {
                const uint32_t indexOffset = shapeIndexOffset;

                // store start index into shape array
                inst->shapeIndex = int32_t(shapeIndex);

                int32_t shapeStart = 0;
                const int32_t* offsets = asset->getShapeOffsets();
                const int32_t* indices = asset->getShapeIndices();
                const float* particles = asset->getParticles();
                const float* centers = asset->getShapeCenters();
                for (uint32_t s = 0; s < numShapes; ++s) {
                    shapeOffsets[1 + shapeIndex] = offsets[s] + int32_t(indexOffset);
                    shapeCoefficients[shapeIndex] = asset->getShapeCoefficients()[s];
                    if (plasticDeformation) {
                        shapePlasticThresholds[shapeIndex] =
                            asset->getShapePlasticThresholds() ? asset->getShapePlasticThresholds()[s] : 0.0f;
                        shapePlasticCreeps[shapeIndex] =
                            asset->getShapePlasticCreeps() ? asset->getShapePlasticCreeps()[s] : 0.0f;
                    }
                    std::memcpy(&m_ShapeTranslations[size_t(shapeIndex) * 3], &inst->shapeTranslations[s * 3], 12);
                    std::memcpy(&m_ShapeRotations[size_t(shapeIndex) * 4], &inst->shapeRotations[s * 4], 16);

                    ++shapeIndex;

                    const int32_t shapeEnd = offsets[s];
                    for (int32_t i = shapeStart; i < shapeEnd; ++i) {
                        const int32_t currentParticle = indices[i];

                        // remap indices and create local space positions for each shape
                        const Vec3 rest =
                            Vec3(&particles[size_t(currentParticle) * 4]) - Vec3(&centers[size_t(s) * 3]);
                        shapeRestPositions[size_t(shapeIndexOffset) * 3 + 0] = rest.x;
                        shapeRestPositions[size_t(shapeIndexOffset) * 3 + 1] = rest.y;
                        shapeRestPositions[size_t(shapeIndexOffset) * 3 + 2] = rest.z;
                        shapeIndices[shapeIndexOffset] = remap[indices[i]];

                        ++shapeIndexOffset;
                    }
                    shapeStart = shapeEnd;
                }
            }

            const uint32_t numTriangles = asset->getTriangleCount();
            if (numTriangles) {
                // triangles
                const int32_t* srcTriIndices = asset->getTriangleIndices();
                for (uint32_t i = 0; i < numTriangles * 3; ++i) m_TriangleIndices[triOut++] = remap[srcTriIndices[i]];

                if (asset->isInflatable()) {
                    InflatableDesc inflatable;
                    inflatable.startTri = inst->triangleIndex;
                    inflatable.numTris = int32_t(numTriangles);
                    inflatable.restVolume = asset->getInflatableVolume();
                    inflatable.overPressure = asset->getInflatablePressure();
                    inflatable.constraintScale = asset->getInflatableStiffness();
                    inst->inflatableIndex = int32_t(inflatables.size());
                    inflatables.push_back(inflatable);
                }
            }
        }

        // go through each joint and add shape matching constraint to the solver
        for (PhysSoftJoint* joint : m_SoftJoints) {
            const uint32_t numJointParticles = uint32_t(joint->particleIndices.size());

            // store start index into shape array
            joint->shapeIndex = int32_t(shapeIndex);

            const int32_t offset = shapeOffsets[shapeIndex];
            shapeOffsets[1 + shapeIndex] = offset + int32_t(numJointParticles);

            for (uint32_t i = 0; i < numJointParticles; ++i) {
                shapeIndices[shapeIndexOffset] = joint->particleIndices[i];
                std::memcpy(&shapeRestPositions[size_t(shapeIndexOffset) * 3], &joint->particleLocalPositions[i * 3], 12);
                ++shapeIndexOffset;
            }

            std::memcpy(&m_ShapeTranslations[size_t(shapeIndex) * 3], joint->shapeTranslations, 12);
            std::memcpy(&m_ShapeRotations[size_t(shapeIndex) * 4], joint->shapeRotations, 16);
            shapeCoefficients[shapeIndex] = joint->stiffness;

            ++shapeIndex;
        }

        // ---- the domain's constraint sets ----
        nvrhi::FRESULT result = nvrhi::FS_OK;

        // springs
        {
            PhysBufferHandle indices, lengths, coefficients;
            result = upload(springIndices.data(), totalNumSprings * 2, 4, "ephys container springs", indices);
            if (NVRHI_SUCCEEDED(result))
                result = upload(springLengths.data(), totalNumSprings, 4, "ephys container spring lengths", lengths);
            if (NVRHI_SUCCEEDED(result))
                result = upload(springCoefficients.data(), totalNumSprings, 4, "ephys container spring coefficients",
                                coefficients);
            if (NVRHI_SUCCEEDED(result))
                result = m_Domain->getSprings()->upload(commandList, indices, lengths, coefficients, totalNumSprings);
            if (NVRHI_FAILED(result)) return result;
        }

        // shapes
        {
            ShapeMatchBuffers buffers;
            PhysBufferHandle offsets, indices, rest, coefficients, thresholds, creeps, rotations, translations;
            if (totalNumShapes) {
                result = upload(shapeOffsets.data(), totalNumShapes + 1, 4, "ephys container shape offsets", offsets);
                if (NVRHI_SUCCEEDED(result))
                    result = upload(shapeIndices.data(), totalNumShapeIndices, 4, "ephys container shape indices", indices);
                if (NVRHI_SUCCEEDED(result))
                    result = upload(shapeRestPositions.data(), totalNumShapeIndices, 12, "ephys container shape rest", rest);
                if (NVRHI_SUCCEEDED(result))
                    result = upload(shapeCoefficients.data(), totalNumShapes, 4, "ephys container shape coefficients",
                                    coefficients);
                if (NVRHI_SUCCEEDED(result) && plasticDeformation) {
                    result = upload(shapePlasticThresholds.data(), totalNumShapes, 4, "ephys container plastic thresholds",
                                    thresholds);
                    if (NVRHI_SUCCEEDED(result))
                        result = upload(shapePlasticCreeps.data(), totalNumShapes, 4, "ephys container plastic creeps",
                                        creeps);
                }
                if (NVRHI_SUCCEEDED(result))
                    result = upload(m_ShapeRotations.data(), totalNumShapes, 16, "ephys container shape rotations",
                                    rotations);
                if (NVRHI_SUCCEEDED(result))
                    result = upload(m_ShapeTranslations.data(), totalNumShapes, 12, "ephys container shape translations",
                                    translations);
                if (NVRHI_FAILED(result)) return result;
                buffers.offsets = offsets;
                buffers.indices = indices;
                buffers.restPositions = rest;
                buffers.stiffness = coefficients;
                buffers.plasticThresholds = thresholds;
                buffers.plasticCreeps = creeps;
                buffers.rotations = rotations;
                buffers.translations = translations;
            }
            result = m_Domain->getShapeMatches()->upload(commandList, buffers, totalNumShapes, totalNumShapeIndices);
            if (NVRHI_FAILED(result)) return result;
        }

        // triangles
        {
            PhysBufferHandle indices;
            result = upload(m_TriangleIndices.data(), totalNumTris * 3, 4, "ephys container triangles", indices);
            if (NVRHI_SUCCEEDED(result))
                result = m_Domain->getTriangles()->upload(commandList, indices, nullptr, totalNumTris);
            if (NVRHI_FAILED(result)) return result;
        }

        // inflatables
        result = m_Domain->getInflatables()->upload(commandList, inflatables.empty() ? nullptr : inflatables.data(),
                                                    uint32_t(inflatables.size()));
        if (NVRHI_FAILED(result)) return result;

        m_NeedsCompact = false;
        return nvrhi::FS_OK;
    }

    PhysDeviceHandle m_Device;
    PhysSolverHandle m_Solver;
    IPhysDomain* m_Domain;  // the solver keeps it alive
    uint32_t m_MaxParticles;

    // the allocator
    std::vector<int32_t> m_FreeList;
    std::vector<uint8_t> m_IsFree;  // 1: on the free list
    std::vector<int32_t> m_ActiveList;
    std::vector<PhysInstance*> m_Instances;  // in creation order; an instance removes itself when it goes
    std::vector<PhysSoftJoint*> m_SoftJoints;

    // the mirrors
    HostArray<float> m_Particles, m_ParticlesRest, m_Velocities, m_Normals, m_Bounds;
    HostArray<int32_t> m_Phases;
    HostArray<int32_t> m_TriangleIndices;
    HostArray<float> m_TriangleNormals;
    HostArray<float> m_ShapeRotations, m_ShapeTranslations;

    // what the next push uploads
    std::vector<uint8_t> m_Dirty;
    bool m_AnyDirty = false;
    bool m_AllDirty = true;  // the mirrors replace the domain's particles on the first push

    bool m_NeedsCompact = false;
    bool m_NeedsActiveListRebuild = false;
    bool m_ParticlesMapped = false, m_TrianglesMapped = false, m_ShapesMapped = false;

    // the last pull
    PhysBufferHandle m_RbParticles, m_RbVelocities, m_RbPhases, m_RbNormals, m_RbBounds;
    PhysBufferHandle m_RbShapeRotations, m_RbShapeTranslations, m_RbTriangleNormals;
    uint32_t m_PulledShapes = 0, m_PulledTriangles = 0;
    bool m_PullPending = false;
};

PhysInstance::PhysInstance(PhysContainer* container, IPhysAsset* asset) : m_Container(container), m_Asset(asset) {}
PhysInstance::~PhysInstance() {
    if (m_Container) m_Container->removeInstance(this);
}
IPhysContainer* PhysInstance::getContainer() const noexcept { return m_Container.Get(); }

PhysSoftJoint::PhysSoftJoint(PhysContainer* container) : m_Container(container) {}
PhysSoftJoint::~PhysSoftJoint() {
    if (m_Container) m_Container->removeSoftJoint(this);
}
IPhysContainer* PhysSoftJoint::getContainer() const noexcept { return m_Container.Get(); }
nvrhi::FRESULT PhysSoftJoint::setTransform(const float* position, const float* rotation) noexcept {
    return m_Container->setJointTransform(this, position, rotation);
}

}  // namespace

nvrhi::FRESULT detail::createContainer(IPhysDevice* device, IPhysSolver* solver, uint32_t maxParticles,
                                       IPhysContainer** ppContainer) noexcept {
    if (!ppContainer) return nvrhi::FE_INVALID_ARGS;
    *ppContainer = nullptr;
    IPhysMessageCallback* callback = device->getDesc().messageCallback;
    const uint32_t capacity = solver->getDomain()->getParticles()->getMaxParticles();
    if (maxParticles == 0 || maxParticles > capacity) {
        report(callback, PhysMessageSeverity::Error,
               "createContainer: maxParticles %u is not in 1 .. %u (the particle capacity of the domain)", maxParticles,
               capacity);
        return nvrhi::FE_INVALID_ARGS;
    }
    try {
        *ppContainer = MAKE_RC_OBJ(PhysContainer, device, solver, maxParticles);
        return nvrhi::FS_OK;
    } catch (const std::bad_alloc&) {
        report(callback, PhysMessageSeverity::Error, "createContainer: out of memory");
        return nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        report(callback, PhysMessageSeverity::Error, "createContainer: unexpected error");
        return nvrhi::FE_GENERIC_ERROR;
    }
}

}  // namespace ephys
