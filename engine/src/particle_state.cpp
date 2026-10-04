#include "data_model.h"

#include <nvrhi/core/foundation.h>

#include <algorithm>
#include <new>
#include <vector>

// IParticleState and IDiffuseParticleState. The particle half of FleX's Solver (reference/flex/src/
// Solver.cpp: Init, SetParticles ... GetNormals, SetActive, SetDiffuseParticles, CopyBufferImpl).

namespace ephys {
namespace {

using detail::DataObject;

constexpr uint32_t c_ArrayCount = uint32_t(ParticleArray::Count);

const char* const c_ArrayNames[c_ArrayCount] = {"Position", "Velocity",     "Phase",
                                                "Normal",   "RestPosition", "ActiveIndex"};
const char* const c_ArrayDebugNames[c_ArrayCount] = {
    "ephys particle positions", "ephys particle velocities",     "ephys particle phases",
    "ephys particle normals",   "ephys particle rest positions", "ephys particle active indices"};

NVRHI_CLASS_CLSID(ParticleState, "89ac2efb-6764-44fc-957f-7cd37c1095c6")
class ParticleState final : public nvrhi::ObjectImpl<IParticleState>, private DataObject {
 public:
    NVRHI_DECLARE_UUID_TRAITS(ParticleState)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(ParticleState)
    NVRHI_IMPLEMENTS_INTERFACE(IParticleState)
    NVRHI_IMPLEMENTS_CLASS(ParticleState)
    NVRHI_END_INTERFACE_TABLE()

    ParticleState(IPhysDevice* device, uint32_t maxParticles)
        : DataObject(device), m_MaxParticles(maxParticles) {}

    // Creates and clears the arrays (FleX: CreateWithZero, and the identity in the active indices).
    nvrhi::FRESULT init() {
        const std::vector<uint8_t> zeros(size_t(m_MaxParticles) * 16, 0);
        for (uint32_t i = 0; i < c_ArrayCount; ++i) {
            nvrhi::FRESULT result = createArray(c_ArrayDebugNames[i], strideOf(i), m_MaxParticles, m_Arrays[i]);
            if (NVRHI_FAILED(result))
                return result;
            if (ParticleArray(i) == ParticleArray::ActiveIndex) {
                std::vector<int32_t> identity(m_MaxParticles);
                for (uint32_t k = 0; k < m_MaxParticles; ++k)
                    identity[k] = int32_t(k);
                result = m_Device->writeBuffer(nullptr, m_Arrays[i], identity.data(), 0, m_MaxParticles);
            } else {
                result = m_Device->writeBuffer(nullptr, m_Arrays[i], zeros.data(), 0, m_MaxParticles);
            }
            if (NVRHI_FAILED(result))
                return result;
        }

        nvrhi::FRESULT result = createArray("ephys particle bounds", 12, 2, m_Bounds);
        if (NVRHI_FAILED(result))
            return result;
        return m_Device->writeBuffer(nullptr, m_Bounds, zeros.data(), 0, 2);
    }

    uint32_t getMaxParticles() const noexcept override { return m_MaxParticles; }
    uint32_t getActiveCount() const noexcept override { return m_ActiveCount; }

    nvrhi::FRESULT setActiveCount(uint32_t count) noexcept override {
        if (count > m_MaxParticles)
            return fail(nvrhi::FE_INVALID_ARGS,
                        "IParticleState::setActiveCount: %u particles; the capacity is %u", count,
                        m_MaxParticles);
        m_ActiveCount = count;
        ++m_ContentRevision;
        return nvrhi::FS_OK;
    }

    IPhysBuffer* getArray(ParticleArray array) const noexcept override {
        return uint32_t(array) < c_ArrayCount ? m_Arrays[uint32_t(array)].Get() : nullptr;
    }

    nvrhi::FRESULT setArray(nvrhi::ICommandList* commandList, ParticleArray array,
                            IPhysBuffer* buffer) noexcept override {
        const uint32_t i = uint32_t(array);
        if (i >= c_ArrayCount)
            return fail(nvrhi::FE_INVALID_ARGS, "IParticleState::setArray: not a particle array");
        if (!m_Device->getInfo().supportsRenderSharedBuffers)
            return fail(
                nvrhi::FE_UNSUPPORTED,
                "IParticleState::setArray(%s): this graphics API cannot share a buffer between the kernels "
                "and a renderer; use setRenderArray and updateRenderArrays",
                c_ArrayNames[i]);

        const char* problem = problemOfRenderBuffer(i, buffer);
        if (!problem) {
            const nvrhi::BufferDesc& rhiDesc = buffer->getRHIBuffer()->getDesc();
            if (rhiDesc.structStride != strideOf(i))
                problem = "the struct stride of the buffer is not the element size of the array";
            else if (!rhiDesc.canHaveUAVs)
                problem = "the buffer cannot have UAVs";
            else if (rhiDesc.cpuAccess != nvrhi::CpuAccessMode::None)
                problem = "the buffer is CPU-visible";
        }
        if (problem)
            return fail(nvrhi::FE_INVALID_ARGS, "IParticleState::setArray(%s): %s", c_ArrayNames[i], problem);

        // Already the array (the same buffer through another wrapper counts).
        if (buffer->getRHIBuffer() == m_Arrays[i]->getRHIBuffer())
            return nvrhi::FS_OK;

        const nvrhi::FRESULT result = copy(commandList, buffer, 0, m_Arrays[i], 0, m_MaxParticles);
        if (NVRHI_FAILED(result))
            return result;

        m_Arrays[i] = buffer;
        m_Substituted[i] = true;
        m_RenderArrays[i] = nullptr;  // the array is its own render array now
        ++m_LayoutRevision;
        ++m_ContentRevision;
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT resetArray(nvrhi::ICommandList* commandList, ParticleArray array) noexcept override {
        const uint32_t i = uint32_t(array);
        if (i >= c_ArrayCount)
            return fail(nvrhi::FE_INVALID_ARGS, "IParticleState::resetArray: not a particle array");
        if (!m_Substituted[i])
            return nvrhi::FS_OK;

        PhysBufferHandle own;
        nvrhi::FRESULT result = createArray(c_ArrayDebugNames[i], strideOf(i), m_MaxParticles, own);
        if (NVRHI_FAILED(result))
            return result;
        result = copy(commandList, own, 0, m_Arrays[i], 0, m_MaxParticles);
        if (NVRHI_FAILED(result))
            return result;

        m_Arrays[i] = own;
        m_Substituted[i] = false;
        ++m_LayoutRevision;
        ++m_ContentRevision;
        return nvrhi::FS_OK;
    }

    bool isArraySubstituted(ParticleArray array) const noexcept override {
        return uint32_t(array) < c_ArrayCount && m_Substituted[uint32_t(array)];
    }

    nvrhi::FRESULT upload(nvrhi::ICommandList* commandList, ParticleArray array, IPhysBuffer* hostBuffer,
                          const CopyRegion* region) noexcept override {
        return transfer("upload", commandList, array, hostBuffer, region, true);
    }

    nvrhi::FRESULT download(nvrhi::ICommandList* commandList, ParticleArray array, IPhysBuffer* hostBuffer,
                            const CopyRegion* region) noexcept override {
        return transfer("download", commandList, array, hostBuffer, region, false);
    }

    nvrhi::FRESULT setRenderArray(ParticleArray array, IPhysBuffer* buffer) noexcept override {
        const uint32_t i = uint32_t(array);
        if (i >= c_ArrayCount)
            return fail(nvrhi::FE_INVALID_ARGS, "IParticleState::setRenderArray: not a particle array");
        if (m_Substituted[i])
            return fail(nvrhi::FE_INVALID_ARGS,
                        "IParticleState::setRenderArray(%s): the array is a buffer of the application already "
                        "(setArray)",
                        c_ArrayNames[i]);
        if (!buffer) {
            m_RenderArrays[i] = nullptr;
            return nvrhi::FS_OK;
        }

        const char* problem = problemOfRenderBuffer(i, buffer);
        if (!problem && buffer->getDesc().use == BufferUse::Upload)
            problem = "an Upload buffer cannot be a copy destination";
        if (!problem && buffer->getRHIBuffer() == m_Arrays[i]->getRHIBuffer())
            problem = "the buffer is the array itself";
        if (problem)
            return fail(nvrhi::FE_INVALID_ARGS, "IParticleState::setRenderArray(%s): %s", c_ArrayNames[i],
                        problem);

        m_RenderArrays[i] = buffer;
        return nvrhi::FS_OK;
    }

    IPhysBuffer* getRenderArray(ParticleArray array) const noexcept override {
        const uint32_t i = uint32_t(array);
        if (i >= c_ArrayCount)
            return nullptr;
        return m_Substituted[i] ? m_Arrays[i].Get() : m_RenderArrays[i].Get();
    }

    bool areRenderArraysShared() const noexcept override {
        for (uint32_t i = 0; i < c_ArrayCount; ++i)
            if (m_RenderArrays[i])
                return false;
        return true;
    }

    nvrhi::FRESULT updateRenderArrays(nvrhi::ICommandList* commandList) noexcept override {
        for (uint32_t i = 0; i < c_ArrayCount; ++i) {
            if (!m_RenderArrays[i])
                continue;
            const nvrhi::FRESULT result =
                copy(commandList, m_RenderArrays[i], 0, m_Arrays[i], 0, m_MaxParticles);
            if (NVRHI_FAILED(result))
                return result;
        }
        return nvrhi::FS_OK;
    }

    IPhysBuffer* getBounds() const noexcept override { return m_Bounds; }
    uint64_t getLayoutRevision() const noexcept override { return m_LayoutRevision; }
    uint64_t getContentRevision() const noexcept override { return m_ContentRevision; }
    uint64_t markContentChanged() noexcept override { return ++m_ContentRevision; }

 private:
    static uint32_t strideOf(uint32_t array) { return particleArrayStride(ParticleArray(array)); }

    // What a buffer of the application lacks to stand in for, or to mirror, an array.
    const char* problemOfRenderBuffer(uint32_t array, IPhysBuffer* buffer) const {
        if (!buffer)
            return "the buffer is null";
        if (buffer->getElementStride() != strideOf(array))
            return "the element stride of the buffer is not that of the array";
        if (buffer->getElementCount() < m_MaxParticles)
            return "the buffer holds fewer elements than the capacity";
        return nullptr;
    }

    // FleX's CopyBufferImpl between an array and a buffer of the caller. The array side of the region is
    // limited by the capacity, also where a substituted buffer is larger.
    nvrhi::FRESULT transfer(const char* method, nvrhi::ICommandList* commandList, ParticleArray array,
                            IPhysBuffer* hostBuffer, const CopyRegion* region, bool toDevice) noexcept {
        const uint32_t i = uint32_t(array);
        if (i >= c_ArrayCount)
            return fail(nvrhi::FE_INVALID_ARGS, "IParticleState::%s: not a particle array", method);
        if (!hostBuffer)
            return fail(nvrhi::FE_INVALID_ARGS, "IParticleState::%s(%s): the buffer is null", method,
                        c_ArrayNames[i]);

        CopyRegion all;
        if (!region) {
            all.elementCount = std::min(hostBuffer->getElementCount(), m_MaxParticles);
            region = &all;
        }
        const uint32_t hostOffset = toDevice ? region->srcOffset : region->dstOffset;
        const uint32_t arrayOffset = toDevice ? region->dstOffset : region->srcOffset;

        const char* problem = problemOf(hostBuffer, strideOf(i), hostOffset, region->elementCount, toDevice);
        if (!problem && uint64_t(arrayOffset) + uint64_t(region->elementCount) > m_MaxParticles)
            problem = "the range exceeds the capacity of the particle state";
        if (!problem && hostBuffer->getRHIBuffer() == m_Arrays[i]->getRHIBuffer())
            problem = "the buffer is the array itself";
        if (problem)
            return fail(nvrhi::FE_INVALID_ARGS, "IParticleState::%s(%s): %s", method, c_ArrayNames[i], problem);

        const nvrhi::FRESULT result =
            toDevice
                ? copy(commandList, m_Arrays[i], arrayOffset, hostBuffer, hostOffset, region->elementCount)
                : copy(commandList, hostBuffer, hostOffset, m_Arrays[i], arrayOffset, region->elementCount);
        if (NVRHI_SUCCEEDED(result) && toDevice)
            ++m_ContentRevision;
        return result;
    }

    uint32_t m_MaxParticles;
    uint32_t m_ActiveCount = 0;
    PhysBufferHandle m_Arrays[c_ArrayCount];
    PhysBufferHandle m_RenderArrays[c_ArrayCount];  // setRenderArray: the copy targets of the fallback
    bool m_Substituted[c_ArrayCount] = {};
    PhysBufferHandle m_Bounds;
    uint64_t m_LayoutRevision = 0;
    uint64_t m_ContentRevision = 0;
};

NVRHI_CLASS_CLSID(DiffuseParticleState, "ac3bb05c-c309-42dd-990b-461438ffb35a")
class DiffuseParticleState final : public nvrhi::ObjectImpl<IDiffuseParticleState>, private DataObject {
 public:
    NVRHI_DECLARE_UUID_TRAITS(DiffuseParticleState)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(DiffuseParticleState)
    NVRHI_IMPLEMENTS_INTERFACE(IDiffuseParticleState)
    NVRHI_IMPLEMENTS_CLASS(DiffuseParticleState)
    NVRHI_END_INTERFACE_TABLE()

    DiffuseParticleState(IPhysDevice* device, uint32_t maxParticles)
        : DataObject(device), m_MaxParticles(maxParticles) {}

    nvrhi::FRESULT init() {
        const std::vector<uint8_t> zeros(size_t(m_MaxParticles) * 16, 0);
        nvrhi::FRESULT result = createArray("ephys diffuse positions", 16, m_MaxParticles, m_Positions);
        if (NVRHI_FAILED(result))
            return result;
        result = createArray("ephys diffuse velocities", 16, m_MaxParticles, m_Velocities);
        if (NVRHI_FAILED(result))
            return result;
        result = createArray("ephys diffuse count", 4, 1, m_Count);
        if (NVRHI_FAILED(result))
            return result;

        result = m_Device->writeBuffer(nullptr, m_Positions, zeros.data(), 0, m_MaxParticles);
        if (NVRHI_FAILED(result))
            return result;
        result = m_Device->writeBuffer(nullptr, m_Velocities, zeros.data(), 0, m_MaxParticles);
        if (NVRHI_FAILED(result))
            return result;
        return m_Device->writeBuffer(nullptr, m_Count, zeros.data(), 0, 1);
    }

    uint32_t getMaxParticles() const noexcept override { return m_MaxParticles; }
    IPhysBuffer* getPositions() const noexcept override { return m_Positions; }
    IPhysBuffer* getVelocities() const noexcept override { return m_Velocities; }
    IPhysBuffer* getCountBuffer() const noexcept override { return m_Count; }

    nvrhi::FRESULT upload(nvrhi::ICommandList* commandList, IPhysBuffer* positions, IPhysBuffer* velocities,
                          uint32_t count) noexcept override {
        if (count > m_MaxParticles)
            return fail(nvrhi::FE_INVALID_ARGS,
                        "IDiffuseParticleState::upload: %u particles; the capacity is %u", count,
                        m_MaxParticles);
        if (count != 0) {
            if (const char* problem = problemOf(positions, 16, 0, count, true))
                return fail(nvrhi::FE_INVALID_ARGS, "IDiffuseParticleState::upload: positions: %s", problem);
            if (const char* problem = problemOf(velocities, 16, 0, count, true))
                return fail(nvrhi::FE_INVALID_ARGS, "IDiffuseParticleState::upload: velocities: %s", problem);

            nvrhi::FRESULT result = copy(commandList, m_Positions, 0, positions, 0, count);
            if (NVRHI_FAILED(result))
                return result;
            result = copy(commandList, m_Velocities, 0, velocities, 0, count);
            if (NVRHI_FAILED(result))
                return result;
        }
        const int32_t value = int32_t(count);
        const nvrhi::FRESULT result = m_Device->writeBuffer(commandList, m_Count, &value, 0, 1);
        if (NVRHI_SUCCEEDED(result))
            ++m_ContentRevision;
        return result;
    }

    nvrhi::FRESULT download(nvrhi::ICommandList* commandList, IPhysBuffer* positions, IPhysBuffer* velocities,
                            IPhysBuffer* countBuffer) noexcept override {
        const uint32_t positionCount = positions ? std::min(positions->getElementCount(), m_MaxParticles) : 0;
        const uint32_t velocityCount = velocities ? std::min(velocities->getElementCount(), m_MaxParticles) : 0;
        if (positions)
            if (const char* problem = problemOf(positions, 16, 0, positionCount, false))
                return fail(nvrhi::FE_INVALID_ARGS, "IDiffuseParticleState::download: positions: %s", problem);
        if (velocities)
            if (const char* problem = problemOf(velocities, 16, 0, velocityCount, false))
                return fail(nvrhi::FE_INVALID_ARGS, "IDiffuseParticleState::download: velocities: %s", problem);
        if (countBuffer)
            if (const char* problem = problemOf(countBuffer, 4, 0, 1, false))
                return fail(nvrhi::FE_INVALID_ARGS, "IDiffuseParticleState::download: countBuffer: %s",
                            problem);

        nvrhi::FRESULT result = nvrhi::FS_OK;
        if (positions)
            result = copy(commandList, positions, 0, m_Positions, 0, positionCount);
        if (NVRHI_SUCCEEDED(result) && velocities)
            result = copy(commandList, velocities, 0, m_Velocities, 0, velocityCount);
        if (NVRHI_SUCCEEDED(result) && countBuffer)
            result = copy(commandList, countBuffer, 0, m_Count, 0, 1);
        return result;
    }

    uint64_t getContentRevision() const noexcept override { return m_ContentRevision; }
    uint64_t markContentChanged() noexcept override { return ++m_ContentRevision; }

 private:
    uint32_t m_MaxParticles;
    PhysBufferHandle m_Positions;
    PhysBufferHandle m_Velocities;
    PhysBufferHandle m_Count;
    uint64_t m_ContentRevision = 0;
};

// MAKE_RC_OBJ, then init(); the object is released again when init() fails.
template <typename Class, typename Interface>
nvrhi::FRESULT make(IPhysDevice* device, uint32_t maxParticles, Interface** ppState,
                    const char* what) noexcept {
    if (!ppState)
        return nvrhi::FE_INVALID_ARGS;
    *ppState = nullptr;
    try {
        nvrhi::AutoPtr<Class> state = nvrhi::TakeOver(MAKE_RC_OBJ(Class, device, maxParticles));
        const nvrhi::FRESULT result = state->init();
        if (NVRHI_FAILED(result)) {
            detail::report(device->getDesc().messageCallback, PhysMessageSeverity::Error,
                           "createDomain: cannot create the %s", what);
            return result;
        }
        *ppState = state.Detach();
        return nvrhi::FS_OK;
    } catch (const std::bad_alloc&) {
        return nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        return nvrhi::FE_GENERIC_ERROR;
    }
}

}  // namespace

namespace detail {

nvrhi::FRESULT createParticleState(IPhysDevice* device, uint32_t maxParticles,
                                   IParticleState** ppState) noexcept {
    return make<ParticleState, IParticleState>(device, maxParticles, ppState, "particle state");
}

nvrhi::FRESULT createDiffuseParticleState(IPhysDevice* device, uint32_t maxParticles,
                                          IDiffuseParticleState** ppState) noexcept {
    return make<DiffuseParticleState, IDiffuseParticleState>(device, maxParticles, ppState,
                                                             "diffuse particle state");
}

}  // namespace detail
}  // namespace ephys
