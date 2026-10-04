#include "data_model.h"

#include <nvrhi/core/foundation.h>

#include <new>

// IPhysDomain: the parts and the parameter block. It owns no buffer of its own and dispatches nothing.

namespace ephys {
namespace {

NVRHI_CLASS_CLSID(PhysDomain, "a401ef80-0112-4ec4-8980-f5150198683e")
class PhysDomain final : public nvrhi::ObjectImpl<IPhysDomain> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(PhysDomain)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysDomain)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysDomain)
    NVRHI_IMPLEMENTS_CLASS(PhysDomain)
    NVRHI_END_INTERFACE_TABLE()

    PhysDomain(IPhysDevice* device, const DomainDesc& desc)
        : m_Device(device), m_Desc(desc) {}

    nvrhi::FRESULT init() {
        nvrhi::FRESULT result = detail::createParticleState(m_Device, m_Desc.maxParticles, &m_Particles);
        if (NVRHI_SUCCEEDED(result) && m_Desc.maxDiffuseParticles != 0)
            result =
                detail::createDiffuseParticleState(m_Device, m_Desc.maxDiffuseParticles, &m_DiffuseParticles);
        if (NVRHI_SUCCEEDED(result))
            result = detail::createSpringSet(m_Device, m_Desc.reserveSprings, &m_Springs);
        if (NVRHI_SUCCEEDED(result))
            result = detail::createShapeMatchSet(m_Device, m_Desc.reserveRigids, m_Desc.reserveRigidIndices,
                                                 &m_ShapeMatches);
        if (NVRHI_SUCCEEDED(result))
            result = detail::createTriangleSet(m_Device, m_Desc.reserveTriangles, &m_Triangles);
        if (NVRHI_SUCCEEDED(result))
            result = detail::createInflatableSet(m_Device, m_Desc.reserveInflatables, &m_Inflatables);
        if (NVRHI_SUCCEEDED(result))
            result = detail::createCollisionShapeSet(m_Device, m_Desc.reserveShapes, &m_Shapes);
        return result;
    }

    const DomainDesc& getDesc() const noexcept override { return m_Desc; }
    IPhysDevice* getDevice() const noexcept override { return m_Device; }
    IParticleState* getParticles() const noexcept override { return m_Particles; }
    IDiffuseParticleState* getDiffuseParticles() const noexcept override { return m_DiffuseParticles; }
    ISpringSet* getSprings() const noexcept override { return m_Springs; }
    IShapeMatchSet* getShapeMatches() const noexcept override { return m_ShapeMatches; }
    ITriangleSet* getTriangles() const noexcept override { return m_Triangles; }
    IInflatableSet* getInflatables() const noexcept override { return m_Inflatables; }
    ICollisionShapeSet* getShapes() const noexcept override { return m_Shapes; }

    void setParams(const PhysParams& params) noexcept override {
        m_Params = params;
        ++m_ParamsRevision;
    }
    const PhysParams& getParams() const noexcept override { return m_Params; }
    uint64_t getParamsRevision() const noexcept override { return m_ParamsRevision; }

 private:
    PhysDeviceHandle m_Device;
    DomainDesc m_Desc;
    ParticleStateHandle m_Particles;
    DiffuseParticleStateHandle m_DiffuseParticles;
    SpringSetHandle m_Springs;
    ShapeMatchSetHandle m_ShapeMatches;
    TriangleSetHandle m_Triangles;
    InflatableSetHandle m_Inflatables;
    CollisionShapeSetHandle m_Shapes;
    PhysParams m_Params;
    uint64_t m_ParamsRevision = 0;
};

}  // namespace

namespace detail {

nvrhi::FRESULT createDomain(IPhysDevice* device, const DomainDesc& desc, IPhysDomain** ppDomain) noexcept {
    if (!ppDomain)
        return nvrhi::FE_INVALID_ARGS;
    *ppDomain = nullptr;

    IPhysMessageCallback* callback = device->getDesc().messageCallback;
    if (desc.maxParticles == 0) {
        report(callback, PhysMessageSeverity::Error, "createDomain: maxParticles is 0");
        return nvrhi::FE_INVALID_ARGS;
    }
    // The solver works on 16-byte elements with 32-bit element arithmetic.
    if (desc.maxParticles > c_MaxArrayCount || desc.maxDiffuseParticles > c_MaxArrayCount) {
        report(callback, PhysMessageSeverity::Error,
               "createDomain: %u particles and %u diffuse particles are too many", desc.maxParticles,
               desc.maxDiffuseParticles);
        return nvrhi::FE_INVALID_ARGS;
    }

    try {
        nvrhi::AutoPtr<PhysDomain> domain = nvrhi::TakeOver(MAKE_RC_OBJ(PhysDomain, device, desc));
        // A part that fails has said why.
        const nvrhi::FRESULT result = domain->init();
        if (NVRHI_FAILED(result))
            return result;
        *ppDomain = domain.Detach();
        return nvrhi::FS_OK;
    } catch (const std::bad_alloc&) {
        report(callback, PhysMessageSeverity::Error, "createDomain: out of memory");
        return nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        return nvrhi::FE_GENERIC_ERROR;
    }
}

}  // namespace detail
}  // namespace ephys
