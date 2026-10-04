#ifndef ETHEREAL_PHYS_DOMAIN_H
#define ETHEREAL_PHYS_DOMAIN_H

// IPhysDomain: everything a simulation is described by. One particle state, optional diffuse particles,
// the four constraint sets, the collision shapes and the simulation parameters.
//
// The rule that organises the engine: anything that owns a buffer is data, anything that dispatches a
// kernel is operations. A domain and its parts are the data half. They own the buffers an application
// fills and reads, and dispatch nothing. A solver is the operations half: it is created for one domain,
// uses only the public interfaces of that domain, and keeps the scratch it derives (the sorted arrays, the
// grid, deltas, contacts, neighbours, half-springs, Lagrange multipliers) where nobody else can see it.
// PBD and XPBD are two solvers over the same domain.
//
// A domain replaces the data half of FleX's NvFlexSolver, which was data and operations in one object.
//
// Do not pattern-match on the gvdb or sph sample interfaces. IGVDBVolume violates the no-duplicate-name
// rule twice, returns aggregates by value repeatedly, is noexcept nowhere, and passes std::vector across
// its interface; ISPHParticles likewise returns dm::box3 by value with no noexcept. Both got away with it
// as in-EXE sample code. ethereal_phys is a DLL and gets no relaxation. Take exactly two things from them,
// the data/ops split and the contracts (revision counters, areRenderBuffersShared, updateRenderBuffers,
// "a solver is created for one particle set and uses only its public interface"), and nothing from their
// method signatures.
//
// The interfaces follow donut/nvrhi/doc/com-interface-abi.md.

#include <ethereal/phys/collision.h>
#include <ethereal/phys/constraints.h>
#include <ethereal/phys/params.h>
#include <ethereal/phys/particles.h>

#include <nvrhi/nvrhi.h>

#include <cstddef>
#include <cstdint>

namespace ephys {

struct IPhysDevice;  // device.h

// The creation-time description of a domain.
//
// The two particle capacities are fixed for the life of the domain, as FleX fixes them at solver creation
// (NvFlexSolverDesc): every per-particle buffer of the domain and of its solvers is that large.
//
// The reserve* members are not limits. The constraint sets and the shape set grow when an upload needs
// more room, as FleX grows them; a reserve allocates the room up front, so that the first uploads replace
// no buffer. 0 allocates nothing until the first upload.
struct DomainDesc {
    uint32_t maxParticles = 0x4000;    // the capacity of the particle state; not 0
    uint32_t maxDiffuseParticles = 0;  // the capacity of the diffuse particles; 0: the domain has none
    uint32_t reserveSprings = 0;       // ISpringSet
    uint32_t reserveRigids = 0;        // IShapeMatchSet: shape-matching constraints
    uint32_t reserveRigidIndices = 0;  // IShapeMatchSet: particle indices over all of them
    uint32_t reserveTriangles = 0;     // ITriangleSet
    uint32_t reserveInflatables = 0;   // IInflatableSet
    uint32_t reserveShapes = 0;        // ICollisionShapeSet
};

static_assert(sizeof(DomainDesc) == 32, "DomainDesc layout changed: bump c_HeaderVersion");
static_assert(offsetof(DomainDesc, maxDiffuseParticles) == 4);
static_assert(offsetof(DomainDesc, reserveShapes) == 28);

// A domain (IPhysDevice::createDomain). Not thread-safe, like the device.
//
// The parts are objects of their own, created with the domain and kept by it: the accessors do not AddRef,
// and return the same object for the life of the domain. A caller may keep its own reference to a part; a
// part keeps the device alive, not the domain.
NVRHI_IID(IPhysDomain, "77cb7ee1-6e93-4526-ace2-fc9294eb43f4")
struct IPhysDomain : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IPhysDomain)

    // The description the domain was created with; the object keeps it (a non-owning view).
    [[nodiscard]] virtual const DomainDesc& getDesc() const noexcept = 0;

    // The device the domain was created by. Does not AddRef: the domain keeps it alive.
    [[nodiscard]] virtual IPhysDevice* getDevice() const noexcept = 0;

    [[nodiscard]] virtual IParticleState* getParticles() const noexcept = 0;

    // nullptr when DomainDesc::maxDiffuseParticles is 0.
    [[nodiscard]] virtual IDiffuseParticleState* getDiffuseParticles() const noexcept = 0;

    [[nodiscard]] virtual ISpringSet* getSprings() const noexcept = 0;
    [[nodiscard]] virtual IShapeMatchSet* getShapeMatches() const noexcept = 0;
    [[nodiscard]] virtual ITriangleSet* getTriangles() const noexcept = 0;
    [[nodiscard]] virtual IInflatableSet* getInflatables() const noexcept = 0;
    [[nodiscard]] virtual ICollisionShapeSet* getShapes() const noexcept = 0;

    // The simulation parameters (NvFlexSetParams / NvFlexGetParams). The domain is their one owner: a
    // solver has no parameters of its own and reads these, so two solvers over one domain cannot disagree.
    // (The XPBD compliance values are not simulation parameters of the domain: only an XPBD solver has
    // them.)
    //
    // getParams returns a view of the block the domain keeps: reading a member copies nothing, and
    // "PhysParams p = domain->getParams();" makes a copy to modify. The defaults are those of PhysParams.
    virtual void setParams(const PhysParams& params) noexcept = 0;
    [[nodiscard]] virtual const PhysParams& getParams() const noexcept = 0;

    // Advances with every setParams. A solver compares it to know when to recompute what it derives from
    // the parameters (the kernel constants, the fluid rest density). Starts at 0.
    [[nodiscard]] virtual uint64_t getParamsRevision() const noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysDomain> PhysDomainHandle;

}  // namespace ephys

#endif /* ETHEREAL_PHYS_DOMAIN_H */
