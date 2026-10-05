#include "readback_impl.h"

#include <ethereal/phys/device.h>

#include "phys_messages.h"

#include <nvrhi/core/foundation.h>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <new>
#include <vector>

// IPhysReadback: FleX's Solver::GetSmoothParticles, GetDensities, GetAnisotropy, GetDiffuseParticles,
// GetContacts, GetNeighbors and GetBounds (FleX/src/Solver.cpp), over the public interfaces of the
// solver (getOutput, getNeighborStride, getSubstepCount, getDesc) and of its domain, recording
// IPhysDevice::copyBuffer.
//
// Each getter copies what FleX copies, from the buffer FleX copies from, so the order of the elements is
// FleX's: no remapping kernel is involved, because FleX returns its internal-order arrays as they are and
// hands out the maps instead (GetContacts' `indices` is its reverse lookup, the API-to-internal map).
//
// What is different from the reference, and why:
//
// - Every getter takes a region (FleX: only the per-particle getters, with an NvFlexCopyDesc). Without one
//   a destination receives as much as it holds, up to what the source has; FleX copied fixed counts.
// - GetNeighbors copied maxParticles * maxContactsPerParticle elements of the neighbour lists (a slip in the
//   reference: the lists have maxNeighborsPerParticle columns). Here the lists are copied whole.
// - A destination that does not fit is refused with a message; FleX's CopyBufferImpl copied past it.

namespace ephys {
namespace {

using detail::report;

// A copy the call will record, once every copy of the call was checked.
struct Copy {
    IPhysBuffer* dst = nullptr;
    IPhysBuffer* src = nullptr;
    CopyRegion region;
};

NVRHI_CLASS_CLSID(PhysReadback, "fd9149b5-8e3a-4511-b726-5f76179fae0c")
class PhysReadback final : public nvrhi::ObjectImpl<IPhysReadback> {
 public:
    NVRHI_DECLARE_UUID_TRAITS(PhysReadback)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(PhysReadback)
    NVRHI_IMPLEMENTS_INTERFACE(IPhysReadback)
    NVRHI_IMPLEMENTS_CLASS(PhysReadback)
    NVRHI_END_INTERFACE_TABLE()

    explicit PhysReadback(IPhysSolver* solver)
        : m_Solver(solver), m_Domain(solver->getDomain()), m_Device(m_Domain->getDevice()) {}

    IPhysSolver* getSolver() const noexcept override { return m_Solver; }

    // Solver::GetSmoothParticles: the particle positions until the first step (FleX's mStepCounter is 0).
    nvrhi::FRESULT readSmoothedPositions(nvrhi::ICommandList* commandList, IPhysBuffer* positions,
                                         const CopyRegion* region) noexcept override {
        return guarded([&]() -> nvrhi::FRESULT {
            Call call(*this, "readSmoothedPositions");
            if (!positions) return call.fail(nvrhi::FE_INVALID_ARGS, "the destination is null");
            IPhysBuffer* source = m_Solver->getSubstepCount() != 0
                                      ? m_Solver->getOutput(SolverOutput::SmoothedPositions)
                                      : m_Domain->getParticles()->getArray(ParticleArray::Position);
            call.perParticle("positions", positions, source, 16, particles(), region);
            return call.record(commandList);
        });
    }

    // Solver::GetDensities.
    nvrhi::FRESULT readDensities(nvrhi::ICommandList* commandList, IPhysBuffer* densities,
                                 const CopyRegion* region) noexcept override {
        return guarded([&]() -> nvrhi::FRESULT {
            Call call(*this, "readDensities");
            if (!densities) return call.fail(nvrhi::FE_INVALID_ARGS, "the destination is null");
            call.perParticle("densities", densities, m_Solver->getOutput(SolverOutput::Densities), 4, particles(), region);
            return call.record(commandList);
        });
    }

    // Solver::GetAnisotropy: one region for the three axes, as FleX's one NvFlexCopyDesc.
    nvrhi::FRESULT readAnisotropy(nvrhi::ICommandList* commandList, IPhysBuffer* q1, IPhysBuffer* q2, IPhysBuffer* q3,
                                  const CopyRegion* region) noexcept override {
        return guarded([&]() -> nvrhi::FRESULT {
            Call call(*this, "readAnisotropy");
            if (!q1 && !q2 && !q3) return call.fail(nvrhi::FE_INVALID_ARGS, "every destination is null");
            call.perParticle("q1", q1, m_Solver->getOutput(SolverOutput::Anisotropy1), 16, particles(), region);
            call.perParticle("q2", q2, m_Solver->getOutput(SolverOutput::Anisotropy2), 16, particles(), region);
            call.perParticle("q3", q3, m_Solver->getOutput(SolverOutput::Anisotropy3), 16, particles(), region);
            return call.record(commandList);
        });
    }

    // Solver::GetDiffuseParticles: nothing without diffuse particles (FleX tests mMaxDiffuseParticles).
    nvrhi::FRESULT readDiffuseParticles(nvrhi::ICommandList* commandList, IPhysBuffer* positions, IPhysBuffer* velocities,
                                        IPhysBuffer* count, const CopyRegion* region) noexcept override {
        return guarded([&]() -> nvrhi::FRESULT {
            Call call(*this, "readDiffuseParticles");
            if (!positions && !velocities && !count) return call.fail(nvrhi::FE_INVALID_ARGS, "every destination is null");
            IDiffuseParticleState* diffuse = m_Domain->getDiffuseParticles();
            if (!diffuse) return nvrhi::FS_OK;
            const uint32_t capacity = diffuse->getMaxParticles();
            call.perParticle("positions", positions, diffuse->getPositions(), 16, capacity, region);
            call.perParticle("velocities", velocities, diffuse->getVelocities(), 16, capacity, region);
            call.single("count", count, diffuse->getCountBuffer(), 4, 0);
            return call.record(commandList);
        });
    }

    // Solver::GetContacts: the planes, velocities and counts in internal order, and the reverse lookup (the
    // API-to-internal map) as the indices.
    nvrhi::FRESULT readContacts(nvrhi::ICommandList* commandList, IPhysBuffer* planes, IPhysBuffer* velocities,
                                IPhysBuffer* indices, IPhysBuffer* counts, const CopyRegion* region) noexcept override {
        return guarded([&]() -> nvrhi::FRESULT {
            Call call(*this, "readContacts");
            if (!planes && !velocities && !indices && !counts) return call.fail(nvrhi::FE_INVALID_ARGS, "every destination is null");
            const uint32_t contacts = m_Solver->getDesc().maxContactsPerParticle;
            call.perParticle("planes", planes, m_Solver->getOutput(SolverOutput::ContactPlanes), 16, particles(), region,
                             contacts);
            call.perParticle("velocities", velocities, m_Solver->getOutput(SolverOutput::ContactVelocities), 16, particles(),
                             region, contacts);
            call.perParticle("indices", indices, m_Solver->getOutput(SolverOutput::ApiToInternal), 4, particles(), region);
            call.perParticle("counts", counts, m_Solver->getOutput(SolverOutput::ContactCounts), 4, particles(), region);
            return call.record(commandList);
        });
    }

    // Solver::GetNeighbors.
    nvrhi::FRESULT readNeighbors(nvrhi::ICommandList* commandList, IPhysBuffer* neighbors, IPhysBuffer* counts,
                                 IPhysBuffer* apiToInternal, IPhysBuffer* internalToApi,
                                 const CopyRegion* region) noexcept override {
        return guarded([&]() -> nvrhi::FRESULT {
            Call call(*this, "readNeighbors");
            if (!neighbors && !counts && !apiToInternal && !internalToApi)
                return call.fail(nvrhi::FE_INVALID_ARGS, "every destination is null");
            call.neighborLists(neighbors, m_Solver->getOutput(SolverOutput::Neighbors), m_Solver->getNeighborStride(),
                               m_Solver->getDesc().maxNeighborsPerParticle, region);
            call.perParticle("counts", counts, m_Solver->getOutput(SolverOutput::NeighborCounts), 4, particles(), region);
            call.perParticle("apiToInternal", apiToInternal, m_Solver->getOutput(SolverOutput::ApiToInternal), 4, particles(),
                             region);
            // Typed (R32_UINT) beside the structured buffers above; a copy does not care.
            call.perParticle("internalToApi", internalToApi, m_Solver->getOutput(SolverOutput::InternalToApi), 4, particles(),
                             region);
            return call.record(commandList);
        });
    }

    // Solver::GetBounds: element 0 of the bounds is the lower corner, element 1 the upper.
    nvrhi::FRESULT readBounds(nvrhi::ICommandList* commandList, IPhysBuffer* lower, IPhysBuffer* upper) noexcept override {
        return guarded([&]() -> nvrhi::FRESULT {
            Call call(*this, "readBounds");
            if (!lower && !upper) return call.fail(nvrhi::FE_INVALID_ARGS, "every destination is null");
            IPhysBuffer* bounds = m_Domain->getParticles()->getBounds();
            call.single("lower", lower, bounds, 12, 0);
            call.single("upper", upper, bounds, 12, 1);
            return call.record(commandList);
        });
    }

 private:
    // Runs the body of a method; a call that cannot allocate its list of copies records nothing.
    template <typename Body>
    static nvrhi::FRESULT guarded(const Body& body) noexcept {
        try {
            return body();
        } catch (const std::bad_alloc&) {
            return nvrhi::FE_OUT_OF_MEMORY;
        } catch (...) {
            return nvrhi::FE_GENERIC_ERROR;
        }
    }

    // The particle capacity: every per-particle output has this many meaningful elements.
    uint32_t particles() const { return m_Domain->getParticles()->getMaxParticles(); }

    // The copies of one call: checked first, recorded only when all of them are right.
    class Call {
     public:
        Call(const PhysReadback& owner, const char* method) : m_Owner(owner), m_Method(method) {}

        nvrhi::FRESULT fail(nvrhi::FRESULT code, const char* format, ...) {
            char text[512];
            va_list args;
            va_start(args, format);
            vsnprintf(text, sizeof(text), format, args);
            va_end(args);
            report(m_Owner.m_Device->getDesc().messageCallback, PhysMessageSeverity::Error, "IPhysReadback::%s: %s",
                   m_Method, text);
            if (NVRHI_SUCCEEDED(m_Result)) m_Result = code;
            return code;
        }

        // A per-particle array of `particles` meaningful elements, `perParticle` elements per particle.
        void perParticle(const char* what, IPhysBuffer* dst, IPhysBuffer* src, uint32_t stride, uint32_t particles,
                         const CopyRegion* region, uint32_t perParticle = 1) {
            if (!dst || NVRHI_FAILED(m_Result)) return;
            if (!src) {
                fail(nvrhi::FE_GENERIC_ERROR, "%s: the solver has no such output", what);
                return;
            }
            if (!checkDestination(what, dst, stride)) return;
            const uint64_t available = std::min<uint64_t>(src->getElementCount(), uint64_t(particles) * perParticle);
            Copy copy;
            copy.dst = dst;
            copy.src = src;
            if (!region) {
                copy.region.elementCount = uint32_t(std::min<uint64_t>(dst->getElementCount(), available));
            } else {
                const uint64_t srcBegin = uint64_t(region->srcOffset) * perParticle;
                const uint64_t dstBegin = uint64_t(region->dstOffset) * perParticle;
                const uint64_t count = uint64_t(region->elementCount) * perParticle;
                if (srcBegin + count > available) {
                    fail(nvrhi::FE_INVALID_ARGS, "%s: the region (particles %u to %u) leaves the source (%u particles)",
                         what, region->srcOffset, region->srcOffset + region->elementCount, particles);
                    return;
                }
                if (dstBegin + count > dst->getElementCount()) {
                    fail(nvrhi::FE_INVALID_ARGS, "%s: the region needs %llu elements, the destination has %u", what,
                         static_cast<unsigned long long>(dstBegin + count), dst->getElementCount());
                    return;
                }
                copy.region.srcOffset = uint32_t(srcBegin);
                copy.region.dstOffset = uint32_t(dstBegin);
                copy.region.elementCount = uint32_t(count);
            }
            m_Copies.push_back(copy);
        }

        // One element of src to element 0 of dst.
        void single(const char* what, IPhysBuffer* dst, IPhysBuffer* src, uint32_t stride, uint32_t element) {
            if (!dst || NVRHI_FAILED(m_Result)) return;
            if (!checkDestination(what, dst, stride)) return;
            if (dst->getElementCount() < 1) {
                fail(nvrhi::FE_INVALID_ARGS, "%s: the destination has no element", what);
                return;
            }
            Copy copy;
            copy.dst = dst;
            copy.src = src;
            copy.region.srcOffset = element;
            copy.region.elementCount = 1;
            m_Copies.push_back(copy);
        }

        // The column-wise neighbour lists: neighbour k of internal particle i at element k * stride + i.
        void neighborLists(IPhysBuffer* dst, IPhysBuffer* src, uint32_t stride, uint32_t columns, const CopyRegion* region) {
            if (!dst || NVRHI_FAILED(m_Result)) return;
            if (!checkDestination("neighbors", dst, 4)) return;
            if (!region) {
                Copy copy;
                copy.dst = dst;
                copy.src = src;
                copy.region.elementCount = std::min(dst->getElementCount(), src->getElementCount());
                m_Copies.push_back(copy);
                return;
            }
            if (region->elementCount == 0) return;
            if (uint64_t(region->srcOffset) + region->elementCount > stride) {
                fail(nvrhi::FE_INVALID_ARGS,
                     "neighbors: the region (internal indices %u to %u) leaves the particles of the last step (%u)",
                     region->srcOffset, region->srcOffset + region->elementCount, stride);
                return;
            }
            const uint64_t needed = uint64_t(columns - 1) * stride + region->dstOffset + region->elementCount;
            if (needed > dst->getElementCount()) {
                fail(nvrhi::FE_INVALID_ARGS, "neighbors: the region needs %llu elements (stride %u), the destination has %u",
                     static_cast<unsigned long long>(needed), stride, dst->getElementCount());
                return;
            }
            for (uint32_t k = 0; k < columns; ++k) {
                Copy copy;
                copy.dst = dst;
                copy.src = src;
                copy.region.srcOffset = k * stride + region->srcOffset;
                copy.region.dstOffset = k * stride + region->dstOffset;
                copy.region.elementCount = region->elementCount;
                m_Copies.push_back(copy);
            }
        }

        nvrhi::FRESULT record(nvrhi::ICommandList* commandList) {
            if (NVRHI_FAILED(m_Result)) return m_Result;
            for (const Copy& copy : m_Copies) {
                if (copy.region.elementCount == 0) continue;
                const nvrhi::FRESULT result = m_Owner.m_Device->copyBuffer(commandList, copy.dst, copy.src, copy.region);
                if (NVRHI_FAILED(result)) return result;  // the device said why
            }
            return nvrhi::FS_OK;
        }

     private:
        bool checkDestination(const char* what, IPhysBuffer* dst, uint32_t stride) {
            if (dst->getElementStride() != stride) {
                fail(nvrhi::FE_INVALID_ARGS, "%s: the destination has %u-byte elements, not %u", what, dst->getElementStride(),
                     stride);
                return false;
            }
            if (dst->getDesc().use == BufferUse::Upload) {
                fail(nvrhi::FE_INVALID_ARGS, "%s: an Upload buffer cannot be a copy destination", what);
                return false;
            }
            return true;
        }

        const PhysReadback& m_Owner;
        const char* m_Method;
        nvrhi::FRESULT m_Result = nvrhi::FS_OK;
        std::vector<Copy> m_Copies;
    };

    PhysSolverHandle m_Solver;
    IPhysDomain* m_Domain;  // the solver's: alive while m_Solver is
    IPhysDevice* m_Device;  // the domain's
};

}  // namespace

namespace detail {

nvrhi::FRESULT createReadback(IPhysSolver* solver, IPhysReadback** ppReadback) noexcept {
    if (!ppReadback) return nvrhi::FE_INVALID_ARGS;
    *ppReadback = nullptr;
    try {
        *ppReadback = MAKE_RC_OBJ(PhysReadback, solver);
        return nvrhi::FS_OK;
    } catch (const std::bad_alloc&) {
        return nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        return nvrhi::FE_GENERIC_ERROR;
    }
}

}  // namespace detail
}  // namespace ephys
