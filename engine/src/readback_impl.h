#ifndef ETHEREAL_PHYS_SRC_READBACK_IMPL_H
#define ETHEREAL_PHYS_SRC_READBACK_IMPL_H

// Module-private: how the device creates the readback object of a solver (readback.h). The implementation
// class stays in readback.cpp.

#include <ethereal/phys/readback.h>

namespace ephys::detail {

// Creates the IPhysReadback of `solver`, a solver of the calling device (the device checked that). Returns
// FS_OK and one reference in *ppReadback, or an FE_* code and nullptr.
nvrhi::FRESULT createReadback(IPhysSolver* solver, IPhysReadback** ppReadback) noexcept;

}  // namespace ephys::detail

#endif /* ETHEREAL_PHYS_SRC_READBACK_IMPL_H */
