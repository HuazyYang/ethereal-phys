#ifndef ETHEREAL_PHYS_SRC_PHYS_BUFFER_H
#define ETHEREAL_PHYS_SRC_PHYS_BUFFER_H

// Module-private: how the rest of the engine creates IPhysBuffer objects. The implementation class stays
// in phys_buffer.cpp; everything else, the device included, holds the interface.

#include <ethereal/phys/buffer.h>

#include <nvrhi/nvrhi.h>

namespace ephys::detail {

// Each function returns FS_OK and a new reference in *ppBuffer, or an FE_* code, nullptr in *ppBuffer and
// a static string in *ppReason that says what was wrong.

// Creates the nvrhi buffer for desc (the BufferUse recipes of buffer.h) and wraps it.
nvrhi::FRESULT createPhysBuffer(nvrhi::IDevice* device, const PhysBufferDesc& desc, IPhysBuffer** ppBuffer,
                                const char** ppReason) noexcept;

// Wraps a buffer the application created.
nvrhi::FRESULT adoptPhysBuffer(nvrhi::IDevice* device, nvrhi::IBuffer* buffer, uint32_t elementCount,
                               uint32_t elementStride, IPhysBuffer** ppBuffer, const char** ppReason) noexcept;

}  // namespace ephys::detail

#endif /* ETHEREAL_PHYS_SRC_PHYS_BUFFER_H */
