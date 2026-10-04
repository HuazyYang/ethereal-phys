#include <ethereal/phys/device.h>

// The version of the public headers this library was built against. etherealPhysCreateDevice compares it
// with the version in the headers the caller was compiled with, so that a stale layout is an error and not
// memory corruption.
ETHEREAL_PHYS_C_API uint32_t etherealPhysGetHeaderVersion() noexcept {
    return ephys::c_HeaderVersion;
}
