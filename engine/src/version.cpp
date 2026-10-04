#include <ethereal/phys/common/export.h>

#include <cstdint>

// The version of the public headers this library was built against. A client compares it with the
// version in the headers it was compiled with, so that a stale layout is an error and not memory corruption.
ETHEREAL_PHYS_C_API uint32_t etherealPhysGetHeaderVersion() noexcept {
    return 1;
}
