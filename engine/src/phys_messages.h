#ifndef ETHEREAL_PHYS_SRC_PHYS_MESSAGES_H
#define ETHEREAL_PHYS_SRC_PHYS_MESSAGES_H

// Module-private: formats a message and hands it to the message callback of a device.

#include <ethereal/phys/device.h>

#include <cstdarg>
#include <cstdio>

namespace ephys::detail {

inline void report(IPhysMessageCallback* callback, PhysMessageSeverity severity, const char* format, ...) noexcept {
    if (!callback) return;
    char text[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    callback->message(severity, text);
}

}  // namespace ephys::detail

#endif /* ETHEREAL_PHYS_SRC_PHYS_MESSAGES_H */
