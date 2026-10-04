#ifndef ETHEREAL_PHYS_COMMON_EXPORT_H
#define ETHEREAL_PHYS_COMMON_EXPORT_H

// Export macros of ethereal_phys (separate from nvrhi's and nvrhi_core's: a different module).
//
// - ETHEREAL_PHYS_BUILD_DLL: compiling ethereal_phys itself (PRIVATE definition).
// - not defined: compiling a module that links ethereal_phys. The engine is always a shared library, so
//   there is no static configuration and no third state.
//
// The library exports only extern "C" functions (ETHEREAL_PHYS_C_API), each one noexcept and prefixed
// etherealPhys, taking and returning scalars, enums and pointers only; aggregates go through out-pointers.
// No C++ member function, constructor or destructor is exported: non-virtual members are inline in the
// public headers, virtual ones are reached through vtables. ETHEREAL_PHYS_API and ETHEREAL_PHYS_C_API are
// never used inside a class scope. See donut/nvrhi/doc/com-interface-abi.md, section 2.
#if defined(ETHEREAL_PHYS_BUILD_DLL)
#if defined(_WIN32)
#define ETHEREAL_PHYS_API __declspec(dllexport)
#else
#define ETHEREAL_PHYS_API __attribute__((visibility("default")))
#endif
#else
#if defined(_WIN32)
#define ETHEREAL_PHYS_API __declspec(dllimport)
#else
#define ETHEREAL_PHYS_API __attribute__((visibility("default")))
#endif
#endif

#define ETHEREAL_PHYS_EXTERN_C extern "C"
#define ETHEREAL_PHYS_C_API ETHEREAL_PHYS_EXTERN_C ETHEREAL_PHYS_API

#endif /* ETHEREAL_PHYS_COMMON_EXPORT_H */
