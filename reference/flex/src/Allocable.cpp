#include "Allocable.h"
#include <malloc.h>
#include <exception>
#if NVFLEX__USE_MICROSOFT_VLD
#include <vld.h>
#endif

namespace NvFlex {

static void *(*g_FlexMalloc)(size_t) = malloc;

static void (*g_FlexFree)(void *) = free;

void *Allocable::operator new(size_t count) {
    void *p = malloc(count);
    if (!p) throw std::bad_alloc();
    return p;
}

void *Allocable::operator new[](size_t count) {
    void *p = malloc(count);
    if (!p) throw std::bad_alloc();
    return p;
}

void Allocable::operator delete(void *ptr) {
    free(ptr);
}

void Allocable::operator delete[](void *ptr) {
    free(ptr);
}

void *Allocable::allocate(size_t sz) {
    return malloc(sz);
}

void Allocable::deallocate(void *ptr) {
    free(ptr);
}

void FlexSetMallocFunc(void *(*malloc)(size_t)) {
    g_FlexMalloc = malloc;
}

void FlexSetFreeFunc(void (*free)(void *)) {
    g_FlexFree = free;
}

}  // namespace NvFlex