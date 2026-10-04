#ifndef RANDOM_H
#define RANDOM_H
#include "Types.h"
#include "FlexMath.h"

namespace NvFlex {

namespace details {
extern NvFlexUint seed1;
extern NvFlexUint seed2;
}

inline void ResetRandomSeeds(NvFlexUint seed1, NvFlexUint seed2) {
    details::seed1 = seed1;
    details::seed2 = seed2;
}

inline NvFlexUint Rand() {
    using namespace details;
    seed1 = (seed2 * seed1) ^ ((seed1 >> 27) | (32 * seed1)) ^ seed2;
    seed2 = ((seed2 >> 20) | (seed2 << 12)) ^ seed1;
    return seed1;
}

inline float Randf() {
    return (float)Rand() * (1.f / (float)0xffffffffu);
}

inline float Randf(float min, float max) {
    float t = Randf();
    return lerp(min, max, t);
}

}

#endif /* RANDOM_H */
