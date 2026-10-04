#ifndef ETHEREAL_PHYS_HELPERS_RANDOM_H
#define ETHEREAL_PHYS_HELPERS_RANDOM_H

// The FleX demo's random generator and the sampling functions of core/maths.h its scenes call.
//
// Part of ethereal_phys_helpers (samples/helpers). The generator is FleX's (seed1, seed2; RandInit, Rand,
// Randf) and, as in the demo, one process-wide state: the demo's helpers and scenes drew from the globals of
// its core library, and RandInit() at the start of every scene (FleX's Init) makes a scene's particle jitter,
// random convexes and initial velocities the same on every run. A generator at zero stays at zero: call
// RandInit first. The engine's asset builders have generators of their own (IPhysAssetBuilder), so nothing
// here affects them.
//
// Step 9a of the migration left PoissonSample3D, TightPack3D, UniformSample* and RandomUnitVector for this
// library, with the demo's generator; they are FleX's algorithms in FleX's order of draws.

#include <donut/core/math/math.h>

#include <cstdint>

namespace ephys::helpers {

constexpr float kPi = 3.141592653589793f;

// seed1 = 315645664, seed2 = seed1 ^ 0x13ab45fe.
void RandInit();

uint32_t Rand();
// min + Rand() % (max - min), in [min, max).
uint32_t Rand(uint32_t min, uint32_t max);
// In [0, 1].
float Randf();
// (1 - t) * min + t * max with t = Randf().
float Randf(float min, float max);
// Randf() * max.
float Randf(float max);

// A random unit vector (two random angles, as FleX: not uniform on the sphere).
dm::float3 RandomUnitVector();
// Three components in [-1, 1].
dm::float3 RandVec3();
// Uniform in the unit ball (dart throwing).
dm::float3 UniformSampleSphereVolume();
// Uniform on the unit sphere.
dm::float3 UniformSampleSphere();
// Uniform on the upper (z >= 0) unit hemisphere.
dm::float3 UniformSampleHemisphere();
// Uniform in the unit disc.
dm::float2 UniformSampleDisc();

// Poisson-disc samples in a ball of the given radius: points at least `separation` apart, by dart throwing
// (O(n^2)), until maxPoints or maxAttempts failures in a row. Returns the number written.
int PoissonSample3D(float radius, float separation, dm::float3* points, int maxPoints, int maxAttempts);

// A dense packing (hexagonal layers) of points `separation` apart in a ball of the given radius, without the
// centre. Returns the number written.
int TightPack3D(float radius, float separation, dm::float3* points, int maxPoints);

}  // namespace ephys::helpers

#endif /* ETHEREAL_PHYS_HELPERS_RANDOM_H */
