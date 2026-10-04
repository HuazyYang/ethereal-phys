#ifndef ETHEREAL_PHYS_UTILS_PERLIN_H
#define ETHEREAL_PHYS_UTILS_PERLIN_H

// Perlin noise for the samples (wind gusts, procedural motion): FleX's core/perlin.h. Part of
// ethereal_phys_utils (samples/common), not of the engine.
//
// Each function sums `octaves` octaves of Ken Perlin's improved noise, the first at frequency 1 and amplitude
// 1, each next at twice the frequency and `persistence` times the amplitude.

namespace ephys::utils {

float Perlin1D(float x, int octaves, float persistence);
float Perlin2D(float x, float y, int octaves, float persistence);
float Perlin3D(float x, float y, float z, int octaves, float persistence);

// Periodic in each dimension with the integer periods px, py, pz (as RenderMan's pnoise()), for coordinates that
// are not negative: FleX wraps the lattice with %, which keeps the sign, so a negative coordinate falls into
// another part of the permutation table. Kept as FleX has it.
float Perlin3DPeriodic(float x, float y, float z, int px, int py, int pz, int octaves, float persistence);

}  // namespace ephys::utils

#endif /* ETHEREAL_PHYS_UTILS_PERLIN_H */
