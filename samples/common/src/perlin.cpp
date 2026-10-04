#include <ethereal/phys_utils/perlin.h>

#include <cmath>
#include <cstdint>

// A port of reference/flex/core/perlin.cpp (Ken Perlin's 2002 "improved noise", after pbrt), with FleX's
// arithmetic. The permutation table is stored once and indexed modulo 256 (FleX stored it twice).

namespace ephys::utils {
namespace {

constexpr int kNoisePermSize = 256;

const unsigned char kNoisePerm[kNoisePermSize] = {
    151, 160, 137,  91,  90,  15, 131,  13, 201,  95,  96,  53, 194, 233,   7, 225,
    140,  36, 103,  30,  69, 142,   8,  99,  37, 240,  21,  10,  23, 190,   6, 148,
    247, 120, 234,  75,   0,  26, 197,  62,  94, 252, 219, 203, 117,  35,  11,  32,
     57, 177,  33,  88, 237, 149,  56,  87, 174,  20, 125, 136, 171, 168,  68, 175,
     74, 165,  71, 134, 139,  48,  27, 166,  77, 146, 158, 231,  83, 111, 229, 122,
     60, 211, 133, 230, 220, 105,  92,  41,  55,  46, 245,  40, 244, 102, 143,  54,
     65,  25,  63, 161,   1, 216,  80,  73, 209,  76, 132, 187, 208,  89,  18, 169,
    200, 196, 135, 130, 116, 188, 159,  86, 164, 100, 109, 198, 173, 186,   3,  64,
     52, 217, 226, 250, 124, 123,   5, 202,  38, 147, 118, 126, 255,  82,  85, 212,
    207, 206,  59, 227,  47,  16,  58,  17, 182, 189,  28,  42, 223, 183, 170, 213,
    119, 248, 152,   2,  44, 154, 163,  70, 221, 153, 101, 155, 167,  43, 172,   9,
    129,  22,  39, 253,  19,  98, 108, 110,  79, 113, 224, 232, 178, 185, 112, 104,
    218, 246,  97, 228, 251,  34, 242, 193, 238, 210, 144,  12, 191, 179, 162, 241,
     81,  51, 145, 235, 249,  14, 239, 107,  49, 192, 214,  31, 181, 199, 106, 157,
    184,  84, 204, 176, 115, 121,  50,  45, 127,   4, 150, 254, 138, 236, 205,  93,
    222, 114,  67,  29,  24,  72, 243, 141, 128, 195,  78,  66, 215,  61, 156, 180,
};

unsigned char Perm(int32_t i) { return kNoisePerm[i & (kNoisePermSize - 1)]; }

int32_t Floor2Int(float val) { return int32_t(std::floor(val)); }

float Lerp(float t, float v1, float v2) { return v1 + t * (v2 - v1); }

// zero first and second derivatives at 0 and 1
float PerlinFade(float val) {
    const float val3 = val * val * val;
    const float val4 = val3 * val;
    return 6.0f * val4 * val - 15.0f * val4 + 10.0f * val3;
}

float Grad1d(int32_t x, float dx) {
    const int32_t h = Perm(x) & 3;
    return (h & 1) ? -dx : dx;
}

float Grad2d(int32_t x, int32_t y, float dx, float dy) {
    const int32_t h = Perm(Perm(x) + y) & 3;
    return ((h & 1) ? -dx : dx) + ((h & 2) ? -dy : dy);
}

float Grad3d(int32_t x, int32_t y, int32_t z, float dx, float dy, float dz) {
    const int32_t h = Perm(Perm(Perm(x) + y) + z) & 15;
    const float u = h < 8 ? dx : dy;
    const float v = h < 4 ? dy : (h == 12 || h == 14) ? dx : dz;
    return ((h & 1) ? -u : u) + ((h & 2) ? -v : v);
}

float Trilinear(int32_t ix0, int32_t iy0, int32_t iz0, int32_t ix1, int32_t iy1, int32_t iz1, float dx, float dy,
                float dz) {
    const float w000 = Grad3d(ix0, iy0, iz0, dx, dy, dz);
    const float w100 = Grad3d(ix1, iy0, iz0, dx - 1, dy, dz);
    const float w010 = Grad3d(ix0, iy1, iz0, dx, dy - 1, dz);
    const float w110 = Grad3d(ix1, iy1, iz0, dx - 1, dy - 1, dz);
    const float w001 = Grad3d(ix0, iy0, iz1, dx, dy, dz - 1);
    const float w101 = Grad3d(ix1, iy0, iz1, dx - 1, dy, dz - 1);
    const float w011 = Grad3d(ix0, iy1, iz1, dx, dy - 1, dz - 1);
    const float w111 = Grad3d(ix1, iy1, iz1, dx - 1, dy - 1, dz - 1);
    const float wx = PerlinFade(dx);
    const float wy = PerlinFade(dy);
    const float wz = PerlinFade(dz);
    const float x00 = Lerp(wx, w000, w100);
    const float x10 = Lerp(wx, w010, w110);
    const float x01 = Lerp(wx, w001, w101);
    const float x11 = Lerp(wx, w011, w111);
    const float y0 = Lerp(wy, x00, x10);
    const float y1 = Lerp(wy, x01, x11);
    return Lerp(wz, y0, y1);
}

float PerlinNoise3DFunctionPeriodic(float x, float y, float z, int px, int py, int pz) {
    const int32_t ix = Floor2Int(x);
    const int32_t iy = Floor2Int(y);
    const int32_t iz = Floor2Int(z);
    const float dx = x - float(ix), dy = y - float(iy), dz = z - float(iz);

    const int32_t ix0 = (ix % px) & (kNoisePermSize - 1);
    const int32_t iy0 = (iy % py) & (kNoisePermSize - 1);
    const int32_t iz0 = (iz % pz) & (kNoisePermSize - 1);
    const int32_t ix1 = ((ix + 1) % px) & (kNoisePermSize - 1);
    const int32_t iy1 = ((iy + 1) % py) & (kNoisePermSize - 1);
    const int32_t iz1 = ((iz + 1) % pz) & (kNoisePermSize - 1);

    return Trilinear(ix0, iy0, iz0, ix1, iy1, iz1, dx, dy, dz);
}

float PerlinNoise3DFunction(float x, float y, float z) {
    int32_t ix = Floor2Int(x);
    int32_t iy = Floor2Int(y);
    int32_t iz = Floor2Int(z);
    const float dx = x - float(ix), dy = y - float(iy), dz = z - float(iz);
    ix &= (kNoisePermSize - 1);
    iy &= (kNoisePermSize - 1);
    iz &= (kNoisePermSize - 1);
    return Trilinear(ix, iy, iz, ix + 1, iy + 1, iz + 1, dx, dy, dz);
}

float PerlinNoise2DFunction(float x, float y) {
    int32_t ix = Floor2Int(x);
    int32_t iy = Floor2Int(y);
    const float dx = x - float(ix), dy = y - float(iy);
    ix &= (kNoisePermSize - 1);
    iy &= (kNoisePermSize - 1);
    const float w00 = Grad2d(ix, iy, dx, dy);
    const float w10 = Grad2d(ix + 1, iy, dx - 1.0f, dy);
    const float w01 = Grad2d(ix, iy + 1, dx, dy - 1.0f);
    const float w11 = Grad2d(ix + 1, iy + 1, dx - 1.0f, dy - 1.0f);
    const float wx = PerlinFade(dx);
    const float wy = PerlinFade(dy);
    const float x0 = Lerp(wx, w00, w10);
    const float x1 = Lerp(wx, w01, w11);
    return Lerp(wy, x0, x1);
}

float PerlinNoise1DFunction(float x) {
    int32_t ix = Floor2Int(x);
    const float dx = x - float(ix);
    ix &= (kNoisePermSize - 1);
    const float w00 = Grad1d(ix, dx);
    const float w10 = Grad1d(ix + 1, dx - 1.0f);
    const float wx = PerlinFade(dx);
    return Lerp(wx, w00, w10);
}

}  // namespace

float Perlin1D(float x, int octaves, float persistence) {
    float r = 0.0f;
    float a = 1.0f;
    int freq = 1;
    for (int i = 0; i < octaves; i++) {
        r += PerlinNoise1DFunction(x * float(freq)) * a;
        a *= persistence;
        freq = 2 << i;
    }
    return r;
}

float Perlin2D(float x, float y, int octaves, float persistence) {
    float r = 0.0f;
    float a = 1.0f;
    int freq = 1;
    for (int i = 0; i < octaves; i++) {
        r += PerlinNoise2DFunction(x * float(freq), y * float(freq)) * a;
        a *= persistence;
        freq = 2 << i;
    }
    return r;
}

float Perlin3D(float x, float y, float z, int octaves, float persistence) {
    float r = 0.0f;
    float a = 1.0f;
    int freq = 1;
    for (int i = 0; i < octaves; i++) {
        r += PerlinNoise3DFunction(x * float(freq), y * float(freq), z * float(freq)) * a;
        a *= persistence;
        freq = 2 << i;
    }
    return r;
}

float Perlin3DPeriodic(float x, float y, float z, int px, int py, int pz, int octaves, float persistence) {
    float r = 0.0f;
    float a = 1.0f;
    int freq = 1;
    for (int i = 0; i < octaves; i++) {
        r += PerlinNoise3DFunctionPeriodic(x * float(freq), y * float(freq), z * float(freq), px, py, pz) * a;
        a *= persistence;
        freq = 2 << i;
    }
    return r;
}

}  // namespace ephys::utils
