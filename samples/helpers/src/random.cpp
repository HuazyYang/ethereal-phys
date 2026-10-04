#include <ethereal/phys_helpers/random.h>

#include <algorithm>
#include <cmath>

// A port of the random generator and the sampling functions of reference/flex/core/maths.h (and RandInit of
// maths.cpp). The arithmetic follows FleX's expressions; the vector type is donut::math.

namespace ephys::helpers {

namespace {

// FleX's seed1 / seed2: one generator for the process, as the demo's core library had.
uint32_t g_Seed1 = 0;
uint32_t g_Seed2 = 0;

constexpr float k2Pi = 2.0f * kPi;

}  // namespace

void RandInit() {
    g_Seed1 = 315645664;
    g_Seed2 = g_Seed1 ^ 0x13ab45fe;
}

uint32_t Rand() {
    g_Seed1 = (g_Seed2 ^ ((g_Seed1 << 5) | (g_Seed1 >> 27))) ^ (g_Seed1 * g_Seed2);
    g_Seed2 = g_Seed1 ^ ((g_Seed2 << 12) | (g_Seed2 >> 20));
    return g_Seed1;
}

uint32_t Rand(uint32_t min, uint32_t max) { return min + Rand() % (max - min); }

float Randf() {
    const uint32_t value = Rand();
    const uint32_t limit = 0xffffffff;
    return float(value) * (1.0f / float(limit));
}

float Randf(float min, float max) {
    const float t = Randf();
    return (1.0f - t) * min + t * max;
}

float Randf(float max) { return Randf() * max; }

dm::float3 RandomUnitVector() {
    const float phi = Randf(kPi * 2.0f);
    const float theta = Randf(kPi * 2.0f);

    const float cosTheta = cosf(theta);
    const float sinTheta = sinf(theta);

    const float cosPhi = cosf(phi);
    const float sinPhi = sinf(phi);

    return dm::float3(cosTheta * sinPhi, cosPhi, sinTheta * sinPhi);
}

dm::float3 RandVec3() {
    // FleX draws the three components as the arguments of one constructor call, in an order C++ leaves to the
    // compiler; here the order is fixed (x, y, z), so a sequence of samples may differ from a FleX build's.
    const float x = Randf(-1.0f, 1.0f);
    const float y = Randf(-1.0f, 1.0f);
    const float z = Randf(-1.0f, 1.0f);
    return dm::float3(x, y, z);
}

dm::float3 UniformSampleSphereVolume() {
    for (;;) {
        const dm::float3 v = RandVec3();
        if (dm::dot(v, v) < 1.0f)
            return v;
    }
}

dm::float3 UniformSampleSphere() {
    const float u1 = Randf(0.0f, 1.0f);
    const float u2 = Randf(0.0f, 1.0f);

    const float z = 1.f - 2.f * u1;
    const float r = sqrtf(std::max(0.f, 1.f - z * z));
    const float phi = 2.f * kPi * u2;
    const float x = r * cosf(phi);
    const float y = r * sinf(phi);

    return dm::float3(x, y, z);
}

dm::float3 UniformSampleHemisphere() {
    const float z = Randf(0.0f, 1.0f);
    const float w = sqrtf(1.0f - z * z);

    const float phi = k2Pi * Randf(0.0f, 1.0f);
    const float x = cosf(phi) * w;
    const float y = sinf(phi) * w;

    return dm::float3(x, y, z);
}

dm::float2 UniformSampleDisc() {
    const float r = sqrtf(Randf(0.0f, 1.0f));
    const float theta = k2Pi * Randf(0.0f, 1.0f);

    return dm::float2(r * cosf(theta), r * sinf(theta));
}

int PoissonSample3D(float radius, float separation, dm::float3* points, int maxPoints, int maxAttempts) {
    // naive O(n^2) dart throwing
    int c = 0;
    while (c < maxPoints) {
        int a = 0;
        while (a < maxAttempts) {
            const dm::float3 p = UniformSampleSphereVolume() * radius;

            // test against the points already generated
            int i = 0;
            for (; i < c; ++i) {
                const dm::float3 d = p - points[i];

                // reject when closer than the separation
                if (dm::dot(d, d) < separation * separation)
                    break;
            }

            // the sample passed all tests: accept
            if (i == c) {
                points[c] = p;
                ++c;
                break;
            }

            ++a;
        }

        // stop when a point could not be placed in maxAttempts
        if (a == maxAttempts)
            break;
    }

    return c;
}

int TightPack3D(float radius, float separation, dm::float3* points, int maxPoints) {
    const int dim = int(ceilf(radius / separation));

    int c = 0;

    for (int z = -dim; z <= dim; ++z) {
        for (int y = -dim; y <= dim; ++y) {
            for (int x = -dim; x <= dim; ++x) {
                const float xpos = float(x) * separation + (((y + z) & 1) ? separation * 0.5f : 0.0f);
                const float ypos = float(y) * sqrtf(0.75f) * separation;
                const float zpos = float(z) * sqrtf(0.75f) * separation;

                const dm::float3 p(xpos, ypos, zpos);

                // skip the centre
                if (dm::dot(p, p) == 0.0f)
                    continue;

                if (c < maxPoints && dm::length(p) <= radius) {
                    points[c] = p;
                    ++c;
                }
            }
        }
    }

    return c;
}

}  // namespace ephys::helpers
