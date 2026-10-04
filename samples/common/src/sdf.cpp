#include <ethereal/phys_utils/voxelize.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <vector>

// A port of the 3D half of reference/flex/core/sdf.cpp, in FleX's evaluation order: a copy of the engine's
// engine/src/ext/sdf.cpp. The heap of the fast marching is std::make_heap / push_heap / pop_heap over the same
// comparison as FleX's.

namespace ephys::utils {

namespace {

int Clamp(int x, int lower, int upper) { return std::min(std::max(lower, x), upper); }

uint32_t Sample(const uint32_t* image, uint32_t w, uint32_t h, uint32_t d, int x, int y, int z) {
    return image[Clamp(z, 0, int(d - 1)) * int(w * h) + Clamp(y, 0, int(h - 1)) * int(w) + Clamp(x, 0, int(w - 1))];
}

// True when the voxel is on the surface: one of its 26 neighbours (clamped at the border) is on the other
// side. dist: half the distance to the nearest such neighbour.
bool EdgeDetect(const uint32_t* img, uint32_t w, uint32_t h, uint32_t d, int x, int y, int z, float& dist) {
    const bool center = Sample(img, w, h, d, x, y, z) != 0;

    float minDist = FLT_MAX;

    for (int k = z - 1; k <= z + 1; ++k) {
        for (int j = y - 1; j <= y + 1; ++j) {
            for (int i = x - 1; i <= x + 1; ++i) {
                if ((0 != Sample(img, w, h, d, i, j, k)) != center) {
                    const int dx = x - i;
                    const int dy = y - j;
                    const int dz = z - k;

                    minDist = std::min(sqrtf(float(dx * dx + dy * dy + dz * dz)) * 0.5f, minDist);
                }
            }
        }
    }

    dist = minDist;
    return minDist != FLT_MAX;
}

struct Coord3D {
    int i, j, k;
    float d;
    int si, sj, sk;

    // a min-heap on d through std::make_heap / push_heap / pop_heap
    bool operator<(const Coord3D& c) const { return d > c.d; }
};

}  // namespace

void MakeSDF(const uint32_t* img, uint32_t w, uint32_t h, uint32_t d, float* output) {
    const float scale = 1.0f / float(std::max(std::max(w, h), d));

    std::vector<Coord3D> queue;

    // the surface voxels
    for (uint32_t z = 0; z < d; ++z) {
        for (uint32_t y = 0; y < h; ++y) {
            for (uint32_t x = 0; x < w; ++x) {
                float dist;
                if (EdgeDetect(img, w, h, d, int(x), int(y), int(z), dist)) {
                    const Coord3D c = {int(x), int(y), int(z), dist, int(x), int(y), int(z)};
                    queue.push_back(c);
                }

                output[z * w * h + y * w + x] = FLT_MAX;
            }
        }
    }

    // no occupied voxel
    if (queue.empty())
        return;

    std::make_heap(queue.begin(), queue.end());

    while (!queue.empty()) {
        std::pop_heap(queue.begin(), queue.end());

        const Coord3D c = queue.back();
        queue.pop_back();

        // freeze the voxel unless it is frozen already
        if (output[c.k * w * h + c.j * w + c.i] == FLT_MAX) {
            output[c.k * w * h + c.j * w + c.i] = c.d;

            // its neighbours
            const int xmin = std::max(c.i - 1, 0), xmax = std::min(c.i + 1, int(w - 1));
            const int ymin = std::max(c.j - 1, 0), ymax = std::min(c.j + 1, int(h - 1));
            const int zmin = std::max(c.k - 1, 0), zmax = std::min(c.k + 1, int(d - 1));

            for (int z = zmin; z <= zmax; ++z) {
                for (int y = ymin; y <= ymax; ++y) {
                    for (int x = xmin; x <= xmax; ++x) {
                        if ((c.i != x || c.j != y || c.k != z) && output[z * w * h + y * w + x] == FLT_MAX) {
                            const int dx = x - c.si;
                            const int dy = y - c.sj;
                            const int dz = z - c.sk;

                            // the distance to the source voxel
                            const float dd = sqrtf(float(dx * dx + dy * dy + dz * dz)) +
                                             output[c.sk * w * h + c.sj * w + c.si];

                            const Coord3D newc = {x, y, z, dd, c.si, c.sj, c.sk};
                            queue.push_back(newc);
                            std::push_heap(queue.begin(), queue.end());
                        }
                    }
                }
            }
        }
    }

    for (uint32_t z = 0; z < d; ++z) {
        for (uint32_t y = 0; y < h; ++y) {
            for (uint32_t x = 0; x < w; ++x) {
                // the interior is negative
                output[z * w * h + y * w + x] *= (img[z * w * h + y * w + x] ? -1.0f : 1.0f) * scale;
            }
        }
    }
}

}  // namespace ephys::utils
