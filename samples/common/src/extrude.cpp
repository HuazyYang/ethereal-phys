#include <ethereal/phys_utils/extrude.h>

#include <algorithm>
#include <cmath>

// A port of FleX/core/extrude.cpp over Donut's math. FleX's ring frame is a 4x4 matrix of column
// vectors; here it is its three axes, rotated by FleX's RotationMatrix (core/maths.h).
//
// Two differences: FleX computed a "next" direction from points[i - 1] at i = 0, out of the bounds of the
// array, and never used it (that read is gone); and FleX joined the first ring to the last vertices the arrays
// held before the call, when they were not empty (the first ring of a call is joined to nothing here).

namespace ephys::utils {
namespace {

constexpr float k2Pi = 2.0f * 3.141592653589f;  // FleX's

dm::float3 safeNormalize(const dm::float3& v, const dm::float3& fallback) {
    const float l = dm::dot(v, v);
    return l > 0.0f ? v * (1.0f / std::sqrt(l)) : fallback;
}

// FleX's RotationMatrix(angle, axis) applied to v: a right-handed rotation about the normalised axis.
dm::float3 rotate(float angle, const dm::float3& axis, const dm::float3& v) {
    const dm::float3 a = dm::normalize(axis);
    const float s = std::sin(angle);
    const float c = std::cos(angle);

    // the columns of FleX's matrix
    const dm::float3 c0(a.x * a.x + (1.0f - a.x * a.x) * c, a.x * a.y * (1.0f - c) + a.z * s,
                        a.x * a.z * (1.0f - c) - a.y * s);
    const dm::float3 c1(a.x * a.y * (1.0f - c) - a.z * s, a.y * a.y + (1.0f - a.y * a.y) * c,
                        a.y * a.z * (1.0f - c) + a.x * s);
    const dm::float3 c2(a.x * a.z * (1.0f - c) + a.y * s, a.y * a.z * (1.0f - c) - a.x * s,
                        a.z * a.z + (1.0f - a.z * a.z) * c);

    return c0 * v.x + c1 * v.y + c2 * v.z;
}

}  // namespace

dm::float3 HermiteInterpolate(const dm::float3& a, const dm::float3& b, const dm::float3& t1, const dm::float3& t2,
                              float t) {
    const float w1 = 1.0f - 3 * t * t + 2 * t * t * t;
    const float w2 = t * t * (3.0f - 2.0f * t);
    const float w3 = t * t * t - 2 * t * t + t;
    const float w4 = t * t * (t - 1.0f);
    return a * w1 + b * w2 + t1 * w3 + t2 * w4;
}

dm::float3 HermiteTangent(const dm::float3& a, const dm::float3& b, const dm::float3& t1, const dm::float3& t2,
                          float t) {
    const float w1 = 6.0f * t * t - 6 * t;
    const float w2 = -6.0f * t * t + 6 * t;
    const float w3 = 3 * t * t - 4 * t + 1;
    const float w4 = 3 * t * t - 2 * t;
    return a * w1 + b * w2 + t1 * w3 + t2 * w4;
}

dm::float3 HermiteSecondDerivative(const dm::float3& a, const dm::float3& b, const dm::float3& t1,
                                   const dm::float3& t2, float t) {
    const float w1 = 12 * t - 6.0f;
    const float w2 = -12.0f * t + 6;
    const float w3 = 6 * t - 4.0f;
    const float w4 = 6 * t - 2.0f;
    return a * w1 + b * w2 + t1 * w3 + t2 * w4;
}

void BasisFromVector(const dm::float3& w, dm::float3& u, dm::float3& v) {
    if (std::fabs(w.x) > std::fabs(w.y)) {
        const float invLen = 1.0f / std::sqrt(w.x * w.x + w.z * w.z);
        u = dm::float3(-w.z * invLen, 0.0f, w.x * invLen);
    } else {
        const float invLen = 1.0f / std::sqrt(w.y * w.y + w.z * w.z);
        u = dm::float3(0.0f, w.z * invLen, -w.y * invLen);
    }
    v = dm::cross(w, u);
}

void Extrude(const dm::float3* points, int numPoints, std::vector<dm::float3>& vertices,
             std::vector<dm::float3>& normals, std::vector<int>& triangles, float radius, int resolution,
             int smoothing) {
    if (numPoints < 2 || resolution <= 0 || smoothing <= 0)
        return;

    // the frame of the ring: three axes, the third along the curve
    const size_t firstVertex = vertices.size();

    dm::float3 axes[3];
    axes[2] = safeNormalize(points[1] - points[0], dm::float3(0.0f, 1.0f, 0.0f));
    BasisFromVector(axes[2], axes[0], axes[1]);

    for (int i = 0; i < numPoints - 1; ++i) {
        const int a = std::max(i - 1, 0);
        const int b = i;
        const int c = std::min(i + 1, numPoints - 1);
        const int d = std::min(i + 2, numPoints - 1);

        const dm::float3 p1 = points[b];
        const dm::float3 p2 = points[c];
        const dm::float3 m1 = 0.5f * (points[c] - points[a]);
        const dm::float3 m2 = 0.5f * (points[d] - points[b]);

        // the last segment gets one more ring, which ends the tube at the last point
        const int segments = (i < numPoints - 2) ? smoothing : smoothing + 1;

        for (int s = 0; s < segments; ++s) {
            const float t = float(s) / float(smoothing);
            const dm::float3 pos = HermiteInterpolate(p1, p2, m1, m2, t);
            const dm::float3 dir = dm::normalize(HermiteTangent(p1, p2, m1, m2, t));

            const dm::float3 cur = axes[2];
            const float angle = std::acos(dm::dot(cur, dir));

            // the minimal rotation from the previous direction to this one (none when parallel)
            if (std::fabs(angle) > 0.001f) {
                const dm::float3 axis = safeNormalize(dm::cross(cur, dir), dm::float3(0.0f));
                for (dm::float3& column : axes)
                    column = rotate(angle, axis, column);
            }

            const size_t startIndex = vertices.size();

            for (int k = 0; k < resolution; ++k) {
                const float step = k2Pi / float(resolution);

                // the ring vertex and its normal
                const dm::float3 v = axes[0] * std::cos(step * float(k)) + axes[1] * std::sin(step * float(k));

                vertices.push_back(v * radius + pos);
                normals.push_back(v);
            }

            // the triangles to the previous ring
            if (startIndex != firstVertex) {
                for (int k = 0; k < resolution; ++k) {
                    const int curIndex = static_cast<int>(startIndex) + k;
                    const int nextIndex = static_cast<int>(startIndex) + (k + 1) % resolution;

                    triangles.push_back(curIndex);
                    triangles.push_back(curIndex - resolution);
                    triangles.push_back(nextIndex - resolution);

                    triangles.push_back(nextIndex - resolution);
                    triangles.push_back(nextIndex);
                    triangles.push_back(curIndex);
                }
            }
        }
    }
}

}  // namespace ephys::utils
