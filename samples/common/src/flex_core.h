#ifndef ETHEREAL_PHYS_UTILS_SRC_FLEX_CORE_H
#define ETHEREAL_PHYS_UTILS_SRC_FLEX_CORE_H

// Private to ethereal_phys_utils: the part of FleX's core/ math (reference/flex/core/vec3.h, vec4.h, maths.h,
// maths.cpp) that the voxeliser and the distance field below need, with FleX's names and FleX's arithmetic.
//
// A copy of the engine's module-private engine/src/ext/flex_core.h (the engine DLL does not export it, and
// donut::math evaluates in another order): so that ethereal_phys_helpers' CreateParticleShape samples a mesh with
// the bits the FleX demo's helpers.h does. Keep the two copies equal.

#include <cfloat>
#include <cmath>
#include <cstdint>

namespace ephys::utils::detail {

template <typename T>
inline T Min(T a, T b) {
    return a < b ? a : b;
}

template <typename T>
inline T Max(T a, T b) {
    return a > b ? a : b;
}

template <typename T>
inline T Sqr(T x) {
    return x * x;
}

inline float Sqrt(float x) { return sqrtf(x); }
inline float InvSqrt(float x) { return 1.0f / sqrtf(x); }

// XVector3<float>.
struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    Vec3() = default;
    explicit Vec3(float a) : x(a), y(a), z(a) {}
    explicit Vec3(const float* p) : x(p[0]), y(p[1]), z(p[2]) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    float& operator[](int i) { return (&x)[i]; }
    float operator[](int i) const { return (&x)[i]; }

    Vec3 operator*(float scale) const { Vec3 r(*this); r *= scale; return r; }
    Vec3 operator/(float scale) const { Vec3 r(*this); r /= scale; return r; }
    Vec3 operator+(const Vec3& v) const { Vec3 r(*this); r += v; return r; }
    Vec3 operator-(const Vec3& v) const { Vec3 r(*this); r -= v; return r; }
    Vec3 operator*(const Vec3& v) const { Vec3 r(*this); r *= v; return r; }
    Vec3 operator/(const Vec3& v) const { Vec3 r(*this); r /= v; return r; }

    Vec3& operator*=(float scale) { x *= scale; y *= scale; z *= scale; return *this; }
    // FleX: a multiplication by the reciprocal, not three divisions.
    Vec3& operator/=(float scale) { const float s = 1.0f / scale; x *= s; y *= s; z *= s; return *this; }
    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3& operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vec3& operator*=(const Vec3& v) { x *= v.x; y *= v.y; z *= v.z; return *this; }
    Vec3& operator/=(const Vec3& v) { x /= v.x; y /= v.y; z /= v.z; return *this; }

    Vec3 operator-() const { return Vec3(-x, -y, -z); }
};

inline Vec3 operator*(float lhs, const Vec3& rhs) {
    Vec3 r(rhs);
    r *= lhs;
    return r;
}

inline float Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vec3 Cross(const Vec3& b, const Vec3& c) {
    return Vec3(b.y * c.z - b.z * c.y, b.z * c.x - b.x * c.z, b.x * c.y - b.y * c.x);
}

inline Vec3 Min(const Vec3& a, const Vec3& b) { return Vec3(Min(a.x, b.x), Min(a.y, b.y), Min(a.z, b.z)); }
inline Vec3 Max(const Vec3& a, const Vec3& b) { return Vec3(Max(a.x, b.x), Max(a.y, b.y), Max(a.z, b.z)); }

inline float LengthSq(const Vec3& v) { return Dot(v, v); }

inline float Length(const Vec3& v) {
    const float lSq = LengthSq(v);
    if (lSq)
        return Sqrt(LengthSq(v));
    return 0.0f;
}

inline Vec3 Normalize(const Vec3& v) {
    Vec3 a(v);
    a /= Length(v);
    return a;
}

inline Vec3 SafeNormalize(const Vec3& v, const Vec3& fallback = Vec3()) {
    const float l = LengthSq(v);
    if (l > 0.0f)
        return v * InvSqrt(l);
    return fallback;
}

// XVector4<float>.
struct Vec4 {
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 0.0f;

    Vec4() = default;
    explicit Vec4(const float* p) : x(p[0]), y(p[1]), z(p[2]), w(p[3]) {}
    Vec4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
    Vec4(const Vec3& v, float w_) : x(v.x), y(v.y), z(v.z), w(w_) {}

    Vec4 operator/(float scale) const { Vec4 r(*this); r /= scale; return r; }
    Vec4 operator+(const Vec4& v) const { Vec4 r(*this); r += v; return r; }

    Vec4& operator/=(float scale) { const float s = 1.0f / scale; x *= s; y *= s; z *= s; w *= s; return *this; }
    Vec4& operator+=(const Vec4& v) { x += v.x; y += v.y; z += v.z; w += v.w; return *this; }
};

// FleX's Vec3(const Vec4&), through the float* conversion of XVector4.
inline Vec3 XYZ(const Vec4& v) { return Vec3(v.x, v.y, v.z); }

// The random generator of maths.h (seed1, seed2, RandInit, Rand, Randf). FleX kept one per module, in two
// globals that start at zero, and a generator at zero stays at zero (Rand() returns 0 until RandInit). Here
// it is an object, owned by an asset builder (ext/asset_builder.h).
struct Random {
    uint32_t seed1 = 0;
    uint32_t seed2 = 0;

    void RandInit() {
        seed1 = 315645664;
        seed2 = seed1 ^ 0x13ab45fe;
    }

    uint32_t Rand() {
        seed1 = (seed2 ^ ((seed1 << 5) | (seed1 >> 27))) ^ (seed1 * seed2);
        seed2 = seed1 ^ ((seed2 << 12) | (seed2 >> 20));
        return seed1;
    }

    // In [0, 1].
    float Randf() {
        const uint32_t value = Rand();
        const uint32_t limit = 0xffffffff;
        return float(value) * (1.0f / float(limit));
    }
};

// maths.h: Moller and Trumbore's ray-triangle test, both faces. t is along dir; u, v, w are barycentric;
// sign is the dot of -dir with the unnormalised face normal.
inline bool IntersectRayTriTwoSided(const Vec3& p, const Vec3& dir, const Vec3& a, const Vec3& b, const Vec3& c,
                                    float& t, float& u, float& v, float& w, float& sign) {
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 n = Cross(ab, ac);

    const float d = Dot(-dir, n);
    const float ood = 1.0f / d;  // FleX: no check for division by zero, infinity arithmetic takes care of it
    const Vec3 ap = p - a;

    t = Dot(ap, n) * ood;
    if (t < 0.0f)
        return false;

    const Vec3 e = Cross(-dir, ap);
    v = Dot(ac, e) * ood;
    if (v < 0.0f || v > 1.0f)
        return false;
    w = -Dot(ab, e) * ood;
    if (w < 0.0f || v + w > 1.0f)
        return false;

    u = 1.0f - v - w;
    sign = d;
    return true;
}

// maths.h: the entry distance of a ray into a box (0 from inside), by the slab of the furthest plane.
inline bool IntersectRayAABB(const Vec3& start, const Vec3& dir, const Vec3& min, const Vec3& max, float& t) {
    float tx = -1.0f, ty = -1.0f, tz = -1.0f;
    bool inside = true;

    if (start.x < min.x) {
        if (dir.x != 0.0f)
            tx = (min.x - start.x) / dir.x;
        inside = false;
    } else if (start.x > max.x) {
        if (dir.x != 0.0f)
            tx = (max.x - start.x) / dir.x;
        inside = false;
    }

    if (start.y < min.y) {
        if (dir.y != 0.0f)
            ty = (min.y - start.y) / dir.y;
        inside = false;
    } else if (start.y > max.y) {
        if (dir.y != 0.0f)
            ty = (max.y - start.y) / dir.y;
        inside = false;
    }

    if (start.z < min.z) {
        if (dir.z != 0.0f)
            tz = (min.z - start.z) / dir.z;
        inside = false;
    } else if (start.z > max.z) {
        if (dir.z != 0.0f)
            tz = (max.z - start.z) / dir.z;
        inside = false;
    }

    if (inside) {
        t = 0.0f;
        return true;
    }

    // The maximum of the candidate planes is the entry point.
    float tmax = tx;
    int taxis = 0;
    if (ty > tmax) {
        tmax = ty;
        taxis = 1;
    }
    if (tz > tmax) {
        tmax = tz;
        taxis = 2;
    }

    if (tmax < 0.0f)
        return false;

    // The hit must lie on the plane picked; the axis of that plane is not tested, for precision.
    const Vec3 hit = start + dir * tmax;
    if ((hit.x < min.x || hit.x > max.x) && taxis != 0)
        return false;
    if ((hit.y < min.y || hit.y > max.y) && taxis != 1)
        return false;
    if ((hit.z < min.z || hit.z > max.z) && taxis != 2)
        return false;

    t = tmax;
    return true;
}

}  // namespace ephys::utils::detail

#endif /* ETHEREAL_PHYS_UTILS_SRC_FLEX_CORE_H */
