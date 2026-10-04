#ifndef ETHEREAL_PHYS_FRAMEWORK_FLEX_MATH_H
#define ETHEREAL_PHYS_FRAMEWORK_FLEX_MATH_H

// The view and projection math of the FleX demo: FleX's Matrix44 (core/mat44.h) and the matrix functions of
// core/maths.h it builds the camera and the light from, kept with FleX's conventions.
//
// Part of ethereal_phys_framework (samples/framework). The demo's shaders (samples/framework/shaders) are written
// for these conventions, so the renderer keeps them instead of Donut's:
// - column vectors and column-major storage: columns[c][r] is row r of column c, and a Matrix44 uploaded as it is
//   stored is the matrix the HLSL (column_major by default) multiplies with mul(M, v);
// - a right-handed view space looking down -z (eye-space depths are negative: the fluid shaders rely on it);
// - an OpenGL projection (clip z in [-w, w]); ConvertToD3DProjection maps it to [0, w] for every graphics API
//   of nvrhi (Vulkan included: nvrhi flips the viewport, so clip space is D3D's).

#include <donut/core/math/math.h>

#include <cmath>

namespace ephys::framework {

constexpr float kPi = 3.141592653589793f;

inline float DegToRad(float t) { return t * (kPi / 180.0f); }
inline float RadToDeg(float t) { return t * (180.0f / kPi); }

struct Matrix44 {
    float columns[4][4];

    static Matrix44 Identity() {
        Matrix44 m{};
        m.columns[0][0] = m.columns[1][1] = m.columns[2][2] = m.columns[3][3] = 1.0f;
        return m;
    }

    // From FleX's float[4][4] initialiser lists, which are columns.
    static Matrix44 FromColumns(const float c[4][4]) {
        Matrix44 m;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                m.columns[i][j] = c[i][j];
        return m;
    }

    float operator()(int row, int col) const { return columns[col][row]; }
    float& operator()(int row, int col) { return columns[col][row]; }

    void SetAxis(int i, const dm::float3& a) {
        columns[i][0] = a.x;
        columns[i][1] = a.y;
        columns[i][2] = a.z;
        columns[i][3] = 0.0f;
    }
    void SetTranslation(const dm::float3& t) {
        columns[3][0] = t.x;
        columns[3][1] = t.y;
        columns[3][2] = t.z;
        columns[3][3] = 1.0f;
    }
};

inline Matrix44 operator*(const Matrix44& a, const Matrix44& b) {
    Matrix44 r{};
    for (int c = 0; c < 4; ++c)
        for (int rr = 0; rr < 4; ++rr) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k)
                s += a.columns[k][rr] * b.columns[c][k];
            r.columns[c][rr] = s;
        }
    return r;
}

inline dm::float4 operator*(const Matrix44& m, const dm::float4& v) {
    dm::float4 r;
    r.x = m.columns[0][0] * v.x + m.columns[1][0] * v.y + m.columns[2][0] * v.z + m.columns[3][0] * v.w;
    r.y = m.columns[0][1] * v.x + m.columns[1][1] * v.y + m.columns[2][1] * v.z + m.columns[3][1] * v.w;
    r.z = m.columns[0][2] * v.x + m.columns[1][2] * v.y + m.columns[2][2] * v.z + m.columns[3][2] * v.w;
    r.w = m.columns[0][3] * v.x + m.columns[1][3] * v.y + m.columns[2][3] * v.z + m.columns[3][3] * v.w;
    return r;
}

inline dm::float3 TransformPoint(const Matrix44& m, const dm::float3& p) {
    const dm::float4 r = m * dm::float4(p, 1.0f);
    return dm::float3(r.x, r.y, r.z);
}

// FleX: TranslationMatrix(Point3).
inline Matrix44 TranslationMatrix(const dm::float3& t) {
    Matrix44 m = Matrix44::Identity();
    m.SetTranslation(t);
    return m;
}

// FleX: ScaleMatrix(Vec3).
inline Matrix44 ScaleMatrix(const dm::float3& s) {
    Matrix44 m = Matrix44::Identity();
    m.columns[0][0] = s.x;
    m.columns[1][1] = s.y;
    m.columns[2][2] = s.z;
    return m;
}

// FleX: RotationMatrix(angle, axis), from PBRT p74.
inline Matrix44 RotationMatrix(float angle, const dm::float3& axis) {
    const dm::float3 a = dm::normalize(axis);
    const float s = sinf(angle);
    const float c = cosf(angle);

    float m[4][4];

    m[0][0] = a.x * a.x + (1.0f - a.x * a.x) * c;
    m[0][1] = a.x * a.y * (1.0f - c) + a.z * s;
    m[0][2] = a.x * a.z * (1.0f - c) - a.y * s;
    m[0][3] = 0.0f;

    m[1][0] = a.x * a.y * (1.0f - c) - a.z * s;
    m[1][1] = a.y * a.y + (1.0f - a.y * a.y) * c;
    m[1][2] = a.y * a.z * (1.0f - c) + a.x * s;
    m[1][3] = 0.0f;

    m[2][0] = a.x * a.z * (1.0f - c) + a.y * s;
    m[2][1] = a.y * a.z * (1.0f - c) - a.x * s;
    m[2][2] = a.z * a.z + (1.0f - a.z * a.z) * c;
    m[2][3] = 0.0f;

    m[3][0] = 0.0f;
    m[3][1] = 0.0f;
    m[3][2] = 0.0f;
    m[3][3] = 1.0f;

    return Matrix44::FromColumns(m);
}

// FleX: RotationMatrix(Quat), the quaternion as x y z w.
Matrix44 RotationMatrix(const dm::float4& q);

// The inverse of a rigid transform (FleX: AffineInverse).
Matrix44 AffineInverse(const Matrix44& m);

// The inverse of any invertible matrix (the demo's renderer used XMMatrixInverse).
Matrix44 Inverse(const Matrix44& m);

// FleX: LookAtMatrix(viewer, target): the view matrix of a camera at viewer looking at target, +y up (OpenGL
// convention, looking down -z).
Matrix44 LookAtMatrix(const dm::float3& viewer, const dm::float3& target);

// FleX: ProjectionMatrix, a drop-in replacement for gluPerspective (fov in degrees, vertical).
inline Matrix44 ProjectionMatrix(float fov, float aspect, float znear, float zfar) {
    const float f = 1.0f / tanf(DegToRad(fov * 0.5f));
    const float zd = znear - zfar;

    const float view[4][4] = {{f / aspect, 0.0f, 0.0f, 0.0f},
                              {0.0f, f, 0.0f, 0.0f},
                              {0.0f, 0.0f, (zfar + znear) / zd, -1.0f},
                              {0.0f, 0.0f, (2.0f * znear * zfar) / zd, 0.0f}};

    return Matrix44::FromColumns(view);
}

// The demo's ConvertToD3DProjection: clip z from [-w, w] to [0, w].
inline Matrix44 ConvertToD3DProjection(const Matrix44& proj) {
    Matrix44 scale = Matrix44::Identity();
    scale.columns[2][2] = 0.5f;
    Matrix44 bias = Matrix44::Identity();
    bias.columns[3][2] = 1.0f;
    return scale * bias * proj;
}

}  // namespace ephys::framework

#endif /* ETHEREAL_PHYS_FRAMEWORK_FLEX_MATH_H */
