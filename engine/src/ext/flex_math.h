#ifndef ETHEREAL_PHYS_SRC_EXT_FLEX_MATH_H
#define ETHEREAL_PHYS_SRC_EXT_FLEX_MATH_H

// Module-private: the quaternions and matrices of FleX's core/ (reference/flex/core/quat.h, mat33.h, mat44.h,
// maths.h) that the container, the soft joints and the moving frame of the extension layer use
// (flexExtContainer.cpp, flexExtMovingFrame.cpp), with FleX's names and in FleX's order of evaluation: the
// parity test compares instance spawns, joint transforms and the moving frame with FleX's extension library
// bit for bit. flex_core.h has the vectors.

#include "ext/flex_core.h"

#include <cmath>
#include <cstring>

namespace ephys::detail::flex {

// XQuat<float>: x y z imaginary, w real. The default is the identity.
struct Quat {
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;

    Quat() = default;
    explicit Quat(const float* p) : x(p[0]), y(p[1]), z(p[2]), w(p[3]) {}
    Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    Quat operator*(float scale) const { Quat r(*this); r *= scale; return r; }
    Quat operator-(const Quat& v) const { Quat r(*this); r -= v; return r; }
    // FleX's quaternion product.
    Quat operator*(const Quat& q) const {
        return Quat(w * q.x + q.w * x + y * q.z - q.y * z, w * q.y + q.w * y + z * q.x - q.z * x,
                    w * q.z + q.w * z + x * q.y - q.x * y, w * q.w - x * q.x - y * q.y - z * q.z);
    }

    Quat& operator*=(float scale) { x *= scale; y *= scale; z *= scale; w *= scale; return *this; }
    Quat& operator-=(const Quat& v) { x -= v.x; y -= v.y; z -= v.z; w -= v.w; return *this; }
};

// Vec3(const Quat&) of FleX: the imaginary part, through the float* conversion.
inline Vec3 XYZ(const Quat& q) { return Vec3(q.x, q.y, q.z); }

inline Quat Inverse(const Quat& q) { return Quat(-q.x, -q.y, -q.z, q.w); }

// quat.h: rotate a vector by a quaternion.
inline Vec3 Rotate(const Quat& q, const Vec3& x) {
    return x * (2.0f * q.w * q.w - 1.0f) + Cross(XYZ(q), x) * q.w * 2.0f + XYZ(q) * Dot(XYZ(q), x) * 2.0f;
}

// mat33.h: three columns.
struct Matrix33 {
    Vec3 cols[3];

    Matrix33() = default;
    Matrix33(const Vec3& c1, const Vec3& c2, const Vec3& c3) {
        cols[0] = c1;
        cols[1] = c2;
        cols[2] = c3;
    }
    explicit Matrix33(const Quat& q) {
        cols[0] = Rotate(q, Vec3(1.0f, 0.0f, 0.0f));
        cols[1] = Rotate(q, Vec3(0.0f, 1.0f, 0.0f));
        cols[2] = Rotate(q, Vec3(0.0f, 0.0f, 1.0f));
    }

    float operator()(int i, int j) const { return cols[j][i]; }
};

// mat33.h: XQuat(const Matrix33&).
inline Quat QuatFromMatrix(const Matrix33& m) {
    Quat q;
    float tr = m(0, 0) + m(1, 1) + m(2, 2), h;
    if (tr >= 0) {
        h = sqrtf(tr + 1);
        q.w = 0.5f * h;
        h = 0.5f / h;

        q.x = (m(2, 1) - m(1, 2)) * h;
        q.y = (m(0, 2) - m(2, 0)) * h;
        q.z = (m(1, 0) - m(0, 1)) * h;
    } else {
        unsigned int i = 0;
        if (m(1, 1) > m(0, 0)) i = 1;
        if (m(2, 2) > m(int(i), int(i))) i = 2;
        switch (i) {
            case 0:
                h = sqrtf((m(0, 0) - (m(1, 1) + m(2, 2))) + 1);
                q.x = 0.5f * h;
                h = 0.5f / h;

                q.y = (m(0, 1) + m(1, 0)) * h;
                q.z = (m(2, 0) + m(0, 2)) * h;
                q.w = (m(2, 1) - m(1, 2)) * h;
                break;
            case 1:
                h = sqrtf((m(1, 1) - (m(2, 2) + m(0, 0))) + 1);
                q.y = 0.5f * h;
                h = 0.5f / h;

                q.z = (m(1, 2) + m(2, 1)) * h;
                q.x = (m(0, 1) + m(1, 0)) * h;
                q.w = (m(0, 2) - m(2, 0)) * h;
                break;
            case 2:
                h = sqrtf((m(2, 2) - (m(0, 0) + m(1, 1))) + 1);
                q.z = 0.5f * h;
                h = 0.5f / h;

                q.x = (m(2, 0) + m(0, 2)) * h;
                q.y = (m(1, 2) + m(2, 1)) * h;
                q.w = (m(1, 0) - m(0, 1)) * h;
                break;
            default:
                q.x = q.y = q.z = q.w = 0;
                break;
        }
    }
    return q;
}

// mat44.h: XMatrix44<float>, column vectors in column-major order (columns[c][r]).
struct Matrix44 {
    float columns[4][4] = {};

    Matrix44() = default;
    explicit Matrix44(const float* d) { std::memcpy(columns, d, sizeof(columns)); }

    static Matrix44 Identity() {
        Matrix44 m;
        m.columns[0][0] = m.columns[1][1] = m.columns[2][2] = m.columns[3][3] = 1.0f;
        return m;
    }

    const float* data() const { return &columns[0][0]; }
    float* data() { return &columns[0][0]; }

    // MatrixMultiply: this * rhs, in FleX's order of accumulation.
    Matrix44 operator*(const Matrix44& rhs) const {
        Matrix44 r;
        const float* lhs = data();
        const float* b = rhs.data();
        float* result = r.data();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result[j * 4 + i] = b[j * 4 + 0] * lhs[i + 0];
                result[j * 4 + i] += b[j * 4 + 1] * lhs[i + 4];
                result[j * 4 + i] += b[j * 4 + 2] * lhs[i + 8];
                result[j * 4 + i] += b[j * 4 + 3] * lhs[i + 12];
            }
        }
        return r;
    }

    void SetAxis(int index, const Vec3& a) {
        columns[index][0] = a.x;
        columns[index][1] = a.y;
        columns[index][2] = a.z;
        columns[index][3] = 0.0f;
    }

    void SetTranslation(const Vec3& p) {
        columns[3][0] = p.x;
        columns[3][1] = p.y;
        columns[3][2] = p.z;
        columns[3][3] = 1.0f;
    }

    Vec3 GetAxis(int i) const { return Vec3(columns[i][0], columns[i][1], columns[i][2]); }
};

// mat44.h: Multiply(const XMatrix44&, const XVector4&).
inline Vec4 operator*(const Matrix44& m, const Vec4& v) {
    const float* mat = m.data();
    Vec4 r;
    r.x = v.x * mat[0] + v.y * mat[4] + v.z * mat[8] + v.w * mat[12];
    r.y = v.x * mat[1] + v.y * mat[5] + v.z * mat[9] + v.w * mat[13];
    r.z = v.x * mat[2] + v.y * mat[6] + v.z * mat[10] + v.w * mat[14];
    r.w = v.x * mat[3] + v.y * mat[7] + v.z * mat[11] + v.w * mat[15];
    return r;
}

// mat44.h: AffineInverse (the transpose of the upper 3 x 3, the translation through it, negated).
inline Matrix44 AffineInverse(const Matrix44& m) {
    Matrix44 inv;
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r) inv.columns[c][r] = m.columns[r][c];

    auto dot3 = [](const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
    inv.columns[3][0] = -dot3(m.columns[3], m.columns[0]);
    inv.columns[3][1] = -dot3(m.columns[3], m.columns[1]);
    inv.columns[3][2] = -dot3(m.columns[3], m.columns[2]);
    inv.columns[3][3] = 1.0f;
    return inv;
}

// maths.h: RotationMatrix(Quat), through Matrix33(q).
inline Matrix44 RotationMatrix(const Quat& q) {
    const Matrix33 rotation(q);
    Matrix44 m;
    m.SetAxis(0, rotation.cols[0]);
    m.SetAxis(1, rotation.cols[1]);
    m.SetAxis(2, rotation.cols[2]);
    m.SetTranslation(Vec3(0.0f));
    return m;
}

// maths.h: TranslationMatrix(Point3).
inline Matrix44 TranslationMatrix(const Vec3& t) {
    Matrix44 m = Matrix44::Identity();
    m.SetTranslation(t);
    return m;
}

}  // namespace ephys::detail::flex

#endif /* ETHEREAL_PHYS_SRC_EXT_FLEX_MATH_H */
