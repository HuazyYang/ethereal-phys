#include <ethereal/phys_framework/flex_math.h>

#include <ethereal/phys_helpers/helpers.h>

// FleX's core/mat44.h and maths.h (see flex_math.h).

namespace ephys::framework {

Matrix44 RotationMatrix(const dm::float4& q) {
    dm::float3 cols[3];
    helpers::QuatToMatrix33(q, cols);

    Matrix44 m = Matrix44::Identity();
    m.SetAxis(0, cols[0]);
    m.SetAxis(1, cols[1]);
    m.SetAxis(2, cols[2]);
    m.SetTranslation(dm::float3(0.0f));
    return m;
}

Matrix44 AffineInverse(const Matrix44& m) {
    Matrix44 inv{};

    // the transpose of the upper 3x3
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r)
            inv.columns[c][r] = m.columns[r][c];

    // -translation times the transpose
    auto dot3 = [](const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
    inv.columns[3][0] = -dot3(m.columns[3], m.columns[0]);
    inv.columns[3][1] = -dot3(m.columns[3], m.columns[1]);
    inv.columns[3][2] = -dot3(m.columns[3], m.columns[2]);
    inv.columns[3][3] = 1.0f;

    return inv;
}

Matrix44 Inverse(const Matrix44& m) {
    // cofactor expansion over the column-major array (the layout does not matter for the inverse: the inverse of
    // the transpose is the transpose of the inverse)
    const float* a = &m.columns[0][0];
    float inv[16];

    inv[0] = a[5] * a[10] * a[15] - a[5] * a[11] * a[14] - a[9] * a[6] * a[15] + a[9] * a[7] * a[14] +
             a[13] * a[6] * a[11] - a[13] * a[7] * a[10];
    inv[4] = -a[4] * a[10] * a[15] + a[4] * a[11] * a[14] + a[8] * a[6] * a[15] - a[8] * a[7] * a[14] -
             a[12] * a[6] * a[11] + a[12] * a[7] * a[10];
    inv[8] = a[4] * a[9] * a[15] - a[4] * a[11] * a[13] - a[8] * a[5] * a[15] + a[8] * a[7] * a[13] +
             a[12] * a[5] * a[11] - a[12] * a[7] * a[9];
    inv[12] = -a[4] * a[9] * a[14] + a[4] * a[10] * a[13] + a[8] * a[5] * a[14] - a[8] * a[6] * a[13] -
              a[12] * a[5] * a[10] + a[12] * a[6] * a[9];
    inv[1] = -a[1] * a[10] * a[15] + a[1] * a[11] * a[14] + a[9] * a[2] * a[15] - a[9] * a[3] * a[14] -
             a[13] * a[2] * a[11] + a[13] * a[3] * a[10];
    inv[5] = a[0] * a[10] * a[15] - a[0] * a[11] * a[14] - a[8] * a[2] * a[15] + a[8] * a[3] * a[14] +
             a[12] * a[2] * a[11] - a[12] * a[3] * a[10];
    inv[9] = -a[0] * a[9] * a[15] + a[0] * a[11] * a[13] + a[8] * a[1] * a[15] - a[8] * a[3] * a[13] -
             a[12] * a[1] * a[11] + a[12] * a[3] * a[9];
    inv[13] = a[0] * a[9] * a[14] - a[0] * a[10] * a[13] - a[8] * a[1] * a[14] + a[8] * a[2] * a[13] +
              a[12] * a[1] * a[10] - a[12] * a[2] * a[9];
    inv[2] = a[1] * a[6] * a[15] - a[1] * a[7] * a[14] - a[5] * a[2] * a[15] + a[5] * a[3] * a[14] +
             a[13] * a[2] * a[7] - a[13] * a[3] * a[6];
    inv[6] = -a[0] * a[6] * a[15] + a[0] * a[7] * a[14] + a[4] * a[2] * a[15] - a[4] * a[3] * a[14] -
             a[12] * a[2] * a[7] + a[12] * a[3] * a[6];
    inv[10] = a[0] * a[5] * a[15] - a[0] * a[7] * a[13] - a[4] * a[1] * a[15] + a[4] * a[3] * a[13] +
              a[12] * a[1] * a[7] - a[12] * a[3] * a[5];
    inv[14] = -a[0] * a[5] * a[14] + a[0] * a[6] * a[13] + a[4] * a[1] * a[14] - a[4] * a[2] * a[13] -
              a[12] * a[1] * a[6] + a[12] * a[2] * a[5];
    inv[3] = -a[1] * a[6] * a[11] + a[1] * a[7] * a[10] + a[5] * a[2] * a[11] - a[5] * a[3] * a[10] -
             a[9] * a[2] * a[7] + a[9] * a[3] * a[6];
    inv[7] = a[0] * a[6] * a[11] - a[0] * a[7] * a[10] - a[4] * a[2] * a[11] + a[4] * a[3] * a[10] +
             a[8] * a[2] * a[7] - a[8] * a[3] * a[6];
    inv[11] = -a[0] * a[5] * a[11] + a[0] * a[7] * a[9] + a[4] * a[1] * a[11] - a[4] * a[3] * a[9] -
              a[8] * a[1] * a[7] + a[8] * a[3] * a[5];
    inv[15] = a[0] * a[5] * a[10] - a[0] * a[6] * a[9] - a[4] * a[1] * a[10] + a[4] * a[2] * a[9] +
              a[8] * a[1] * a[6] - a[8] * a[2] * a[5];

    float det = a[0] * inv[0] + a[1] * inv[4] + a[2] * inv[8] + a[3] * inv[12];
    det = det != 0.0f ? 1.0f / det : 0.0f;

    Matrix44 r;
    float* out = &r.columns[0][0];
    for (int i = 0; i < 16; ++i)
        out[i] = inv[i] * det;
    return r;
}

Matrix44 LookAtMatrix(const dm::float3& viewer, const dm::float3& target) {
    // a basis from the viewer to the target (OpenGL: looking down -z)
    const dm::float3 forward = -dm::normalize(target - viewer);
    dm::float3 up(0.0f, 1.0f, 0.0f);
    const dm::float3 left = dm::normalize(dm::cross(up, forward));
    up = dm::cross(forward, left);

    const float xform[4][4] = {{left.x, left.y, left.z, 0.0f},
                               {up.x, up.y, up.z, 0.0f},
                               {forward.x, forward.y, forward.z, 0.0f},
                               {viewer.x, viewer.y, viewer.z, 1.0f}};

    return AffineInverse(Matrix44::FromColumns(xform));
}

}  // namespace ephys::framework
