#include <cstdint>
#include "FlexMath.h"
#include <cstring>
#include <DirectXMath.h>

namespace NvFlex {

using namespace DirectX;

inline XMVECTOR xm_cast(const NvFlexFloat3& v) {
    return XMLoadFloat3((const XMFLOAT3*)&v);
}

inline XMVECTOR xm_cast(const NvFlexFloat4& v) {
    return XMLoadFloat4((const XMFLOAT4*)&v);
}

inline XMMATRIX xm_cast(const NvFlexFloat4x4& m) {
    return XMLoadFloat4x4((const XMFLOAT4X4*)&m);
}

template <typename Ty>
Ty xm_rcast(XMVECTOR v);

template <>
inline NvFlexFloat3 xm_rcast<NvFlexFloat3>(XMVECTOR v) {
    NvFlexFloat3 r;
    XMStoreFloat3((XMFLOAT3*)&r, v);
    return r;
}

template <>
inline NvFlexFloat4 xm_rcast<NvFlexFloat4>(XMVECTOR v) {
    NvFlexFloat4 r;
    XMStoreFloat4((XMFLOAT4*)&r, v);
    return r;
}

inline NvFlexFloat4x4 xm_rcast(XMMATRIX M) {
    NvFlexFloat4x4 r;
    XMStoreFloat4x4((XMFLOAT4X4*)&r, M);
    return r;
}

NvFlexFloat3 normalize(const NvFlexFloat3& v) {
    XMVECTOR V = xm_cast(v);
    V = XMVector3Normalize(V);
    return xm_rcast<NvFlexFloat3>(V);
}

NvFlexFloat4 pow(const NvFlexFloat4& a, const NvFlexFloat4& b) noexcept {
    XMVECTOR A = xm_cast(a);
    XMVECTOR B = xm_cast(b);
    XMVECTOR C = XMVectorPow(A, B);
    return xm_rcast<NvFlexFloat4>(C);
}

NvFlexFloat4 pow(const NvFlexFloat4& a, float b) noexcept {
    XMVECTOR A = xm_cast(a);
    XMVECTOR B = XMVectorReplicate(b);
    XMVECTOR C = XMVectorPow(A, B);
    return xm_rcast<NvFlexFloat4>(C);
}

float dot(const NvFlexFloat3& a, const NvFlexFloat3& b) noexcept {
    XMVECTOR A = xm_cast(a);
    XMVECTOR B = xm_cast(b);
    XMVECTOR R = XMVector3Dot(A, B);
    return XMVectorGetX(R);
}

NvFlexFloat4x4 transpose(const NvFlexFloat4x4& m) noexcept {
    XMMATRIX M = XMLoadFloat4x4((const XMFLOAT4X4*)&m);
    M = XMMatrixTranspose(M);
    NvFlexFloat4x4 result;
    XMStoreFloat4x4((XMFLOAT4X4*)&result, M);
    return result;
}

float length(const NvFlexFloat3& v) noexcept {
    XMVECTOR V = XMLoadFloat3((const XMFLOAT3*)&v);
    return XMVectorGetX(XMVector3Length(V));
}

float vector3Length(const NvFlexFloat4& v) noexcept {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

NvFlexFloat4 transform4(const NvFlexFloat4& v, const NvFlexFloat4x4& m) noexcept {
    NvFlexFloat4 result;
    XMVECTOR V = XMLoadFloat4((const XMFLOAT4*)&v);
    XMMATRIX M = XMLoadFloat4x4((const XMFLOAT4X4*)&m);
    XMVECTOR Result = XMVector4Transform(V, M);
    XMStoreFloat4((XMFLOAT4*)&result, Result);
    return result;
}

NvFlexFloat4x4 matrixNormalize(const NvFlexFloat4x4& m) noexcept {
    NvFlexFloat4x4 result;
    XMVECTOR V0, V[4];
    V0 = XMLoadFloat4((const XMFLOAT4*)&m.x);
    XMVectorSetW(V0, 0.f);
    V[0] = XMVector4Normalize(V0);
    V0 = XMLoadFloat4((const XMFLOAT4*)&m.y);
    XMVectorSetW(V0, 0.f);
    V[1] = XMVector4Normalize(V0);
    V0 = XMLoadFloat4((const XMFLOAT4*)&m.z);
    XMVectorSetW(V0, 0.f);
    V[2] = XMVector4Normalize(V0);

    XMStoreFloat4((XMFLOAT4*)&result.x, V[0]);
    XMStoreFloat4((XMFLOAT4*)&result.y, V[1]);
    XMStoreFloat4((XMFLOAT4*)&result.z, V[2]);
    result.w = NvFlexFloat4{0.f, 0.f, 0.f, 1.f};

    return result;
}

NvFlexFloat4x4 inverse(const NvFlexFloat4x4& m) noexcept {
    NvFlexFloat4x4 r;
    XMMATRIX M = XMLoadFloat4x4((const XMFLOAT4X4*)&m);
    M = XMMatrixInverse(nullptr, M);
    XMStoreFloat4x4((XMFLOAT4X4*)&r, M);
    return r;
}

NvFlexFloat4x4 identity() noexcept {
    NvFlexFloat4x4 result;
    XMMATRIX I = XMMatrixIdentity();
    XMStoreFloat4x4((XMFLOAT4X4*)&result, I);
    return result;
}

NvFlexFloat4x4 operator*(const NvFlexFloat4x4& a, const NvFlexFloat4x4& b) noexcept {
    NvFlexFloat4x4 r;
    XMMATRIX A = XMLoadFloat4x4((const XMFLOAT4X4*)&a);
    XMMATRIX B = XMLoadFloat4x4((const XMFLOAT4X4*)&b);
    XMMATRIX R = XMMatrixMultiply(A, B);
    XMStoreFloat4x4((XMFLOAT4X4*)&r, R);
    return r;
}

NvFlexFloat4x4 matrixTranslation(float x, float y, float z) {
    XMMATRIX T = XMMatrixTranslation(x, y, z);
    return xm_rcast(T);
}

NvFlexFloat4x4 matrixScaling(float x, float y, float z) {
    XMMATRIX S = XMMatrixScaling(x, y, z);
    return xm_rcast(S);
}

NvFlexFloat3 boundsDiagonal(const NvFlexBounds3f& bounds) {
    return bounds.upper - bounds.lower;
}

}  // namespace NvFlex