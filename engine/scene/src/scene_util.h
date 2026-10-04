// Helpers of ethereal_phys_scene: node bounds refresh and transform decomposition.

#ifndef ETHEREAL_PHYS_SCENE_SCENE_UTIL_H
#define ETHEREAL_PHYS_SCENE_SCENE_UTIL_H

#include <donut/core/math/math.h>
#include <donut/engine/SceneGraph.h>

#include <algorithm>
#include <cmath>

namespace ephys::scene::detail {

// Makes the next SceneGraph::Refresh recompute the node's global bounding box (and so its ancestors').
// Donut recomputes a bounding box only for nodes with structure or transform changes; InvalidateContent sets
// neither. Setting the local transform to its own value is one: the transforms come out the same.
inline void refreshNodeBounds(donut::engine::SceneGraphNode* node) {
    const dm::double3 translation = node->GetTranslation();
    const dm::dquat rotation = node->GetRotation();
    const dm::double3 scaling = node->GetScaling();
    node->SetTransform(&translation, &rotation, &scaling);
}

// An affine transform in Donut's row-vector convention (v' = v * linear + translation, so row i of the
// linear part is the image of local axis i) taken apart as scale-then-rotate-then-translate: linear =
// diag(scale) * R. That is how SceneGraphNode composes a local transform (scaling * rotation * translation).
// (donut::math::decomposeAffine takes the scale from the column lengths, which is that composition only for a
// uniform scale; the scale here is per local axis.)
struct Decomposition {
    dm::double3 translation = dm::double3::zero();
    dm::dquat rotation = dm::dquat::identity();
    dm::double3 scale = dm::double3(1.0);
    bool orthogonal = true;  // the scaled axes are perpendicular (no shear)
    bool positive = true;    // every scale is above 0 and finite
    bool mirrored = false;   // the axes are left-handed: a reflection
};

inline Decomposition decompose(const dm::daffine3& m) {
    Decomposition d;
    d.translation = m.m_translation;
    dm::double3 axes[3] = {m.m_linear.row0, m.m_linear.row1, m.m_linear.row2};
    for (int i = 0; i < 3; ++i) {
        d.scale[i] = dm::length(axes[i]);
        if (!(d.scale[i] > 0.0) || !std::isfinite(d.scale[i])) {
            d.positive = false;
            return d;
        }
        axes[i] /= d.scale[i];
    }
    const double c_ShearTolerance = 1e-5;
    if (std::abs(dm::dot(axes[0], axes[1])) > c_ShearTolerance ||
        std::abs(dm::dot(axes[0], axes[2])) > c_ShearTolerance ||
        std::abs(dm::dot(axes[1], axes[2])) > c_ShearTolerance)
        d.orthogonal = false;
    if (dm::dot(dm::cross(axes[0], axes[1]), axes[2]) < 0.0) d.mirrored = true;

    // The quaternion whose dquat::toMatrix() is the rotation (rows = axes): the standard conversion, on the
    // transpose (Donut's toMatrix is the row-vector form of the usual rotation matrix).
    const double r00 = axes[0].x, r01 = axes[1].x, r02 = axes[2].x;  // column-vector matrix R = rows^T
    const double r10 = axes[0].y, r11 = axes[1].y, r12 = axes[2].y;
    const double r20 = axes[0].z, r21 = axes[1].z, r22 = axes[2].z;
    const double trace = r00 + r11 + r22;
    dm::dquat q;
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = 0.25 * s;
        q.x = (r21 - r12) / s;
        q.y = (r02 - r20) / s;
        q.z = (r10 - r01) / s;
    } else if (r00 > r11 && r00 > r22) {
        const double s = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
        q.w = (r21 - r12) / s;
        q.x = 0.25 * s;
        q.y = (r01 + r10) / s;
        q.z = (r02 + r20) / s;
    } else if (r11 > r22) {
        const double s = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
        q.w = (r02 - r20) / s;
        q.x = (r01 + r10) / s;
        q.y = 0.25 * s;
        q.z = (r12 + r21) / s;
    } else {
        const double s = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
        q.w = (r10 - r01) / s;
        q.x = (r02 + r20) / s;
        q.y = (r12 + r21) / s;
        q.z = 0.25 * s;
    }
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n > 0.0) q = dm::dquat(q.w / n, q.x / n, q.y / n, q.z / n);
    d.rotation = q;
    return d;
}

// True when the scales are equal within a relative tolerance.
inline bool nearlyEqual(double a, double b, double relative = 1e-5) {
    return std::abs(a - b) <= relative * std::max(std::abs(a), std::abs(b));
}

inline bool isUniform(const dm::double3& scale) {
    return nearlyEqual(scale.x, scale.y) && nearlyEqual(scale.x, scale.z);
}

}  // namespace ephys::scene::detail

#endif /* ETHEREAL_PHYS_SCENE_SCENE_UTIL_H */
