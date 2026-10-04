#ifndef ETHEREAL_PHYS_UTILS_EXTRUDE_H
#define ETHEREAL_PHYS_UTILS_EXTRUDE_H

// Tubes along a curve, for rendering ropes and chains: FleX's core/extrude.h, with the Hermite spline helpers
// of core/maths.h it uses. Part of ethereal_phys_utils (samples/common), not of the engine.

#include <donut/core/math/math.h>

#include <vector>

namespace ephys::utils {

// A cubic Hermite spline between a and b with the tangents t1 and t2, at t in [0, 1]: its value, its first
// and its second derivative (FleX's HermiteInterpolate, HermiteTangent, HermiteSecondDerivative).
dm::float3 HermiteInterpolate(const dm::float3& a, const dm::float3& b, const dm::float3& t1, const dm::float3& t2,
                              float t);
dm::float3 HermiteTangent(const dm::float3& a, const dm::float3& b, const dm::float3& t1, const dm::float3& t2,
                          float t);
dm::float3 HermiteSecondDerivative(const dm::float3& a, const dm::float3& b, const dm::float3& t1,
                                   const dm::float3& t2, float t);

// Two unit vectors u, v that make an orthonormal basis with the unit vector w (FleX's BasisFromVector).
void BasisFromVector(const dm::float3& w, dm::float3& u, dm::float3& v);

// Extrudes a circle of `radius` along the Catmull-Rom (Hermite) curve through points: `resolution` vertices
// per ring, `smoothing` rings per segment of the curve (one more on the last). Appends the vertices, their
// normals and the triangles (indices into the vertex array, counting what it held before) to the arrays. The
// rings are carried along the curve by the minimal rotation from one tangent to the next. Nothing is appended
// for fewer than two points.
void Extrude(const dm::float3* points, int numPoints, std::vector<dm::float3>& positions,
             std::vector<dm::float3>& normals, std::vector<int>& indices, float radius, int resolution, int smoothing);

}  // namespace ephys::utils

#endif /* ETHEREAL_PHYS_UTILS_EXTRUDE_H */
