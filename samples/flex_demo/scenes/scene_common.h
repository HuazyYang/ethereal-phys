#ifndef FLEX_DEMO_SCENES_SCENE_COMMON_H
#define FLEX_DEMO_SCENES_SCENE_COMMON_H

// What the ported scenes of the FleX demo see: the framework's Scene and DemoState, the helpers library and its
// random generator, and FleX's short vector names. A scene's g_x is g.x (the DemoState its hooks get), and a
// helper's g_buffers is its first argument.

#include <ethereal/phys/device.h>
#include <ethereal/phys/ext/asset_builder.h>
#include <ethereal/phys/ext/moving_frame.h>
#include <ethereal/phys_framework/demo_context.h>
#include <ethereal/phys_framework/flex_math.h>
#include <ethereal/phys_framework/scene.h>
#include <ethereal/phys_helpers/helpers.h>
#include <ethereal/phys_helpers/random.h>
#include <ethereal/phys_utils/mesh.h>
#include <ethereal/phys_utils/perlin.h>

#include <imgui.h>

#include <cfloat>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

using ephys::framework::ClothRenderBuffers;
using ephys::framework::DemoState;
using ephys::framework::Scene;
using namespace ephys;
using namespace ephys::helpers;
using ephys::utils::Perlin1D;

using ephys::framework::DegToRad;

using float3 = dm::float3;
using float4 = dm::float4;

constexpr float k2Pi = 2.0f * kPi;

// FleX's Quat(): the identity rotation (x y z w).
inline float4 QuatIdentity() { return float4(0.0f, 0.0f, 0.0f, 1.0f); }

// C++ leaves the order in which the arguments of a call are evaluated unspecified. Where a FleX call draws more than
// one random number in its arguments, the ported scene draws them explicitly, in the order MSVC (FleX's compiler)
// does, found with a replica of the calls built with /O2 /fp:fast: AddRandomConvex(n, Vec3(x, y, Randf()), ...,
// Randf()) draws the position's z and then the angle; CreateParticleShape(f, a + Vec3(Randf(), 0, Randf()),
// Vec3(.., Randf(), ..), ...) draws the scale's, then the z of the jitter Vec3, then its x.

#endif /* FLEX_DEMO_SCENES_SCENE_COMMON_H */
