#ifndef FLEX_DEMO_SCENES_SCENE_COMMON_H
#define FLEX_DEMO_SCENES_SCENE_COMMON_H

// What the ported scenes of the FleX demo see: the framework's Scene and DemoState, the helpers library and its
// random generator, and FleX's short vector names. A scene's g_x is g.x (the DemoState its hooks get), and a
// helper's g_buffers is its first argument.

#include <ethereal/phys_framework/demo_context.h>
#include <ethereal/phys_framework/scene.h>
#include <ethereal/phys_helpers/helpers.h>
#include <ethereal/phys_helpers/random.h>
#include <ethereal/phys_utils/mesh.h>
#include <ethereal/phys_utils/perlin.h>

#include <cfloat>
#include <cmath>
#include <utility>

using ephys::framework::DemoState;
using ephys::framework::Scene;
using namespace ephys;
using namespace ephys::helpers;
using ephys::utils::Perlin1D;

using float3 = dm::float3;
using float4 = dm::float4;

#endif /* FLEX_DEMO_SCENES_SCENE_COMMON_H */
