#ifndef ETHEREAL_PHYS_HELPERS_HELPERS_H
#define ETHEREAL_PHYS_HELPERS_HELPERS_H

// The scene-authoring functions of the FleX demo (reference/flex/demo/helpers.h), as a library.
//
// Part of ethereal_phys_helpers (samples/helpers). FleX's helpers wrote into the global g_buffers (and g_mesh,
// g_ropes, the skinning arrays); here every function takes the SimBuffers it writes into (sim_buffers.h) as its
// first argument, and otherwise keeps FleX's name, arguments, defaults and algorithm. The randomness is the
// demo's generator (random.h), drawn in FleX's order.
//
// What changed beyond that, and why:
// - Collision shapes are descriptions (ShapeDesc): AddBox, AddSphere, AddCapsule and AddRandomConvex append one;
//   the framework creates the scene-graph leaf and, for a convex, the IConvexAsset (FleX called
//   NvFlexCreateConvexMesh and CreateGpuMesh here).
// - Meshes are ethereal_phys_utils' (Mesh over donut::math); the transforms are Donut's affine arithmetic, so
//   a sampled position may differ from FleX's in the last bits (the voxelisation and the distance field are
//   FleX's to the bit).
// - Paths: a function that loads a file takes a std::filesystem::path; FleX's "../../data/x" is the sample's
//   asset directory (the framework's DemoState::AssetPath).
//
// The functions of helpers.h not here yet (CreateRope, CreateTetMesh, the triangle mesh and distance field
// shapes, CreateRandomBody, the cluster, link and skinning builders of the soft bodies and SampleMesh) come with
// the scenes that call them (step 10b of the migration); the drawing helpers of the demo's UI (DrawImguiString
// and its siblings) are the framework's ImGui panel.

#include <ethereal/phys_helpers/random.h>
#include <ethereal/phys_helpers/sim_buffers.h>

#include <filesystem>

namespace ephys::helpers {

// ---- Quaternions (FleX's Quat as x y z w) -----------------------------------------------------------------

dm::float4 QuatFromAxisAngle(const dm::float3& axis, float angle);
// FleX's Rotate(q, x): x rotated by the unit quaternion q.
dm::float3 QuatRotate(const dm::float4& q, const dm::float3& x);
// FleX's Matrix33(q): the columns are the images of the axes.
void QuatToMatrix33(const dm::float4& q, dm::float3 cols[3]);

// ---- Particles ---------------------------------------------------------------------------------------------

// The bounds of all particles of buffers (FleX: GetParticleBounds over g_buffers->positions).
void GetParticleBounds(const SimBuffers& buffers, dm::float3& lower, dm::float3& upper);

// A dimx x dimy x dimz block of particles `radius` apart from `lower`, each jittered by RandomUnitVector() *
// jitter. rigid: the block is one shape-matching constraint of stiffness rigidStiffness.
void CreateParticleGrid(SimBuffers& buffers, const dm::float3& lower, int dimx, int dimy, int dimz, float radius,
                        const dm::float3& velocity, float invMass, bool rigid, float rigidStiffness, int phase,
                        float jitter = 0.005f);

// The particles of a dim^3 grid inside a ball around center.
void CreateParticleSphere(SimBuffers& buffers, const dm::float3& center, int dim, float radius,
                          const dm::float3& velocity, float invMass, bool rigid, float rigidStiffness, int phase,
                          float jitter = 0.005f);

// A spring between particles i and j, its rest length (1 + give) times their distance now.
void CreateSpring(SimBuffers& buffers, int i, int j, float stiffness, float give = 0.0f);

// Fills a mesh with particles: rotated about y by `rotation`, scaled so that its largest edge is `scale` (per
// axis), its lower corner at `lower`, voxelised at `spacing`. rigid: one shape-matching constraint, with the
// distance-field normals of the particles as rest normals. skin: the mesh (expanded by skinExpand, offset by
// skinOffset) is appended to buffers.mesh, coloured by phase (or `color`), and skinned to its 4 nearest
// particles of this shape. springStiffness > 0: springs between neighbouring voxels.
void CreateParticleShape(SimBuffers& buffers, const Mesh* srcMesh, const dm::float3& lower, const dm::float3& scale,
                         float rotation, float spacing, const dm::float3& velocity, float invMass, bool rigid,
                         float rigidStiffness, int phase, bool skin, float jitter = 0.005f,
                         const dm::float3& skinOffset = dm::float3(0.0f), float skinExpand = 0.0f,
                         const dm::float4& color = dm::float4(0.0f), float springStiffness = 0.0f);

// The same, with the mesh read from a file (nothing happens when it cannot be read, as in FleX).
void CreateParticleShape(SimBuffers& buffers, const std::filesystem::path& filename, const dm::float3& lower,
                         const dm::float3& scale, float rotation, float spacing, const dm::float3& velocity,
                         float invMass, bool rigid, float rigidStiffness, int phase, bool skin, float jitter = 0.005f,
                         const dm::float3& skinOffset = dm::float3(0.0f), float skinExpand = 0.0f,
                         const dm::float4& color = dm::float4(0.0f), float springStiffness = 0.0f);

// Moves the vertices of buffers.mesh with the rigids they are skinned to: needs the particle positions, the rest
// positions and the rigid rotations of this frame in buffers, and buffers.meshRestPositions.
void SkinMesh(SimBuffers& buffers);

// A cloth: a dx x dy grid in the x-z plane at lower (dz layers up), with stretch, bend and shear springs and
// two triangles per quad (normals up).
void CreateSpringGrid(SimBuffers& buffers, const dm::float3& lower, int dx, int dy, int dz, float radius, int phase,
                      float stretchStiffness, float bendStiffness, float shearStiffness, const dm::float3& velocity,
                      float invMass);

// The non-fluid particle nearest the ray origin + t * dir within `radius` of the ray, or -1; outT its t.
int PickParticle(const dm::float3& origin, const dm::float3& dir, const dm::float4* particles, const int* phases,
                 int n, float radius, float& outT);

// The centres of mass of the rigids (offsets / indices as SimBuffers::rigidOffsets / rigidIndices), averaged
// relative to the mean of all positions, as FleX does for accuracy.
void CalculateRigidCentersOfMass(const dm::float4* restPositions, int numRestPositions, const int* offsets,
                                 dm::float3* translations, const int* indices, int numRigids);

// The rest positions of the rigids' particles relative to their centres of mass.
void CalculateRigidLocalPositions(const dm::float4* restPositions, const int* offsets,
                                  const dm::float3* translations, const int* indices, int numRigids,
                                  dm::float3* localPositions);

// ---- Collision shapes --------------------------------------------------------------------------------------

// A static (or dynamic) box; channels: PhaseShapeChannel* bits.
void AddBox(SimBuffers& buffers, const dm::float3& halfEdge = dm::float3(2.0f),
            const dm::float3& center = dm::float3(0.0f), const dm::float4& quat = dm::float4(0.f, 0.f, 0.f, 1.f),
            bool dynamic = false, int channels = PhaseShapeChannelMask);

// A 4 x 1 x 4 box under the particles: centred on their bounds in x and z, its top at y = 1.
void AddPlinth(SimBuffers& buffers);

void AddSphere(SimBuffers& buffers, float radius, const dm::float3& position, const dm::float4& rotation);

// A capsule along the local x axis.
void AddCapsule(SimBuffers& buffers, float radius, float halfHeight, const dm::float3& position,
                const dm::float4& rotation);

// A convex of numPlanes (6 to 12) planes of a 12-DOP, each at a random distance in [minDist, maxDist] from its
// centre, at position, rotated by angle about axis.
void AddRandomConvex(SimBuffers& buffers, int numPlanes, const dm::float3& position, float minDist, float maxDist,
                     const dm::float3& axis, float angle);

// Removes every shape (FleX: ClearShapes).
void ClearShapes(SimBuffers& buffers);

// Marks the shapes changed, so that the framework sends them at the next frame (FleX: UpdateShapes).
void UpdateShapes(SimBuffers& buffers);

// The union of the world bounds of the shapes; the empty box (lower FLT_MAX, upper -FLT_MAX) without shapes.
void GetShapeBounds(const SimBuffers& buffers, dm::float3& totalLower, dm::float3& totalUpper);

}  // namespace ephys::helpers

#endif /* ETHEREAL_PHYS_HELPERS_HELPERS_H */
