#ifndef ETHEREAL_PHYS_PARAMS_H
#define ETHEREAL_PHYS_PARAMS_H

// Plain data that crosses the ethereal_phys boundary: simulation parameters, the solver description and the
// buffer copy region. donut/nvrhi/doc/com-interface-abi.md, section 4, applies to every struct here: no base
// class, fixed-width members, enums with a fixed underlying type, explicit padding, no alignas and no
// #pragma pack, and a static_assert on the size and on the offset of every member that follows padding or
// an array. A change to any layout in the public headers bumps c_HeaderVersion.
//
// The structs carry default member initialisers instead of a "set defaults" export (FleX had
// NvFlexSetSolverDescDefaults): a DLL exports no C++ function for that, and an initialiser is evaluated in
// the caller's module over a layout this header fixes.

#include <cfloat>
#include <cstddef>
#include <cstdint>

namespace ephys {

// The version of the public headers (every struct layout and every interface vtable under
// <ethereal/phys/...>). etherealPhysCreateDevice refuses a caller compiled against another version, so
// that a stale layout is an error and not memory corruption (device.h).
//
// 1: params.h, buffer.h, device.h.
// 2: kernels.h; IPhysDevice::getKernels.
static constexpr uint32_t c_HeaderVersion = 2;

// Selects the constraint formulation of a solver. It is a creation-time choice (SolverDesc::mode), not a
// simulation parameter: PBD and XPBD are two solver objects, and nothing branches on a mode at run time.
enum class SolverMode : uint8_t {
    PBD = 0,   // position-based dynamics: a per-iteration stiffness multiplier, so the effective stiffness
               // depends on dt, substeps and iterations
    XPBD = 1,  // extended PBD (Macklin, Mueller, Chentanez 2016): compliant constraints with accumulated
               // Lagrange multipliers, stiffness independent of dt, substeps and iterations
};

// How the relaxation factor is applied inside the solver.
enum class RelaxationMode : uint8_t {
    Global = 0,  // a fixed multiplier on each constraint's position delta
    Local = 1,   // a fixed multiplier on each constraint's delta divided by the particle's constraint count:
                 // slower but more reliable convergence
};

// Which solver features are compiled into the kernels of a solver.
enum class FeatureMode : uint8_t {
    Default = 0,       // all features
    SimpleSolids = 1,  // simple per-particle collision (no per-particle SDF normals, no fluids)
    SimpleFluids = 2,  // single-phase fluid particles only (no solids)
};

// Simulation parameters of a solver: FleX's NvFlexParams, with three changes.
//
// - solverMode left: it is SolverDesc::mode.
// - stiffnessMin, stiffnessMax, springDamping and volumeCompliance left: they are XpbdComplianceParams,
//   which only an XPBD solver accepts.
// - relaxationMode has a fixed underlying type and explicit padding, and planes is the last member
//   (numPlanes moved up with it, in front of relaxationMode).
//
// The defaults are the values a FleX solver holds before the first NvFlexSetParams (the Solver
// constructor in reference/flex/src/Solver.cpp): everything zero except numIterations, maxSpeed,
// maxAcceleration, relaxationMode and relaxationFactor. They describe a solver that does nothing useful
// yet (radius 0, no gravity); a scene sets what it needs. The FleX demo's per-scene reset
// (reference/flex/demo/main.cpp) is a different, application-level set and is not what is used here.
struct PhysParams {
    int32_t numIterations = 1;  // solver iterations per substep

    float gravity[3] = {0.0f, 0.0f, 0.0f};  // constant acceleration applied to all particles
    float radius = 0.0f;                    // the maximum interaction radius for particles
    float solidRestDistance = 0.0f;  // distance non-fluid particles keep from each other, in (0, radius]
    float fluidRestDistance = 0.0f;  // spacing of fluid particles at the rest density, in (0, radius];
                                     // generally 50-70% of radius

    // common
    float dynamicFriction = 0.0f;   // friction against shapes
    float staticFriction = 0.0f;    // static friction against shapes
    float particleFriction = 0.0f;  // friction between particles
    float restitution = 0.0f;       // restitution against shapes; particle collisions are always inelastic
    float adhesion = 0.0f;          // how strongly particles stick to surfaces they hit, [0, +inf)
    float sleepThreshold = 0.0f;    // particles slower than this are considered fixed

    float maxSpeed = FLT_MAX;         // particle velocity magnitude is clamped to this at the end of a step
    float maxAcceleration = FLT_MAX;  // particle acceleration magnitude is clamped to this at the end of a
                                      // step (limits popping after large interpenetrations)

    float shockPropagation = 0.0f;  // artificially decreases particle mass with height above a fixed
                                    // reference point, so that stacks and piles converge faster
    float dissipation = 0.0f;       // damps particle velocity by its number of contacts
    float damping = 0.0f;           // viscous drag, proportional and opposite to the particle velocity

    // cloth
    float wind[3] = {0.0f, 0.0f, 0.0f};  // constant acceleration applied to particles of dynamic triangles;
                                         // needs drag > 0
    float drag = 0.0f;  // drag on particles of dynamic triangles, proportional to velocity^2 * area
    float lift = 0.0f;  // lift on particles of dynamic triangles, proportional to velocity^2 * area

    // fluid
    float cohesion = 0.0f;              // how strongly particles hold each other together, [0, +inf)
    float surfaceTension = 0.0f;        // how strongly particles minimise surface area, [0, +inf)
    float viscosity = 0.0f;             // XSPH viscosity
    float vorticityConfinement = 0.0f;  // rotational forces that increase vorticity
    float anisotropyScale = 0.0f;       // anisotropy of the ellipsoids for rendering; 0: not computed
    float anisotropyMin = 0.0f;         // clamps the anisotropy scale to this fraction of the radius
    float anisotropyMax = 0.0f;         // clamps the anisotropy scale to this fraction of the radius
    float smoothing = 0.0f;             // Laplacian smoothing of the positions for rendering; 0: not computed
    float solidPressure = 0.0f;         // pressure from solid surfaces on particles
    float freeSurfaceDrag = 0.0f;       // drag on boundary fluid particles
    float buoyancy = 0.0f;              // gravity is scaled by this for fluid particles

    // diffuse particles
    float diffuseThreshold = 0.0f;  // kinetic energy + divergence above which diffuse particles spawn
    float diffuseBuoyancy = 0.0f;   // scales the force opposing gravity on diffuse particles
    float diffuseDrag = 0.0f;       // scales the force towards neighbouring fluid particles
    int32_t diffuseBallistic = 0;   // neighbour count below which a diffuse particle is ballistic
    float diffuseLifetime = 0.0f;   // seconds; particles spawn with a random lifetime in [0, diffuseLifetime]

    // collision
    float collisionDistance = 0.0f;        // distance particles keep from shapes; greater than zero for
                                           // robust collision against triangle meshes
    float particleCollisionMargin = 0.0f;  // enlarges the radius used during neighbour finding
    float shapeCollisionMargin = 0.0f;     // enlarges the radius used during contact finding against shapes

    int32_t numPlanes = 0;  // number of entries of planes in use

    RelaxationMode relaxationMode = RelaxationMode::Local;
    uint8_t _pad[3] = {0, 0, 0};    // explicit: keeps relaxationFactor at a 4-byte offset on every compiler
    float relaxationFactor = 1.0f;  // convergence rate of the parallel solver; above 1 may be unstable

    float planes[8][4] = {};  // collision planes, a*x + b*y + c*z + d = 0
};

static_assert(sizeof(PhysParams) == 308, "PhysParams layout changed: bump c_HeaderVersion");
static_assert(alignof(PhysParams) == 4, "PhysParams alignment changed: bump c_HeaderVersion");
static_assert(offsetof(PhysParams, gravity) == 4);
static_assert(offsetof(PhysParams, radius) == 16);  // after gravity[3]
static_assert(offsetof(PhysParams, wind) == 72);
static_assert(offsetof(PhysParams, drag) == 84);  // after wind[3]
static_assert(offsetof(PhysParams, diffuseBallistic) == 148);
static_assert(offsetof(PhysParams, numPlanes) == 168);
static_assert(offsetof(PhysParams, relaxationMode) == 172);
static_assert(offsetof(PhysParams, _pad) == 173);
static_assert(offsetof(PhysParams, relaxationFactor) == 176);  // after the padding
static_assert(offsetof(PhysParams, planes) == 180);

// The XPBD-only parameters. Only an XPBD solver accepts them (through its own interface), so a PBD solver
// cannot be handed compliance values.
//
// A spring or rigid stiffness coefficient k in [0, 1] is mapped geometrically onto
// [stiffnessMin, stiffnessMax], and the compliance is its reciprocal:
// alpha = 1 / (stiffnessMin * (stiffnessMax / stiffnessMin)^k). k <= 0 disables the constraint; a negative
// k still marks a tether. These are spring constants (N/m), not moduli.
struct XpbdComplianceParams {
    float stiffnessMin = 1.0e3f;    // the stiffness k -> 0+ approaches; values <= 0 are clamped to 1e-6
    float stiffnessMax = 1.0e9f;    // the stiffness k = 1 maps to; values below stiffnessMin are clamped to it
    float springDamping = 0.0f;     // Rayleigh constraint damping of springs; acts on compliant springs only
    float volumeCompliance = 0.0f;  // compliance of inflatable volume constraints (inflatables carry no k)
};

static_assert(sizeof(XpbdComplianceParams) == 16, "XpbdComplianceParams layout changed: bump c_HeaderVersion");
static_assert(offsetof(XpbdComplianceParams, volumeCompliance) == 12);

// The creation-time description of a solver. The defaults of the four capacities are those of FleX's
// NvFlexSetSolverDescDefaults.
struct SolverDesc {
    SolverMode mode = SolverMode::PBD;
    FeatureMode featureMode = FeatureMode::Default;
    uint8_t _pad[2] = {0, 0};               // explicit: keeps maxParticles at a 4-byte offset
    uint32_t maxParticles = 0x4000;         // maximum number of regular particles
    uint32_t maxDiffuseParticles = 0;       // maximum number of diffuse particles
    uint32_t maxNeighborsPerParticle = 96;  // around 32 is enough for solids; fluids may need up to 128
    uint32_t maxContactsPerParticle = 6;    // maximum number of shape contacts per particle
};

static_assert(sizeof(SolverDesc) == 20, "SolverDesc layout changed: bump c_HeaderVersion");
static_assert(offsetof(SolverDesc, featureMode) == 1);
static_assert(offsetof(SolverDesc, _pad) == 2);
static_assert(offsetof(SolverDesc, maxParticles) == 4);  // after the padding
static_assert(offsetof(SolverDesc, maxContactsPerParticle) == 16);

// A source and a destination range for a buffer copy, in elements (FleX's NvFlexCopyDesc).
struct CopyRegion {
    uint32_t srcOffset = 0;     // first element read from the source
    uint32_t dstOffset = 0;     // first element written in the destination
    uint32_t elementCount = 0;  // number of elements copied
};

static_assert(sizeof(CopyRegion) == 12, "CopyRegion layout changed: bump c_HeaderVersion");
static_assert(offsetof(CopyRegion, elementCount) == 8);

}  // namespace ephys

#endif /* ETHEREAL_PHYS_PARAMS_H */
