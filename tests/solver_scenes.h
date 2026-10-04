// Test support: the scenes the solver tests run, as plain arrays.
//
// A Scene describes one simulation in host memory, in the terms both the engine and FleX understand: the
// particle arrays, the constraint sets, the collision geometry and the parameters. solver_test.cpp loads it
// into an ephys domain; parity_test.cpp loads the same arrays into a FleX solver as well and compares.
//
// The builders are procedural and deterministic (a fixed linear congruential generator for the jitter):
// two builds of a scene are the same bytes. They are the minimum of reference/flex/demo/helpers.h that the
// scenes need (CreateParticleGrid, CreateSpringGrid, the rigid body set-up of CreateParticleShape and
// CalculateRigidLocalPositions, the inflatable set-up of core/cloth.h), rewritten over std::vector.

#ifndef ETHEREAL_PHYS_TESTS_SOLVER_SCENES_H
#define ETHEREAL_PHYS_TESTS_SOLVER_SCENES_H

#include <ethereal/phys/device.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace scenes {

struct Vec3 {
    float x = 0, y = 0, z = 0;
};
inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalize(Vec3 a) {
    const float l = length(a);
    return l > 0 ? a * (1.0f / l) : a;
}

// A triangle mesh asset: float4 vertices, three indices per triangle, bounds in its local space.
struct Mesh {
    std::vector<float> vertices;
    std::vector<int32_t> indices;
    float lower[3] = {0, 0, 0};
    float upper[3] = {0, 0, 0};
};

// A convex asset: planes a x + b y + c z + d = 0, and its bounds.
struct Convex {
    std::vector<float> planes;
    float lower[3] = {0, 0, 0};
    float upper[3] = {0, 0, 0};
};

// A signed distance field asset.
struct Sdf {
    int dim = 0;
    std::vector<float> field;
};

struct Shape {
    ephys::CollisionShapeType type = ephys::CollisionShapeType::Sphere;
    bool dynamic = false;
    float geometry[4] = {0, 0, 0, 0};  // as CollisionGeometry::raw, without the asset id
    int asset = -1;                    // index into Scene::meshes, convexes or sdfs
    float position[3] = {0, 0, 0};
    float rotation[4] = {0, 0, 0, 1};
    float velocity[3] = {0, 0, 0};  // a moving shape: its position advances by velocity * dt per step
};

struct Scene;

// What changes in the parameters of a scene from step to step (the wind of xpbdCloth): called before step
// `step` (0: the first) with a copy of Scene::params to modify.
typedef void (*SceneUpdate)(const Scene& scene, uint32_t step, ephys::PhysParams& params);

struct Scene {
    std::string name;
    ephys::PhysParams params;
    // The solver the scene is made for. An XPBD scene carries its compliance block as well
    // (IPhysSolverXPBD::setCompliance; FleX's NvFlexParams::stiffnessMin ... volumeCompliance).
    ephys::SolverMode mode = ephys::SolverMode::PBD;
    ephys::XpbdComplianceParams compliance;
    SceneUpdate update = nullptr;
    ephys::FeatureMode featureMode = ephys::FeatureMode::Default;
    uint32_t maxNeighbors = 96;
    uint32_t maxContacts = 6;
    uint32_t maxDiffuse = 0;
    uint32_t spareParticles = 37;  // capacity beyond the particles of the scene
    float dt = 1.0f / 60.0f;
    uint32_t substeps = 2;

    // Particles.
    std::vector<float> positions;      // 4 per particle: x y z 1/m
    std::vector<float> velocities;     // 3
    std::vector<int32_t> phases;       // 1
    std::vector<float> normals;        // 4; empty: none are uploaded
    std::vector<float> restPositions;  // 4; empty: none are uploaded

    // Springs.
    std::vector<int32_t> springIndices;  // 2 per spring
    std::vector<float> springLengths;
    std::vector<float> springStiffness;

    // Shape matching.
    std::vector<int32_t> rigidOffsets;  // numRigids + 1 when there are rigids
    std::vector<int32_t> rigidIndices;
    std::vector<float> rigidRestPositions;  // 3 per index
    std::vector<float> rigidRestNormals;    // 4 per index
    std::vector<float> rigidStiffness;
    std::vector<float> rigidThresholds;  // empty: no plastic deformation
    std::vector<float> rigidCreeps;
    std::vector<float> rigidRotations;     // 4 per rigid
    std::vector<float> rigidTranslations;  // 3 per rigid

    // Dynamic triangles.
    std::vector<int32_t> triangleIndices;  // 3 per triangle
    std::vector<float> triangleNormals;    // 3 per triangle

    // Inflatables.
    std::vector<ephys::InflatableDesc> inflatables;

    // Collision geometry.
    std::vector<Mesh> meshes;
    std::vector<Convex> convexes;
    std::vector<Sdf> sdfs;
    std::vector<Shape> shapes;

    uint32_t numParticles() const { return uint32_t(phases.size()); }
    uint32_t maxParticles() const { return numParticles() + spareParticles; }
    uint32_t numSprings() const { return uint32_t(springLengths.size()); }
    uint32_t numRigids() const { return uint32_t(rigidStiffness.size()); }
    uint32_t numTriangles() const { return uint32_t(triangleIndices.size() / 3); }
    bool hasMovingShapes() const {
        for (const Shape& s : shapes)
            if (s.velocity[0] != 0 || s.velocity[1] != 0 || s.velocity[2] != 0) return true;
        return false;
    }

    // The position of a shape at the end of step `step` (0: the first step) and at its start.
    void shapePosition(const Shape& s, uint32_t step, float current[4], float previous[4]) const {
        for (int k = 0; k < 3; ++k) {
            previous[k] = s.position[k] + s.velocity[k] * dt * float(step);
            current[k] = s.position[k] + s.velocity[k] * dt * float(step + 1);
        }
        previous[3] = current[3] = 0.0f;
    }
};

// ---- Builders ----------------------------------------------------------------------------------------------

class Jitter {
 public:
    explicit Jitter(uint32_t seed) : m_State(seed) {}
    // In [-1, 1).
    float next() {
        m_State = m_State * 1664525u + 1013904223u;
        return float(m_State >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }

 private:
    uint32_t m_State;
};

inline void addParticle(Scene& s, Vec3 p, float invMass, Vec3 v, int32_t phase) {
    s.positions.insert(s.positions.end(), {p.x, p.y, p.z, invMass});
    s.velocities.insert(s.velocities.end(), {v.x, v.y, v.z});
    s.phases.push_back(phase);
}

// helpers.h: CreateParticleGrid.
inline void addParticleGrid(Scene& s, Vec3 lower, int dimx, int dimy, int dimz, float spacing, Vec3 velocity,
                            float invMass, int32_t phase, float jitter, Jitter& random) {
    for (int x = 0; x < dimx; ++x)
        for (int y = 0; y < dimy; ++y)
            for (int z = 0; z < dimz; ++z) {
                const Vec3 offset = {random.next() * jitter, random.next() * jitter, random.next() * jitter};
                addParticle(s, lower + Vec3{float(x), float(y), float(z)} * spacing + offset, invMass, velocity, phase);
            }
}

inline void addSpring(Scene& s, int32_t i, int32_t j, float stiffness) {
    const Vec3 a = {s.positions[size_t(i) * 4], s.positions[size_t(i) * 4 + 1], s.positions[size_t(i) * 4 + 2]};
    const Vec3 b = {s.positions[size_t(j) * 4], s.positions[size_t(j) * 4 + 1], s.positions[size_t(j) * 4 + 2]};
    s.springIndices.push_back(i);
    s.springIndices.push_back(j);
    s.springLengths.push_back(length(a - b));
    s.springStiffness.push_back(stiffness);
}

// The six planes of the box [lower, upper], facing inwards (helpers of demo/main.cpp).
inline void setPlaneBox(ephys::PhysParams& params, Vec3 lower, Vec3 upper, int numPlanes = 6) {
    const float planes[6][4] = {{0, 1, 0, -lower.y}, {0, 0, 1, -lower.z},  {1, 0, 0, -lower.x},
                                {-1, 0, 0, upper.x}, {0, 0, -1, upper.z}, {0, -1, 0, upper.y}};
    for (int i = 0; i < numPlanes; ++i) std::memcpy(params.planes[i], planes[i], sizeof(planes[i]));
    params.numPlanes = numPlanes;
}

// The parameters every scene starts from: those of the FleX demo before a scene changes them.
inline ephys::PhysParams defaultParams() {
    ephys::PhysParams p;
    p.gravity[1] = -9.8f;
    p.radius = 0.15f;
    p.numIterations = 3;
    p.anisotropyMin = 0.1f;
    p.anisotropyMax = 2.0f;
    p.maxSpeed = FLT_MAX;
    p.maxAcceleration = 100.0f;
    p.relaxationMode = ephys::RelaxationMode::Local;
    p.relaxationFactor = 1.0f;
    p.solidPressure = 1.0f;
    p.buoyancy = 1.0f;
    p.diffuseThreshold = 100.0f;
    p.diffuseBuoyancy = 1.0f;
    p.diffuseDrag = 0.8f;
    p.diffuseBallistic = 16;
    p.diffuseLifetime = 2.0f;
    return p;
}

// What the demo derives after a scene set its parameters (demo/main.cpp, after the scene's Initialize).
inline void finishParams(ephys::PhysParams& p) {
    if (p.solidRestDistance == 0.0f) p.solidRestDistance = p.radius;
    if (p.fluidRestDistance > 0.0f) p.solidRestDistance = p.fluidRestDistance;
    if (p.collisionDistance == 0.0f) p.collisionDistance = std::max(p.solidRestDistance, p.fluidRestDistance) * 0.5f;
    if (p.particleFriction == 0.0f) p.particleFriction = p.dynamicFriction * 0.1f;
    if (p.shapeCollisionMargin == 0.0f) p.shapeCollisionMargin = p.collisionDistance * 0.5f;
}

// (i) A block of solid particles dropped into a box of planes.
inline Scene granular() {
    Scene s;
    s.name = "granular";
    s.params = defaultParams();
    s.params.radius = 0.1f;
    s.params.dynamicFriction = 0.5f;
    s.params.staticFriction = 0.6f;
    s.params.particleFriction = 0.1f;
    s.params.restitution = 0.1f;
    s.params.dissipation = 0.01f;
    s.params.numIterations = 3;
    setPlaneBox(s.params, {-1.0f, 0.0f, -1.0f}, {1.0f, 4.0f, 1.0f});
    finishParams(s.params);

    Jitter random(1);
    addParticleGrid(s, {-0.45f, 0.3f, -0.45f}, 10, 10, 10, 0.1f, {0, 0, 0}, 1.0f,
                    ephys::makePhase(0, ephys::PhaseSelfCollide), 0.005f, random);
    return s;
}

// (viii) The granular block with the other relaxation mode, more substeps and iterations, and the features
// the plain scene leaves off: shock propagation, the sleep threshold, damping, adhesion.
inline Scene granularGlobal() {
    Scene s = granular();
    s.name = "granular_global";
    s.params.relaxationMode = ephys::RelaxationMode::Global;
    s.params.relaxationFactor = 0.25f;
    s.params.numIterations = 5;
    s.params.shockPropagation = 0.5f;
    s.params.sleepThreshold = 0.02f;
    s.params.damping = 0.1f;
    s.params.adhesion = 0.05f;
    s.params.particleCollisionMargin = 0.01f;
    s.substeps = 3;
    return s;
}

// (ii) A dam break: a column of fluid in one corner of a box.
inline Scene fluid() {
    Scene s;
    s.name = "fluid";
    s.params = defaultParams();
    s.params.radius = 0.1f;
    s.params.fluidRestDistance = 0.055f;
    s.params.dynamicFriction = 0.1f;
    s.params.cohesion = 0.025f;
    s.params.viscosity = 0.05f;
    s.params.vorticityConfinement = 40.0f;
    s.params.surfaceTension = 0.02f;
    s.params.adhesion = 0.05f;
    s.params.freeSurfaceDrag = 0.01f;
    s.params.anisotropyScale = 1.0f;
    s.params.smoothing = 1.0f;
    s.params.numIterations = 3;
    setPlaneBox(s.params, {0.0f, 0.0f, 0.0f}, {2.0f, 4.0f, 0.8f});
    finishParams(s.params);

    Jitter random(2);
    addParticleGrid(s, {0.05f, 0.05f, 0.05f}, 12, 24, 12, 0.055f, {0, 0, 0}, 1.0f,
                    ephys::makePhase(0, ephys::PhaseSelfCollide | ephys::PhaseFluid), 0.002f, random);
    s.maxNeighbors = 96;
    return s;
}

// (vii) The dam break with diffuse particles.
inline Scene fluidDiffuse() {
    Scene s = fluid();
    s.name = "fluid_diffuse";
    s.params.surfaceTension = 0.0f;  // the plain density kernels
    s.params.diffuseThreshold = 0.5f;
    s.params.diffuseLifetime = 1.5f;
    s.maxDiffuse = 4096;
    return s;
}

// (iii) A square of cloth in the x z plane, pinned along one edge, with wind.
inline Scene cloth() {
    Scene s;
    s.name = "cloth";
    s.params = defaultParams();
    s.params.radius = 0.05f;
    s.params.dynamicFriction = 0.25f;
    s.params.wind[0] = 0.5f;
    s.params.wind[2] = 2.0f;
    s.params.drag = 0.06f;
    s.params.lift = 0.03f;
    s.params.numIterations = 5;
    setPlaneBox(s.params, {-2.0f, 0.0f, -2.0f}, {2.0f, 4.0f, 2.0f}, 1);
    finishParams(s.params);

    // helpers.h: CreateSpringGrid, with dz = 1.
    const int dx = 24, dy = 24;
    const float spacing = 0.05f;
    const Vec3 lower = {-0.6f, 1.5f, -0.6f};
    const int32_t phase = ephys::makePhase(0, ephys::PhaseSelfCollide | ephys::PhaseSelfCollideFilter);
    auto index = [&](int x, int y) { return y * dx + x; };
    for (int y = 0; y < dy; ++y)
        for (int x = 0; x < dx; ++x) {
            addParticle(s, lower + Vec3{float(x), 0.0f, float(y)} * spacing, y == 0 ? 0.0f : 1.0f, {0, 0, 0}, phase);
            if (x > 0 && y > 0) {
                s.triangleIndices.insert(s.triangleIndices.end(), {index(x - 1, y - 1), index(x, y - 1), index(x, y)});
                s.triangleIndices.insert(s.triangleIndices.end(), {index(x - 1, y - 1), index(x, y), index(x - 1, y)});
                s.triangleNormals.insert(s.triangleNormals.end(), {0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f});
            }
        }
    const float stretch = 0.9f, bend = 0.75f, shear = 0.8f;
    for (int y = 0; y < dy; ++y)
        for (int x = 0; x < dx; ++x) {
            if (x > 0) addSpring(s, index(x, y), index(x - 1, y), stretch);
            if (x > 1) addSpring(s, index(x, y), index(x - 2, y), bend);
            if (y > 0 && x < dx - 1) addSpring(s, index(x, y), index(x + 1, y - 1), shear);
            if (y > 0 && x > 0) addSpring(s, index(x, y), index(x - 1, y - 1), shear);
        }
    for (int x = 0; x < dx; ++x)
        for (int y = 0; y < dy; ++y) {
            if (y > 0) addSpring(s, index(x, y), index(x, y - 1), stretch);
            if (y > 1) addSpring(s, index(x, y), index(x, y - 2), bend);
        }
    // Tethers (negative stiffness: they resist stretching only) from the pinned edge to the free corners.
    addSpring(s, index(0, 0), index(0, dy - 1), -0.5f);
    addSpring(s, index(dx - 1, 0), index(dx - 1, dy - 1), -0.5f);

    s.restPositions = s.positions;
    s.normals.assign(s.positions.size(), 0.0f);
    for (size_t i = 0; i < s.normals.size(); i += 4) s.normals[i + 1] = 1.0f;
    return s;
}

// A rigid body: a block of dimx x dimy x dimz particles around `centre`.
inline void addRigidBlock(Scene& s, Vec3 centre, int dimx, int dimy, int dimz, float spacing, float stiffness,
                          int32_t group, Vec3 velocity) {
    if (s.rigidOffsets.empty()) s.rigidOffsets.push_back(0);
    const int32_t phase = ephys::makePhase(group, 0);
    const Vec3 half = Vec3{float(dimx - 1), float(dimy - 1), float(dimz - 1)} * (spacing * 0.5f);
    const Vec3 extent = half + Vec3{spacing * 0.5f, spacing * 0.5f, spacing * 0.5f};  // the surface of the block
    for (int x = 0; x < dimx; ++x)
        for (int y = 0; y < dimy; ++y)
            for (int z = 0; z < dimz; ++z) {
                const Vec3 local = Vec3{float(x), float(y), float(z)} * spacing - half;
                const int32_t index = int32_t(s.numParticles());
                addParticle(s, centre + local, 1.0f, velocity, phase);

                // The distance field of the block at the particle: the normal of the nearest face, and the
                // (negative) depth below it.
                const float depth[3] = {extent.x - std::fabs(local.x), extent.y - std::fabs(local.y),
                                        extent.z - std::fabs(local.z)};
                int axis = 0;
                if (depth[1] < depth[axis]) axis = 1;
                if (depth[2] < depth[axis]) axis = 2;
                float normal[4] = {0, 0, 0, -depth[axis]};
                const float component = axis == 0 ? local.x : axis == 1 ? local.y : local.z;
                normal[axis] = component < 0.0f ? -1.0f : 1.0f;

                s.rigidIndices.push_back(index);
                s.rigidRestPositions.insert(s.rigidRestPositions.end(), {local.x, local.y, local.z});
                s.rigidRestNormals.insert(s.rigidRestNormals.end(), normal, normal + 4);
            }
    s.rigidOffsets.push_back(int32_t(s.rigidIndices.size()));
    s.rigidStiffness.push_back(stiffness);
    s.rigidRotations.insert(s.rigidRotations.end(), {0.0f, 0.0f, 0.0f, 1.0f});
    s.rigidTranslations.insert(s.rigidTranslations.end(), {centre.x, centre.y, centre.z});
}

// (iv) A few shape-matched bodies falling onto the ground and onto each other.
inline Scene rigid() {
    Scene s;
    s.name = "rigid";
    s.params = defaultParams();
    s.params.radius = 0.1f;
    s.params.dynamicFriction = 0.4f;
    s.params.staticFriction = 0.4f;
    s.params.numIterations = 4;
    setPlaneBox(s.params, {-1.5f, 0.0f, -1.5f}, {1.5f, 4.0f, 1.5f});
    finishParams(s.params);

    const float spacing = 0.05f;
    addRigidBlock(s, {0.0f, 0.25f, 0.0f}, 5, 4, 5, spacing, 1.0f, 1, {0, 0, 0});
    addRigidBlock(s, {0.13f, 0.6f, 0.04f}, 4, 4, 4, spacing, 1.0f, 2, {0, 0, 0});
    addRigidBlock(s, {-0.05f, 0.95f, 0.12f}, 6, 3, 3, spacing, 0.8f, 3, {0.2f, 0, 0});
    addRigidBlock(s, {0.5f, 0.4f, -0.3f}, 3, 3, 9, spacing, 1.0f, 4, {-1.0f, 0, 0.3f});

    // The particles carry the normals of their body (NvFlexSetNormals), as CreateParticleShape sets them.
    s.normals.assign(s.positions.size(), 0.0f);
    for (size_t i = 0; i < s.rigidIndices.size(); ++i)
        std::memcpy(&s.normals[size_t(s.rigidIndices[i]) * 4], &s.rigidRestNormals[i * 4], 16);
    return s;
}

// (iv) ... with plastic deformation: the bodies hit the ground fast enough to yield.
inline Scene rigidPlastic() {
    Scene s = rigid();
    s.name = "rigid_plastic";
    for (size_t i = 0; i < s.velocities.size(); i += 3) s.velocities[i + 1] -= 3.0f;
    s.rigidThresholds.assign(s.numRigids(), 0.002f);
    s.rigidCreeps.assign(s.numRigids(), 0.6f);
    s.rigidThresholds[1] = 1000.0f;  // one body that never yields
    return s;
}

// An icosphere: `subdivisions` rounds of splitting every triangle of an icosahedron in four.
inline void icosphere(int subdivisions, float radius, std::vector<Vec3>& vertices, std::vector<int32_t>& indices) {
    const float t = (1.0f + std::sqrt(5.0f)) * 0.5f;
    vertices = {{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t},
                {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
    indices = {0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11, 1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8,
               3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9, 4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9, 8, 1};
    for (int round = 0; round < subdivisions; ++round) {
        std::map<std::pair<int32_t, int32_t>, int32_t> midpoints;
        auto midpoint = [&](int32_t a, int32_t b) {
            const std::pair<int32_t, int32_t> key = std::minmax(a, b);
            auto it = midpoints.find(key);
            if (it != midpoints.end()) return it->second;
            vertices.push_back((vertices[size_t(a)] + vertices[size_t(b)]) * 0.5f);
            const int32_t index = int32_t(vertices.size() - 1);
            midpoints[key] = index;
            return index;
        };
        std::vector<int32_t> next;
        for (size_t i = 0; i < indices.size(); i += 3) {
            const int32_t a = indices[i], b = indices[i + 1], c = indices[i + 2];
            const int32_t ab = midpoint(a, b), bc = midpoint(b, c), ca = midpoint(c, a);
            next.insert(next.end(), {a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca});
        }
        indices = next;
    }
    for (Vec3& v : vertices) v = normalize(v) * radius;
}

// (v) A closed mesh held in shape by a volume constraint: springs along its edges, its triangles as dynamic
// triangles, one inflatable over them (core/cloth.h: the rest volume and the constraint scale).
inline Scene inflatable() {
    Scene s;
    s.name = "inflatable";
    s.params = defaultParams();
    s.params.radius = 0.1f;
    s.params.dynamicFriction = 0.4f;
    s.params.drag = 0.02f;
    s.params.numIterations = 5;
    setPlaneBox(s.params, {-2.0f, 0.0f, -2.0f}, {2.0f, 4.0f, 2.0f});
    finishParams(s.params);

    std::vector<Vec3> vertices;
    std::vector<int32_t> indices;
    icosphere(2, 0.4f, vertices, indices);  // 162 vertices, 320 triangles: the last block of 512 is not full

    const Vec3 centre = {0.0f, 0.75f, 0.0f};
    const int32_t phase = ephys::makePhase(0, ephys::PhaseSelfCollide | ephys::PhaseSelfCollideFilter);
    for (const Vec3& v : vertices) addParticle(s, centre + v, 1.0f, {0.3f, 0, 0}, phase);
    s.restPositions = s.positions;

    std::map<std::pair<int32_t, int32_t>, bool> edges;
    float restVolume = 0.0f;
    std::vector<Vec3> gradients(vertices.size());
    for (size_t i = 0; i < indices.size(); i += 3) {
        const int32_t tri[3] = {indices[i], indices[i + 1], indices[i + 2]};
        s.triangleIndices.insert(s.triangleIndices.end(), tri, tri + 3);
        const Vec3 v1 = vertices[size_t(tri[0])], v2 = vertices[size_t(tri[1])], v3 = vertices[size_t(tri[2])];
        const Vec3 n = cross(v2 - v1, v3 - v1);
        const Vec3 unit = normalize(n);
        s.triangleNormals.insert(s.triangleNormals.end(), {unit.x, unit.y, unit.z});
        restVolume += dot(v1, n);
        for (int k = 0; k < 3; ++k) {
            gradients[size_t(tri[k])] = gradients[size_t(tri[k])] + n;
            const std::pair<int32_t, int32_t> key = std::minmax(tri[k], tri[(k + 1) % 3]);
            if (!edges[key]) {
                edges[key] = true;
                addSpring(s, key.first, key.second, 0.9f);
            }
        }
    }
    float constraintScale = 0.0f;
    for (const Vec3& g : gradients) constraintScale += dot(g, g);
    constraintScale = 1.0f / constraintScale;

    ephys::InflatableDesc inflatable;
    inflatable.startTri = 0;
    inflatable.numTris = int32_t(indices.size() / 3);
    inflatable.restVolume = restVolume;
    inflatable.overPressure = 1.1f;
    inflatable.constraintScale = constraintScale;
    s.inflatables.push_back(inflatable);
    return s;
}

// A quaternion (x y z w) for a rotation of `angle` radians about a unit axis.
inline void setRotation(Shape& shape, Vec3 axis, float angle) {
    const float half = angle * 0.5f;
    const float sine = std::sin(half);
    shape.rotation[0] = axis.x * sine;
    shape.rotation[1] = axis.y * sine;
    shape.rotation[2] = axis.z * sine;
    shape.rotation[3] = std::cos(half);
}

// (vi) A sheet of particles falling onto one shape of every kind; one of them moves.
inline Scene shapes() {
    Scene s;
    s.name = "shapes";
    s.params = defaultParams();
    s.params.radius = 0.1f;
    s.params.dynamicFriction = 0.3f;
    s.params.staticFriction = 0.3f;
    s.params.restitution = 0.2f;
    s.params.numIterations = 3;
    setPlaneBox(s.params, {-3.5f, 0.0f, -2.0f}, {3.5f, 4.0f, 2.0f});
    finishParams(s.params);

    // The shapes sit in a row along x, one unit apart.
    auto at = [](float x, float y) { return Vec3{x, y, 0.0f}; };
    auto place = [](Shape& shape, Vec3 p) {
        shape.position[0] = p.x;
        shape.position[1] = p.y;
        shape.position[2] = p.z;
    };

    {
        Shape sphere;
        sphere.type = ephys::CollisionShapeType::Sphere;
        sphere.geometry[0] = 0.3f;
        place(sphere, at(-2.5f, 0.5f));
        s.shapes.push_back(sphere);
    }
    {
        Shape capsule;
        capsule.type = ephys::CollisionShapeType::Capsule;
        capsule.geometry[0] = 0.2f;
        capsule.geometry[1] = 0.25f;
        place(capsule, at(-1.5f, 0.5f));
        setRotation(capsule, {0, 1, 0}, 0.6f);
        s.shapes.push_back(capsule);
    }
    {
        // The moving one: it is swept from its previous to its current transform in every step.
        Shape box;
        box.type = ephys::CollisionShapeType::Box;
        box.dynamic = true;
        box.geometry[0] = 0.3f;
        box.geometry[1] = 0.15f;
        box.geometry[2] = 0.3f;
        place(box, at(-0.5f, 0.45f));
        setRotation(box, normalize({0.3f, 0, 1}), 0.4f);
        box.velocity[0] = 0.6f;
        box.velocity[1] = 0.9f;
        s.shapes.push_back(box);
    }
    {
        // A convex of eight planes: a cube with two corners cut off.
        Convex convex;
        const float d = 0.25f;
        const float planes[8][4] = {{1, 0, 0, -d},  {-1, 0, 0, -d}, {0, 1, 0, -d}, {0, -1, 0, -d},
                                    {0, 0, 1, -d},  {0, 0, -1, -d}, {0.57735f, 0.57735f, 0.57735f, -0.3f},
                                    {-0.57735f, 0.57735f, -0.57735f, -0.3f}};
        for (const auto& plane : planes) convex.planes.insert(convex.planes.end(), plane, plane + 4);
        for (int k = 0; k < 3; ++k) {
            convex.lower[k] = -d;
            convex.upper[k] = d;
        }
        s.convexes.push_back(convex);

        Shape shape;
        shape.type = ephys::CollisionShapeType::ConvexMesh;
        shape.geometry[0] = 1.0f;
        shape.geometry[1] = 1.2f;
        shape.geometry[2] = 1.0f;
        shape.asset = 0;
        place(shape, at(0.5f, 0.45f));
        setRotation(shape, {0, 1, 0}, 0.3f);
        s.shapes.push_back(shape);
    }
    {
        // A triangle mesh: a tilted, bumpy sheet of 12 x 12 quads.
        Mesh mesh;
        const int n = 13;
        for (int k = 0; k < 3; ++k) {
            mesh.lower[k] = FLT_MAX;
            mesh.upper[k] = -FLT_MAX;
        }
        for (int z = 0; z < n; ++z)
            for (int x = 0; x < n; ++x) {
                const float fx = float(x) / float(n - 1) - 0.5f, fz = float(z) / float(n - 1) - 0.5f;
                const float v[4] = {fx * 0.8f, 0.08f * std::sin(fx * 9.0f) * std::cos(fz * 7.0f) + 0.25f * fx,
                                    fz * 0.8f, 0.0f};
                mesh.vertices.insert(mesh.vertices.end(), v, v + 4);
                for (int k = 0; k < 3; ++k) {
                    mesh.lower[k] = std::min(mesh.lower[k], v[k]);
                    mesh.upper[k] = std::max(mesh.upper[k], v[k]);
                }
            }
        for (int z = 0; z + 1 < n; ++z)
            for (int x = 0; x + 1 < n; ++x) {
                const int32_t a = z * n + x, b = a + 1, c = a + n, d = c + 1;
                mesh.indices.insert(mesh.indices.end(), {a, c, b, b, c, d});  // facing +y
            }
        s.meshes.push_back(mesh);

        Shape shape;
        shape.type = ephys::CollisionShapeType::TriangleMesh;
        shape.geometry[0] = shape.geometry[1] = shape.geometry[2] = 1.0f;
        shape.asset = 0;
        place(shape, at(1.5f, 0.4f));
        s.shapes.push_back(shape);
    }
    {
        // A distance field: a sphere of radius 0.35 in the unit cube the field covers.
        Sdf sdf;
        sdf.dim = 24;
        sdf.field.resize(size_t(sdf.dim) * sdf.dim * sdf.dim);
        for (int z = 0; z < sdf.dim; ++z)
            for (int y = 0; y < sdf.dim; ++y)
                for (int x = 0; x < sdf.dim; ++x) {
                    const Vec3 p = {(float(x) + 0.5f) / float(sdf.dim) - 0.5f, (float(y) + 0.5f) / float(sdf.dim) - 0.5f,
                                    (float(z) + 0.5f) / float(sdf.dim) - 0.5f};
                    sdf.field[(size_t(z) * sdf.dim + y) * sdf.dim + x] = length(p) - 0.35f;
                }
        s.sdfs.push_back(sdf);

        Shape shape;
        shape.type = ephys::CollisionShapeType::Sdf;
        shape.geometry[0] = 0.8f;  // the width of the field in world space
        shape.asset = 0;
        place(shape, Vec3{2.5f - 0.4f, 0.1f, -0.4f});  // the corner of the field
        s.shapes.push_back(shape);
    }

    Jitter random(6);
    // Just above the tops of the shapes, falling: every shape has contacts in the first steps.
    addParticleGrid(s, {-2.9f, 0.84f, -0.35f}, 59, 3, 8, 0.1f, {0, -2.0f, 0}, 1.0f,
                    ephys::makePhase(0, ephys::PhaseSelfCollide), 0.004f, random);
    return s;
}

// Every scene, in the order the tests run them.
inline std::vector<Scene> all() {
    return {granular(), granularGlobal(), fluid(), fluidDiffuse(), cloth(), rigid(), rigidPlastic(), inflatable(), shapes()};
}

// ---- The XPBD reference scenes --------------------------------------------------------------------------------
//
// The four scenes of FleX's XPBD branch (reference/flex/demo/scenes/xpbd{chain,beam,cloth,balloon}.h), which
// reproduce sequences of the paper's supplementary video (Macklin, Mueller, Chentanez, "XPBD: Position-Based
// Simulation of Compliant Constrained Dynamics", MIG 2016). Ported from the scene code and from what the demo
// does around it (demo/main.cpp: the parameter defaults of Init, the ground plane, the derived distances, the
// rest positions and the particle normals it uploads). What is left out is rendering only: the camera, the
// colours, the draw options, the smoothing and anisotropy the demo computes for its fluid renderer.

// The demo's Init around a scene: the ground plane (numPlanes 1, planes[0] = y up), the derived distances,
// the rest positions and the particle normals it uploads (the normalised sum of the triangle normals at a
// particle, or +y).
inline void finishDemoScene(Scene& s) {
    s.params.numPlanes = 1;
    const float ground[4] = {0.0f, 1.0f, 0.0f, 0.0f};
    std::memcpy(s.params.planes[0], ground, sizeof(ground));
    finishParams(s.params);

    s.restPositions = s.positions;
    s.normals.assign(s.positions.size(), 0.0f);
    for (size_t t = 0; t + 2 < s.triangleIndices.size(); t += 3) {
        Vec3 v[3];
        for (int k = 0; k < 3; ++k) {
            const size_t p = size_t(s.triangleIndices[t + size_t(k)]) * 4;
            v[k] = {s.positions[p], s.positions[p + 1], s.positions[p + 2]};
        }
        const Vec3 n = cross(v[1] - v[0], v[2] - v[0]);
        for (int k = 0; k < 3; ++k) {
            const size_t p = size_t(s.triangleIndices[t + size_t(k)]) * 4;
            s.normals[p] += n.x;
            s.normals[p + 1] += n.y;
            s.normals[p + 2] += n.z;
        }
    }
    for (size_t i = 0; i < s.normals.size(); i += 4) {
        const Vec3 n = {s.normals[i], s.normals[i + 1], s.normals[i + 2]};
        const float l = length(n);
        const Vec3 unit = l > 0.0f ? n * (1.0f / l) : Vec3{0.0f, 1.0f, 0.0f};  // SafeNormalize
        s.normals[i] = unit.x;
        s.normals[i + 1] = unit.y;
        s.normals[i + 2] = unit.z;
        s.normals[i + 3] = 0.0f;
    }
}

// helpers.h: CreateSpringGrid, all of it: a dx x dy x dz grid of particles in the x z plane (row index along
// z), two triangles per cell, and the stretch, bending and shear springs in FleX's order.
inline void addSpringGrid(Scene& s, Vec3 lower, int dx, int dy, int dz, float radius, int32_t phase,
                          float stretchStiffness, float bendStiffness, float shearStiffness, Vec3 velocity,
                          float invMass) {
    const int32_t base = int32_t(s.numParticles());
    auto grid = [&](int x, int y) { return base + y * dx + x; };
    for (int z = 0; z < dz; ++z)
        for (int y = 0; y < dy; ++y)
            for (int x = 0; x < dx; ++x) {
                addParticle(s, lower + Vec3{float(x), float(z), float(y)} * radius, invMass, velocity, phase);
                if (x > 0 && y > 0) {
                    s.triangleIndices.insert(s.triangleIndices.end(), {grid(x - 1, y - 1), grid(x, y - 1), grid(x, y)});
                    s.triangleIndices.insert(s.triangleIndices.end(), {grid(x - 1, y - 1), grid(x, y), grid(x - 1, y)});
                    s.triangleNormals.insert(s.triangleNormals.end(), {0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f});
                }
            }
    for (int y = 0; y < dy; ++y)  // horizontal
        for (int x = 0; x < dx; ++x) {
            if (x > 0) addSpring(s, grid(x, y), grid(x - 1, y), stretchStiffness);
            if (x > 1) addSpring(s, grid(x, y), grid(x - 2, y), bendStiffness);
            if (y > 0 && x < dx - 1) addSpring(s, grid(x, y), grid(x + 1, y - 1), shearStiffness);
            if (y > 0 && x > 0) addSpring(s, grid(x, y), grid(x - 1, y - 1), shearStiffness);
        }
    for (int x = 0; x < dx; ++x)  // vertical
        for (int y = 0; y < dy; ++y) {
            if (y > 0) addSpring(s, grid(x, y), grid(x, y - 1), stretchStiffness);
            if (y > 1) addSpring(s, grid(x, y), grid(x, y - 2), bendStiffness);
        }
}

// What every XPBD scene starts from: the demo's parameters (defaultParams), FleX's XPBD defaults, and no
// spare capacity (the demo's g_numExtraParticles).
inline Scene xpbdScene(const char* name) {
    Scene s;
    s.name = name;
    s.params = defaultParams();
    s.mode = ephys::SolverMode::XPBD;
    s.compliance = ephys::XpbdComplianceParams();
    s.spareParticles = 0;
    return s;
}

// (x1) xpbdchain.h, XPBD Chain (paper Sec. 6.2): twenty unit-mass particles joined by distance constraints of
// compliance 1e-8 m/N (the stiffness range pinned to [1e8, 1e8] N/m), hanging from a fixed particle and
// released horizontally; 50 iterations, 2 substeps.
inline Scene xpbdChain() {
    Scene s = xpbdScene("xpbd_chain");
    const int numParticles = 20;
    const float segment = 0.1f;
    const Vec3 anchor = {0.0f, 2.5f, 0.0f};
    const int32_t phase = ephys::makePhase(0, 0);
    for (int i = 0; i < numParticles; ++i)
        addParticle(s, {anchor.x + float(i) * segment, anchor.y, anchor.z}, i == 0 ? 0.0f : 1.0f, {0, 0, 0}, phase);
    for (int i = 0; i < numParticles - 1; ++i) addSpring(s, i, i + 1, 1.0f);

    s.params.radius = 0.05f;
    s.params.dynamicFriction = 0.0f;
    s.params.dissipation = 0.0f;
    s.params.damping = 0.0f;
    s.params.drag = 0.0f;
    s.params.relaxationFactor = 1.0f;
    s.params.maxAcceleration = FLT_MAX;  // the released chain snaps taut; do not clamp it
    s.params.numIterations = 50;
    s.compliance.stiffnessMin = 1.0e8f;
    s.compliance.stiffnessMax = 1.0e8f;
    s.compliance.springDamping = 0.0f;
    s.substeps = 2;
    finishDemoScene(s);
    return s;
}

// (x2) xpbdbeam.h, XPBD Cantilever Beam (paper Sec. 6.3, as a lattice of springs: FleX has no FEM
// constraint): 40 x 8 unit-mass particles, stretch, shear and bending springs of coefficient 0.54
// (~1.7e6 N/m), clamped along the left edge; 20 iterations, 16 substeps.
inline Scene xpbdBeam() {
    Scene s = xpbdScene("xpbd_beam");
    const int dimx = 40, dimy = 8;
    const float spacing = 0.05f;
    const float radius = spacing;
    const float stiffness = 0.54f;
    addSpringGrid(s, {0, 0, 0}, dimx, dimy, 1, spacing, ephys::makePhase(0, 0), stiffness, stiffness, stiffness,
                  {0, 0, 0}, 1.0f);

    // Stood up: the beam runs along +x, its thickness along y; the whole left edge is clamped.
    const float thickness = float(dimy - 1) * spacing;
    const float wallX = 0.5f, top = 2.0f;
    for (uint32_t i = 0; i < s.numParticles(); ++i) {
        const int row = int(i) / dimx, col = int(i) % dimx;
        float* p = &s.positions[size_t(i) * 4];
        p[0] = wallX + float(col) * spacing;
        p[1] = top - float(row) * spacing;
        p[2] = 0.0f;
        if (col == 0) p[3] = 0.0f;
    }

    // The wall (decoration in the reference: the clamp is the pinned edge).
    Shape wall;
    wall.type = ephys::CollisionShapeType::Box;
    wall.geometry[0] = 0.2f;
    wall.geometry[1] = 0.6f;
    wall.geometry[2] = 0.4f;
    wall.position[0] = wallX - 0.2f - radius;
    wall.position[1] = top - thickness * 0.5f;
    s.shapes.push_back(wall);

    s.params.radius = radius;
    s.params.dynamicFriction = 0.25f;
    s.params.dissipation = 0.0f;
    s.params.drag = 0.0f;
    s.params.damping = 0.5f;  // light viscous drag so the beam settles
    s.params.relaxationFactor = 1.0f;
    s.params.numIterations = 20;
    s.compliance.stiffnessMin = 1.0e3f;
    s.compliance.stiffnessMax = 1.0e9f;
    s.compliance.springDamping = 0.0f;
    s.substeps = 16;
    finishDemoScene(s);
    return s;
}

// core/perlin.cpp: Perlin1D (Ken Perlin's improved noise in one dimension), for the gusts of xpbdCloth.
inline float perlin1D(float x, int octaves, float persistence) {
    static const unsigned char c_Perm[256] = {
        151, 160, 137, 91,  90,  15,  131, 13,  201, 95,  96,  53,  194, 233, 7,   225, 140, 36,  103, 30,  69,  142,
        8,   99,  37,  240, 21,  10,  23,  190, 6,   148, 247, 120, 234, 75,  0,   26,  197, 62,  94,  252, 219, 203,
        117, 35,  11,  32,  57,  177, 33,  88,  237, 149, 56,  87,  174, 20,  125, 136, 171, 168, 68,  175, 74,  165,
        71,  134, 139, 48,  27,  166, 77,  146, 158, 231, 83,  111, 229, 122, 60,  211, 133, 230, 220, 105, 92,  41,
        55,  46,  245, 40,  244, 102, 143, 54,  65,  25,  63,  161, 1,   216, 80,  73,  209, 76,  132, 187, 208, 89,
        18,  169, 200, 196, 135, 130, 116, 188, 159, 86,  164, 100, 109, 198, 173, 186, 3,   64,  52,  217, 226, 250,
        124, 123, 5,   202, 38,  147, 118, 126, 255, 82,  85,  212, 207, 206, 59,  227, 47,  16,  58,  17,  182, 189,
        28,  42,  223, 183, 170, 213, 119, 248, 152, 2,   44,  154, 163, 70,  221, 153, 101, 155, 167, 43,  172, 9,
        129, 22,  39,  253, 19,  98,  108, 110, 79,  113, 224, 232, 178, 185, 112, 104, 218, 246, 97,  228, 251, 34,
        242, 193, 238, 210, 144, 12,  191, 179, 162, 241, 81,  51,  145, 235, 249, 14,  239, 107, 49,  192, 214, 31,
        181, 199, 106, 157, 184, 84,  204, 176, 115, 121, 50,  45,  127, 4,   150, 254, 138, 236, 205, 93,  222, 114,
        67,  29,  24,  72,  243, 141, 128, 195, 78,  66,  215, 61,  156, 180};
    // FleX's NoisePerm is this table twice, so index ix + 1 = 256 is entry 0 again.
    auto grad = [&](int ix, float dx) { return (c_Perm[ix & 255] & 1) ? -dx : dx; };
    auto noise = [&](float v) {
        int ix = int(std::floor(v));
        const float dx = v - float(ix);
        ix &= 255;
        const float w00 = grad(ix, dx);
        const float w10 = grad(ix + 1, dx - 1.0f);
        const float dx3 = dx * dx * dx;
        const float dx4 = dx3 * dx;
        const float wx = 6.0f * dx4 * dx - 15.0f * dx4 + 10.0f * dx3;
        return w00 + wx * (w10 - w00);
    };
    float r = 0.0f;
    float a = 1.0f;
    int freq = 1;
    for (int i = 0; i < octaves; ++i) {
        r += noise(x * float(freq)) * a;
        a *= persistence;
        freq = 2 << i;
    }
    return r;
}

// The constants of xpbdcloth.h.
struct XpbdClothConstants {
    static constexpr int kDim = 64;
    static constexpr float kSpacing = 0.05f;
    static constexpr float kPinHeight = 4.90f;
    static constexpr float kBarLength = 1.456f;
    static constexpr float kBarRadius = 0.063f;
    static constexpr float kXpbdStretch = 0.452f;
    static constexpr float kXpbdShear = 0.25f;
    static constexpr float kXpbdBend = 0.05f;
    static constexpr uint32_t kXpbdSubsteps = 16;
    static constexpr float kSeedRipple = 0.05f;
    static constexpr float kGustRate = 0.7f;
    static constexpr float kWindStrength = 3.0f;
    static constexpr float kGustBias = 0.0f;
    static constexpr float kGustAmp = 1.4f;
    static constexpr float kDrag = 1.0f;
    static constexpr float kLift = 0.2f;
    static constexpr float kDamping = 0.0f;
    static constexpr float kWindDrift = 0.3f;
    static constexpr float kWindDepth = -2.0f;
};

// XPBDHangingCloth::Update, after the demo's UpdateWind has advanced g_windTime by dt: step n blows with the
// gust at (n + 1) * dt.
inline void xpbdClothWind(const Scene& scene, uint32_t step, ephys::PhysParams& params) {
    using C = XpbdClothConstants;
    const float windTime = scene.dt * float(step + 1);
    const float gust = C::kGustBias + C::kGustAmp * perlin1D(windTime * C::kGustRate, 4, 0.5f);
    const Vec3 wind = Vec3{C::kWindDrift, 0.0f, C::kWindDepth} * (C::kWindStrength * gust);
    params.wind[0] = wind.x;
    params.wind[1] = wind.y;
    params.wind[2] = wind.z;
}

// (x3) xpbdcloth.h, XPBD Hanging Cloth (paper Fig. 6 and the video's "Hanging Cloth"): a 64 x 64 cloth (4096
// particles, 23938 springs, the video's "24k constraints") hung by its two top corners below a bar, in a
// gusting wind; 20 iterations, 16 substeps.
//
// With `wind` false the air is still (no wind, no drag, no lift) and `damping` is the viscous damping of the
// particles, so that the sheet comes to rest: the scene of the substep convergence test. (The reference
// takes its own measurement of that, in the comment of kXpbdSubsteps, with the wind on.)
inline Scene xpbdCloth(bool wind = true, float damping = XpbdClothConstants::kDamping) {
    using C = XpbdClothConstants;
    Scene s = xpbdScene(wind ? "xpbd_cloth" : "xpbd_cloth_still");
    const int dim = C::kDim;
    const float spacing = C::kSpacing;
    // A single sheet that never folds onto itself: no self-collision.
    addSpringGrid(s, {0, 0, 0}, dim, dim, 1, spacing, ephys::makePhase(0, 0), C::kXpbdStretch, C::kXpbdBend,
                  C::kXpbdShear, {0, 0, 0}, 1.0f);

    // Stood up with row 0 on top at the pin height, with a few millimetres of out-of-plane ripple that picks
    // the buckling mode of the folds.
    const float width = float(dim - 1) * spacing;
    const float top = C::kPinHeight;
    for (uint32_t i = 0; i < s.numParticles(); ++i) {
        const int row = int(i) / dim, col = int(i) % dim;
        const float u = float(col) / float(dim - 1);
        const float v = float(row) / float(dim - 1);
        const float ripple = C::kSeedRipple * spacing * v * (std::sin(11.0f * u) + 0.5f * std::sin(7.0f * u + 2.0f));
        float* p = &s.positions[size_t(i) * 4];
        p[0] = float(col) * spacing - width * 0.5f;
        p[1] = top - float(row) * spacing;
        p[2] = ripple;
    }
    // The two top corners hang at the cloth's full rest width.
    s.positions[3] = 0.0f;
    s.positions[size_t(dim - 1) * 4 + 3] = 0.0f;

    // The bar: a capsule along x whose drawn length is kBarLength * width (the half-height excludes the caps),
    // its centre one radius above the pin row and a little towards the camera.
    Shape bar;
    bar.type = ephys::CollisionShapeType::Capsule;
    bar.geometry[0] = C::kBarRadius;
    bar.geometry[1] = 0.5f * (C::kBarLength * width - 2.0f * C::kBarRadius);
    bar.position[0] = 0.0f;
    bar.position[1] = top + 1.1f * C::kBarRadius;
    bar.position[2] = 0.5f * C::kBarRadius;
    s.shapes.push_back(bar);

    s.params.radius = spacing;
    s.params.dynamicFriction = 0.25f;
    s.params.dissipation = 0.0f;
    s.params.drag = wind ? C::kDrag : 0.0f;
    s.params.lift = wind ? C::kLift : 0.0f;
    s.params.damping = damping;
    s.params.relaxationMode = ephys::RelaxationMode::Local;  // XPBD's near-rigid constraints need it
    s.params.relaxationFactor = 1.0f;
    s.params.numIterations = 20;
    s.compliance.stiffnessMin = 1.0e3f;
    s.compliance.stiffnessMax = 1.0e9f;
    s.compliance.springDamping = 0.0f;
    s.substeps = C::kXpbdSubsteps;
    finishDemoScene(s);
    if (wind) {
        s.update = xpbdClothWind;
        xpbdClothWind(s, 0, s.params);
    }
    return s;
}

// A balloon of xpbdballoon.h: a closed triangle mesh as one inflatable, with core/cloth.h's ClothMesh for its
// springs (one stretch spring per edge and one bending spring across every edge between two triangles, in the
// order of its sorted edge set), its rest volume and its constraint scale.
inline void addBalloon(Scene& s, const std::vector<Vec3>& vertices, const std::vector<int32_t>& indices,
                       float stiffness, int32_t phase) {
    const int32_t startVertex = int32_t(s.numParticles());
    for (const Vec3& v : vertices) addParticle(s, v, 1.0f, {0, 0, 0}, phase);

    const int32_t triOffset = int32_t(s.triangleIndices.size());
    const int32_t triCount = int32_t(indices.size() / 3);
    std::vector<int32_t> tris;
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        const int32_t a = indices[i], b = indices[i + 1], c = indices[i + 2];
        const Vec3 n = normalize(cross(vertices[size_t(b)] - vertices[size_t(a)],
                                       vertices[size_t(c)] - vertices[size_t(a)])) * -1.0f;
        s.triangleNormals.insert(s.triangleNormals.end(), {n.x, n.y, n.z});
        tris.insert(tris.end(), {a + startVertex, b + startVertex, c + startVertex});
    }
    s.triangleIndices.insert(s.triangleIndices.end(), tris.begin(), tris.end());

    // ClothMesh (tearable): the edges in the order of a std::set of (min, max) pairs, each with the first two
    // triangles that use it.
    struct Edge {
        int32_t tris[2] = {-1, -1};
    };
    std::map<std::pair<int32_t, int32_t>, Edge> edges;
    for (size_t t = 0; t + 2 < tris.size(); t += 3)
        for (size_t k = 0; k < 3; ++k) {
            Edge& e = edges[std::minmax(tris[t + k], tris[t + (k + 1) % 3])];
            if (e.tris[0] == -1)
                e.tris[0] = int32_t(t / 3);
            else if (e.tris[1] == -1)
                e.tris[1] = int32_t(t / 3);
        }
    auto opposite = [&](int32_t tri, const std::pair<int32_t, int32_t>& edge) {
        for (size_t k = 0; k < 3; ++k) {
            const int32_t v = tris[size_t(tri) * 3 + k];
            if (v != edge.first && v != edge.second) return v;
        }
        return int32_t(-1);
    };
    for (const auto& entry : edges) {
        const std::pair<int32_t, int32_t>& edge = entry.first;
        const Edge& e = entry.second;
        addSpring(s, edge.first, edge.second, stiffness);
        if (e.tris[0] != -1 && e.tris[1] != -1 && stiffness > 0.0f)
            addSpring(s, opposite(e.tris[0], edge), opposite(e.tris[1], edge), stiffness);
    }

    float restVolume = 0.0f;
    std::vector<Vec3> gradients(vertices.size());
    for (size_t t = 0; t + 2 < indices.size(); t += 3) {
        const Vec3 v1 = vertices[size_t(indices[t])];
        const Vec3 v2 = vertices[size_t(indices[t + 1])];
        const Vec3 v3 = vertices[size_t(indices[t + 2])];
        const Vec3 n = cross(v2 - v1, v3 - v1);
        restVolume += dot(v1, n);
        for (size_t k = 0; k < 3; ++k) gradients[size_t(indices[t + k])] = gradients[size_t(indices[t + k])] + n;
    }
    float constraintScale = 0.0f;
    for (const Vec3& g : gradients) constraintScale += dot(g, g);

    ephys::InflatableDesc inflatable;
    inflatable.startTri = triOffset / 3;
    inflatable.numTris = triCount;
    inflatable.restVolume = restVolume;
    inflatable.overPressure = 1.0f;  // the rest volume, no over-pressure
    inflatable.constraintScale = 1.0f / constraintScale;
    s.inflatables.push_back(inflatable);
}

// (x4) xpbdballoon.h, XPBD Inflatable Balloon (paper Fig. 1 and the video's "Inflatable Balloon"): four
// balloons on the floor, each a closed cloth surface with a hard volume constraint; the surface coefficients
// 0.35 / 0.5 / 0.65 / 0.85 on [10, 1e6] N/m rise from left to right; 20 iterations, 4 substeps. The demo loads
// data/sphere.ply, a sphere of 642 vertices and 1280 triangles; here it is the icosphere of three
// subdivisions, which has those counts, normalised to a unit diameter as Mesh::Normalize does.
inline Scene xpbdBalloon() {
    Scene s = xpbdScene("xpbd_balloon");
    const float stiffness[4] = {0.35f, 0.5f, 0.65f, 0.85f};
    const float spacing = 2.0f;
    std::vector<Vec3> sphere;
    std::vector<int32_t> indices;
    icosphere(3, 1.0f, sphere, indices);
    for (int i = 0; i < 4; ++i) {
        // Mesh::Normalize: the lower corner of the bounds at the origin, their longest edge 1; then the offset.
        Vec3 lower = {FLT_MAX, FLT_MAX, FLT_MAX}, upper = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
        for (const Vec3& v : sphere) {
            lower = {std::min(lower.x, v.x), std::min(lower.y, v.y), std::min(lower.z, v.z)};
            upper = {std::max(upper.x, v.x), std::max(upper.y, v.y), std::max(upper.z, v.z)};
        }
        const Vec3 edges = upper - lower;
        const float scale = 1.0f / std::max(edges.x, std::max(edges.y, edges.z));
        const Vec3 offset = {(float(i) - 1.5f) * spacing - 0.5f, 0.6f, -0.5f};
        std::vector<Vec3> vertices;
        for (const Vec3& v : sphere) vertices.push_back((v - lower) * scale + offset);
        addBalloon(s, vertices, indices, stiffness[i], ephys::makePhase(i, 0));
    }

    s.params.radius = 0.1f;
    s.params.dynamicFriction = 0.4f;
    s.params.dissipation = 0.0f;
    s.params.damping = 0.5f;  // light viscous drag so the balloons settle
    s.params.particleCollisionMargin = s.params.radius * 0.05f;
    s.params.drag = 0.0f;
    s.params.collisionDistance = 0.01f;
    s.params.relaxationFactor = 1.0f;
    s.params.numIterations = 20;
    s.compliance.stiffnessMin = 10.0f;
    s.compliance.stiffnessMax = 1.0e6f;
    s.compliance.springDamping = 0.0f;
    s.compliance.volumeCompliance = 0.0f;
    s.substeps = 4;
    finishDemoScene(s);
    return s;
}

// The four XPBD scenes, in the order the tests run them.
inline std::vector<Scene> xpbdAll() { return {xpbdChain(), xpbdBeam(), xpbdCloth(), xpbdBalloon()}; }

}  // namespace scenes

#endif /* ETHEREAL_PHYS_TESTS_SOLVER_SCENES_H */
