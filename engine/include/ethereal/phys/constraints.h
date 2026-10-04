#ifndef ETHEREAL_PHYS_CONSTRAINTS_H
#define ETHEREAL_PHYS_CONSTRAINTS_H

// The four constraint sets of a domain (domain.h): springs, shape-matching rigids, dynamic triangles and
// inflatables.
//
// They are data: each owns the arrays the application describes its constraints with, and dispatches no
// kernel. What a solver derives from them stays in the solver: the half-spring arrays it sorts the springs
// into, the pressure terms of the inflatables, the Lagrange multipliers of XPBD. A solver notices a change
// by comparing revisions (getContentRevision, getLayoutRevision).
//
// They replace NvFlexSetSprings / GetSprings, NvFlexSetRigids / GetRigids, NvFlexSetDynamicTriangles /
// GetDynamicTriangles and NvFlexSetInflatables. As there, an upload replaces the whole set and names its
// counts, and a set grows when an upload needs more room than it has.
//
// Capacity. A set starts with the capacity DomainDesc reserves (0: nothing is allocated). An upload of
// more elements than the capacity replaces the buffers by larger ones (the count plus half of it, FleX's
// slack) before it copies; the layout revision then advances and the buffers returned before are no longer
// the set's. With a capacity of 0 the buffer accessors return nullptr.
//
// Host memory is an IPhysBuffer with BufferUse::Upload or BufferUse::Readback and a transfer is a GPU copy
// recorded on a command list: the application's, or the device's own for nullptr (device.h). The one
// exception is IInflatableSet::upload, which reads host memory directly (see there).
//
// Errors. FE_INVALID_ARGS, one message and nothing copied or changed for: a required buffer that is null,
// a buffer with another element stride than the array, a source buffer that holds fewer elements than the
// count, a Readback source or an Upload destination, and a download of more elements than the set holds.
//
// The interfaces follow donut/nvrhi/doc/com-interface-abi.md.

#include <ethereal/phys/buffer.h>
#include <ethereal/phys/params.h>

#include <nvrhi/nvrhi.h>

#include <cstddef>
#include <cstdint>

namespace ephys {

// Distance constraints between two particles (IPhysDomain::getSprings).
//
//   array        element  bytes  elements
//   Indices      int32        4  2 per spring: the two particles
//   RestLengths  float        4  1 per spring
//   Stiffness    float        4  1 per spring
//
// There is one stiffness array, for both solvers. A PBD solver reads k in [-1, 1] as a multiplier per
// iteration; an XPBD solver maps k geometrically onto [stiffnessMin, stiffnessMax] (XpbdComplianceParams).
// Under both a negative k is a tether: a constraint that resists stretching only.
NVRHI_IID(ISpringSet, "a2f06df0-0091-489e-9235-1159cc2e5345")
struct ISpringSet : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(ISpringSet)

    [[nodiscard]] virtual uint32_t getMaxSprings() const noexcept = 0;  // the capacity
    [[nodiscard]] virtual uint32_t getNumSprings() const noexcept = 0;

    // The arrays; they do not AddRef. nullptr while the capacity is 0.
    [[nodiscard]] virtual IPhysBuffer* getIndices() const noexcept = 0;      // 2 * getMaxSprings() elements
    [[nodiscard]] virtual IPhysBuffer* getRestLengths() const noexcept = 0;  // getMaxSprings() elements
    [[nodiscard]] virtual IPhysBuffer* getStiffness() const noexcept = 0;    // getMaxSprings() elements

    // Replaces the set by numSprings springs (NvFlexSetSprings): records copies of 2 * numSprings indices
    // and numSprings rest lengths and stiffness coefficients, from the start of each buffer, and advances
    // the content revision. With numSprings 0 the set is emptied and the buffers are not looked at.
    virtual nvrhi::FRESULT upload(_In_opt_ nvrhi::ICommandList* commandList, _In_opt_ IPhysBuffer* indices,
                                  _In_opt_ IPhysBuffer* restLengths, _In_opt_ IPhysBuffer* stiffness,
                                  uint32_t numSprings) noexcept = 0;

    // Records copies of the first numSprings springs into the buffers given (NvFlexGetSprings); a null
    // buffer is skipped. numSprings is at most getNumSprings().
    virtual nvrhi::FRESULT download(_In_opt_ nvrhi::ICommandList* commandList, _In_opt_ IPhysBuffer* indices,
                                    _In_opt_ IPhysBuffer* restLengths, _In_opt_ IPhysBuffer* stiffness,
                                    uint32_t numSprings) noexcept = 0;

    // Advances when the buffers are replaced (an upload beyond the capacity). Starts at 0.
    [[nodiscard]] virtual uint64_t getLayoutRevision() const noexcept = 0;
    // Advances with every upload and every markContentChanged. Starts at 0.
    [[nodiscard]] virtual uint64_t getContentRevision() const noexcept = 0;
    // For a writer the set cannot see: advances the content revision and returns the new value.
    virtual uint64_t markContentChanged() noexcept = 0;
};
typedef nvrhi::AutoPtr<ISpringSet> SpringSetHandle;

// The buffers of IShapeMatchSet::upload and download, one per array. All pointers are borrowed for the
// call.
struct ShapeMatchBuffers {
    IPhysBuffer* offsets = nullptr;            // int32, numRigids + 1
    IPhysBuffer* indices = nullptr;            // int32, numIndices
    IPhysBuffer* restPositions = nullptr;      // float3 (12 bytes), numIndices
    IPhysBuffer* restNormals = nullptr;        // float4, numIndices; optional
    IPhysBuffer* stiffness = nullptr;          // float, numRigids
    IPhysBuffer* plasticThresholds = nullptr;  // float, numRigids; optional, together with plasticCreeps
    IPhysBuffer* plasticCreeps = nullptr;      // float, numRigids; optional, together with plasticThresholds
    IPhysBuffer* rotations = nullptr;          // float4 quaternion, numRigids
    IPhysBuffer* translations = nullptr;       // float3 (12 bytes), numRigids
};

static_assert(sizeof(ShapeMatchBuffers) == 72, "ShapeMatchBuffers layout changed: bump c_HeaderVersion");
static_assert(offsetof(ShapeMatchBuffers, translations) == 64);

// Shape-matching constraints (IPhysDomain::getShapeMatches): each one, a "rigid" in FleX's words, pulls a
// group of particles towards the best rigid transform of their rest pose. A rigid body is one of them at
// stiffness 1, a soft body is many overlapping ones (clusters). A particle should belong to one rigid.
//
//   array              element            bytes  elements        FleX
//   Offsets            int32                  4  numRigids + 1   the first entry is 0; rigid j owns
//                                                                Indices[Offsets[j]] .. Indices[Offsets[j+1]]
//   Indices            int32                  4  numIndices      particle indices
//   RestPositions      float3                12  numIndices      relative to the centre of mass of the rigid
//   RestNormals        float4                16  numIndices      w: the (negative) signed distance inside
//                                                                the shape
//   Stiffness          float                  4  numRigids       in [0, 1]
//   PlasticThresholds  float                  4  numRigids       the deformation above which a rigid yields
//   PlasticCreeps      float                  4  numRigids       in [0, 1]: how fast it yields
//   Rotations          float4 quaternion     16  numRigids       x y z imaginary
//   Translations       float3                12  numRigids       the centres of mass
//
// Rotations and Translations are inputs at upload and outputs afterwards: a solver writes the transform of
// every rigid there in each step, and the application reads them from here (FleX's NvFlexGetRigids). With
// plastic deformation a solver also changes RestPositions.
NVRHI_IID(IShapeMatchSet, "b2735d9d-a206-4e4d-b36a-81bf2bc60994")
struct IShapeMatchSet : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IShapeMatchSet)

    [[nodiscard]] virtual uint32_t getMaxRigids() const noexcept = 0;  // the capacity in rigids
    [[nodiscard]] virtual uint32_t getNumRigids() const noexcept = 0;
    [[nodiscard]] virtual uint32_t getMaxIndices() const noexcept = 0;  // the capacity in particle indices
    [[nodiscard]] virtual uint32_t getNumIndices() const noexcept = 0;

    // True when the last upload gave plastic thresholds and creeps. Otherwise getPlasticThresholds() and
    // getPlasticCreeps() return nullptr and a solver runs the kernels without plastic deformation.
    [[nodiscard]] virtual bool hasPlasticDeformation() const noexcept = 0;

    // The arrays; they do not AddRef. nullptr while the capacity is 0. Offsets has getMaxRigids() + 1
    // elements; Stiffness, Rotations and Translations (and the two plastic arrays) getMaxRigids();
    // Indices, RestPositions and RestNormals getMaxIndices().
    [[nodiscard]] virtual IPhysBuffer* getOffsets() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getIndices() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getRestPositions() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getRestNormals() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getStiffness() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getPlasticThresholds() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getPlasticCreeps() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getRotations() const noexcept = 0;
    [[nodiscard]] virtual IPhysBuffer* getTranslations() const noexcept = 0;

    // Replaces the set by numRigids rigids over numIndices particle indices (NvFlexSetRigids), copying
    // from the start of each buffer, and advances the content revision. As in FleX:
    // - restNormals may be null: the normals are then zero;
    // - plasticThresholds and plasticCreeps are given together or not at all (hasPlasticDeformation);
    // - every other buffer is required;
    // - with numRigids 0 the set is emptied and the buffers are not looked at.
    virtual nvrhi::FRESULT upload(_In_opt_ nvrhi::ICommandList* commandList, const ShapeMatchBuffers& buffers,
                                  uint32_t numRigids, uint32_t numIndices) noexcept = 0;

    // Records copies of the whole set (getNumRigids() rigids, getNumIndices() indices) into the buffers
    // given (NvFlexGetRigids); a null buffer is skipped. A plastic buffer is FE_INVALID_ARGS when the set
    // has no plastic deformation. Nothing is recorded for an empty set.
    virtual nvrhi::FRESULT download(_In_opt_ nvrhi::ICommandList* commandList,
                                    const ShapeMatchBuffers& buffers) noexcept = 0;

    [[nodiscard]] virtual uint64_t getLayoutRevision() const noexcept = 0;   // as ISpringSet
    [[nodiscard]] virtual uint64_t getContentRevision() const noexcept = 0;  // as ISpringSet
    virtual uint64_t markContentChanged() noexcept = 0;                      // as ISpringSet
};
typedef nvrhi::AutoPtr<IShapeMatchSet> ShapeMatchSetHandle;

// The triangles between particles (IPhysDomain::getTriangles): cloth. A solver computes their normals,
// applies wind, drag and lift to them (PhysParams) and derives the vertex normals of their particles.
// Inflatables are ranges of them.
//
//   array    element  bytes  elements
//   Indices  int32        4  3 per triangle: particle indices
//   Normals  float3      12  1 per triangle: written by a solver
NVRHI_IID(ITriangleSet, "ccf63e22-ded6-4367-9931-843ba36d4ad8")
struct ITriangleSet : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(ITriangleSet)

    [[nodiscard]] virtual uint32_t getMaxTriangles() const noexcept = 0;  // the capacity
    [[nodiscard]] virtual uint32_t getNumTriangles() const noexcept = 0;

    // The arrays; they do not AddRef. nullptr while the capacity is 0.
    [[nodiscard]] virtual IPhysBuffer* getIndices() const noexcept = 0;  // 3 * getMaxTriangles() elements
    [[nodiscard]] virtual IPhysBuffer* getNormals() const noexcept = 0;  // getMaxTriangles() elements

    // Replaces the set by numTriangles triangles (NvFlexSetDynamicTriangles) and advances the content
    // revision. normals may be null, as in FleX: the normals are then whatever the buffer held, until a
    // solver computes them. With numTriangles 0 the set is emptied.
    virtual nvrhi::FRESULT upload(_In_opt_ nvrhi::ICommandList* commandList, _In_opt_ IPhysBuffer* indices,
                                  _In_opt_ IPhysBuffer* normals, uint32_t numTriangles) noexcept = 0;

    // Records copies of the first numTriangles triangles into the buffers given
    // (NvFlexGetDynamicTriangles); a null buffer is skipped. numTriangles is at most getNumTriangles().
    virtual nvrhi::FRESULT download(_In_opt_ nvrhi::ICommandList* commandList, _In_opt_ IPhysBuffer* indices,
                                    _In_opt_ IPhysBuffer* normals, uint32_t numTriangles) noexcept = 0;

    [[nodiscard]] virtual uint64_t getLayoutRevision() const noexcept = 0;   // as ISpringSet
    [[nodiscard]] virtual uint64_t getContentRevision() const noexcept = 0;  // as ISpringSet
    virtual uint64_t markContentChanged() noexcept = 0;                      // as ISpringSet
};
typedef nvrhi::AutoPtr<ITriangleSet> TriangleSetHandle;

// One inflatable as the application describes it: the arguments of NvFlexSetInflatables, one struct per
// inflatable instead of five parallel arrays.
struct InflatableDesc {
    int32_t startTri = 0;          // the first triangle of the closed mesh in the triangle set
    int32_t numTris = 0;           // the number of its triangles (wound counter-clockwise)
    float restVolume = 0.0f;       // the volume of the mesh at rest
    float overPressure = 1.0f;     // 1: the rest volume; above: over-inflated; below: under-inflated
    float constraintScale = 0.0f;  // roughly a stiffness; it includes the PBD constraint scaling
};

static_assert(sizeof(InflatableDesc) == 20, "InflatableDesc layout changed: bump c_HeaderVersion");
static_assert(offsetof(InflatableDesc, constraintScale) == 16);

// One element of IInflatableSet::getInflatables(): what the kernels read (FlexInflatable in
// engine/shaders/KernelParams.hlsli, FleX's InflatableDevice).
struct Inflatable {
    int32_t startTri = 0;
    int32_t numTris = 0;
    float restVolume = 0.0f;  // the volume the constraint aims at: InflatableDesc::restVolume * overPressure
    float constraintScale = 0.0f;
};

static_assert(sizeof(Inflatable) == 16, "Inflatable layout changed: bump c_HeaderVersion (and the kernels)");
static_assert(offsetof(Inflatable, restVolume) == 8);
static_assert(offsetof(Inflatable, constraintScale) == 12);

// Volume constraints over closed triangle meshes (IPhysDomain::getInflatables).
//
// The set keeps one array, of Inflatable: the form the kernels read, and the only form FleX keeps. That
// form is computed from the description (the rest volume is multiplied by the over-pressure), which a GPU
// copy cannot do, so upload takes the descriptions as host memory and writes the array with a buffer
// write. It is the one upload of the constraint sets that is not a copy from an Upload buffer: FleX maps
// the five buffers it is given on the host for the same reason, and a set holds a handful of inflatables.
NVRHI_IID(IInflatableSet, "cd707f92-1c29-40bc-8f6c-a90afeef414e")
struct IInflatableSet : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IInflatableSet)

    [[nodiscard]] virtual uint32_t getMaxInflatables() const noexcept = 0;  // the capacity
    [[nodiscard]] virtual uint32_t getNumInflatables() const noexcept = 0;

    // The array: getMaxInflatables() elements of Inflatable. Does not AddRef. nullptr while the capacity
    // is 0.
    [[nodiscard]] virtual IPhysBuffer* getInflatables() const noexcept = 0;

    // Replaces the set by numInflatables inflatables (NvFlexSetInflatables): records a write of the array
    // and advances the content revision. inflatables is read before the call returns. With numInflatables
    // 0 the set is emptied. FE_INVALID_ARGS for a null pointer with a count.
    virtual nvrhi::FRESULT upload(_In_opt_ nvrhi::ICommandList* commandList,
                                  _In_opt_ const InflatableDesc* inflatables,
                                  uint32_t numInflatables) noexcept = 0;

    // Records a copy of the first numInflatables elements of the array into inflatables (16-byte
    // elements, Inflatable). numInflatables is at most getNumInflatables(). FleX has no such call.
    virtual nvrhi::FRESULT download(_In_opt_ nvrhi::ICommandList* commandList, IPhysBuffer* inflatables,
                                    uint32_t numInflatables) noexcept = 0;

    [[nodiscard]] virtual uint64_t getLayoutRevision() const noexcept = 0;   // as ISpringSet
    [[nodiscard]] virtual uint64_t getContentRevision() const noexcept = 0;  // as ISpringSet
    virtual uint64_t markContentChanged() noexcept = 0;                      // as ISpringSet
};
typedef nvrhi::AutoPtr<IInflatableSet> InflatableSetHandle;

}  // namespace ephys

#endif /* ETHEREAL_PHYS_CONSTRAINTS_H */
