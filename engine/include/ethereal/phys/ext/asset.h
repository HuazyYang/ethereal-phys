#ifndef ETHEREAL_PHYS_EXT_ASSET_H
#define ETHEREAL_PHYS_EXT_ASSET_H

// IPhysAsset and IPhysTearingCloth: a group of particles and constraints built on the host from a mesh by the
// asset builder (ext/asset_builder.h), ready to be uploaded into a domain (step 9 of the migration plan).
//
// They replace FleX's NvFlexExtAsset (reference/flex/include/NvFlexExt.h). That struct held 20 raw pointers
// allocated by the extension library with its own new[] and freed by NvFlexExtDestroyAsset, so the
// application freed memory of another module's heap through the library and wrote into it at will. Here the
// asset is an object: every array is read through a const accessor, the storage belongs to the library, and
// Release replaces NvFlexExtDestroyAsset and NvFlexExtDestroyTearingCloth.
//
// An asset is host data: it owns no buffer and dispatches nothing. The arrays have the element types the
// constraint sets of a domain take (constraints.h), so they can be copied into Upload buffers as they are,
// with one difference noted at getShapeOffsets.
//
// The interfaces follow donut/nvrhi/doc/com-interface-abi.md.

#include <nvrhi/nvrhi.h>

#include <cstddef>
#include <cstdint>

namespace ephys {

// ---- IPhysAsset ------------------------------------------------------------------------------------------

// The particles and constraints of one body: a cloth, a rigid body or a soft body (NvFlexExtAsset). Read
// only and immutable once built, except for a tearing cloth (IPhysTearingCloth below), whose topology the
// tearing operation changes.
//
// Every pointer is a non-owning view of the asset's storage, valid while the asset lives (for a tearing
// cloth: until the next call of IPhysTearingCloth that changes it); nullptr when its count is 0. Indices are
// into the particles of the asset: an application that places the asset at an offset in a domain adds that
// offset when it uploads.
NVRHI_IID(IPhysAsset, "a6d68dad-5a7e-4d5b-8f64-692ebb39b8ea")
struct IPhysAsset : nvrhi::IObject {
    NVRHI_DECLARE_UUID_TRAITS(IPhysAsset)

    // Particles: x, y, z, 1/mass per particle (ParticleArray::Position), getParticleCount() of them in use.
    // The storage holds getMaxParticles(): room for the particles tearing adds (equal to the count for every
    // other asset). NvFlexExtAsset::particles, numParticles, maxParticles.
    [[nodiscard]] virtual const float* getParticles() const noexcept = 0;
    [[nodiscard]] virtual uint32_t getParticleCount() const noexcept = 0;
    [[nodiscard]] virtual uint32_t getMaxParticles() const noexcept = 0;

    // Springs (ISpringSet): two particle indices, a stiffness coefficient (negative: a tether) and a rest
    // length per spring. NvFlexExtAsset::springIndices, springCoefficients, springRestLengths, numSprings.
    [[nodiscard]] virtual const int32_t* getSpringIndices() const noexcept = 0;  // 2 * getSpringCount()
    [[nodiscard]] virtual const float* getSpringCoefficients() const noexcept = 0;
    [[nodiscard]] virtual const float* getSpringRestLengths() const noexcept = 0;
    [[nodiscard]] virtual uint32_t getSpringCount() const noexcept = 0;

    // Shape-matching constraints (IShapeMatchSet). NvFlexExtAsset::shapeIndices, numShapeIndices,
    // shapeOffsets, shapeCoefficients, shapeCenters, numShapes, shapePlasticThresholds, shapePlasticCreeps.
    //
    // getShapeOffsets() has getShapeCount() entries: the END of each shape's run in getShapeIndices()
    // (exclusive), FleX's layout. IShapeMatchSet's offsets have one entry more, a leading 0.
    // getShapeCenters() holds the centre of mass of each shape, x y z (3 floats per shape). The two plastic
    // arrays exist only for a soft body built with a plastic creep (null otherwise).
    [[nodiscard]] virtual const int32_t* getShapeIndices() const noexcept = 0;
    [[nodiscard]] virtual uint32_t getShapeIndexCount() const noexcept = 0;
    [[nodiscard]] virtual const int32_t* getShapeOffsets() const noexcept = 0;
    [[nodiscard]] virtual const float* getShapeCoefficients() const noexcept = 0;
    [[nodiscard]] virtual const float* getShapeCenters() const noexcept = 0;
    [[nodiscard]] virtual uint32_t getShapeCount() const noexcept = 0;
    [[nodiscard]] virtual const float* getShapePlasticThresholds() const noexcept = 0;
    [[nodiscard]] virtual const float* getShapePlasticCreeps() const noexcept = 0;

    // The triangles of a cloth (ITriangleSet), three indices each. NvFlexExtAsset::triangleIndices,
    // numTriangles.
    [[nodiscard]] virtual const int32_t* getTriangleIndices() const noexcept = 0;
    [[nodiscard]] virtual uint32_t getTriangleCount() const noexcept = 0;

    // A volume constraint over the triangles (IInflatableSet): set when a cloth was built with a pressure.
    // The volume is InflatableDesc::restVolume, the pressure its overPressure and the stiffness its
    // constraintScale (constraints.h). NvFlexExtAsset::inflatable, inflatableVolume, inflatablePressure,
    // inflatableStiffness.
    [[nodiscard]] virtual bool isInflatable() const noexcept = 0;
    [[nodiscard]] virtual float getInflatableVolume() const noexcept = 0;
    [[nodiscard]] virtual float getInflatablePressure() const noexcept = 0;
    [[nodiscard]] virtual float getInflatableStiffness() const noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysAsset> PhysAssetHandle;

// ---- Tearing ---------------------------------------------------------------------------------------------

// A particle the application copies after a tear (NvFlexExtTearingParticleClone): copy the particle srcIndex
// of its arrays (position, velocity, phase, rest position, ...) to destIndex. Both are indices into the
// particles of the asset.
struct TearingParticleClone {
    int32_t srcIndex = 0;
    int32_t destIndex = 0;
};

static_assert(sizeof(TearingParticleClone) == 8, "TearingParticleClone layout changed: bump c_HeaderVersion");
static_assert(offsetof(TearingParticleClone, destIndex) == 4);

// An entry of the triangle indices the application changes after a tear (NvFlexExtTearingMeshEdit):
// triIndex is a position in the index array (triangle triIndex / 3, corner triIndex % 3), newParticleIndex
// its new value.
struct TearingMeshEdit {
    int32_t triIndex = 0;
    int32_t newParticleIndex = 0;
};

static_assert(sizeof(TearingMeshEdit) == 8, "TearingMeshEdit layout changed: bump c_HeaderVersion");
static_assert(offsetof(TearingMeshEdit, newParticleIndex) == 4);

// A cloth that tears (IPhysAssetBuilder::createTearingClothFromMesh): an asset that also keeps the topology
// of its mesh (half-edges of FleX's core/cloth.h), so that a vertex can be split in two along a strained
// edge. It answers IID_IPhysAsset as well. Each instance in a simulation needs its own: the topology is the
// instance's. Not thread-safe.
//
// What an application does every frame (FleX's demo/scenes/tearing.h):
//
//   1. setParticles with the positions the solver left (readback of ParticleArray::Position);
//   2. tearClothMesh;
//   3. for each clone, copy the particle srcIndex to destIndex in its own arrays (and in the domain's), and
//      make destIndex active;
//   4. apply each edit to its triangle indices (the asset's getTriangleIndices() already has them);
//   5. upload the springs again: getSpringIndices() changed (the coefficients and rest lengths may have, too).
//
// getParticleCount() includes the clones after step 2 (FleX's NvFlexExtTearClothMesh sets numParticles; the
// demo's additional "numParticles += numParticleCopies" after the call counts them twice and is not part of
// the operation).
NVRHI_IID(IPhysTearingCloth, "5208f1d0-2cff-479f-9b95-68d0cc018399")
struct IPhysTearingCloth : IPhysAsset {
    NVRHI_DECLARE_UUID_TRAITS(IPhysTearingCloth)

    // Writes particles first .. first + count - 1 of the asset's copy (4 floats each, x y z 1/mass): the copy
    // tearing measures strain on. FleX's application wrote NvFlexExtAsset::particles directly.
    // FE_INVALID_ARGS for a null pointer with a count, or a range beyond getMaxParticles().
    virtual nvrhi::FRESULT setParticles(const float* particles, uint32_t first, uint32_t count) noexcept = 0;

    // NvFlexExtTearClothMesh: every spring longer than maxStrain times its rest length splits the mesh at one
    // of its two particles, at most maxSplits splits; a fixed particle (1/mass 0) is never split. A split, and
    // the separation of the vertices it leaves singular, add particles: each is a clone, written to
    // particleCopies (at most maxCopies, and never more than getMaxParticles() - getParticleCount()), and
    // changes triangle indices, written to triangleEdits (at most maxEdits; edits beyond that are applied to
    // the topology and the springs but neither reported nor applied to getTriangleIndices(), as in FleX).
    //
    // Which of the two particles of a spring splits is chosen by the random generator of the builder that
    // created the cloth (IPhysAssetBuilder): FleX's Randf() of the extension module, which createSoftFromMesh
    // seeds. Until a soft body is built the generator stays at zero and the second particle always splits.
    //
    // A clone is not written to the asset's particles (the application copies it, step 3 above). Within one call,
    // a later split that measures a spring or a triangle at a particle cloned earlier in the same call reads what
    // the asset's copy holds at that index: zero, unless setParticles wrote beyond getParticleCount(). FleX read
    // uninitialised memory there; with maxSplits 1 the case does not arise.
    //
    // numParticleCopies and numTriangleEdits receive the counts. particleCopies and triangleEdits may be null
    // when their capacity is 0. FE_INVALID_ARGS for a null array with a capacity.
    virtual nvrhi::FRESULT tearClothMesh(float maxStrain, uint32_t maxSplits,
                                         _Out_opt_ TearingParticleClone* particleCopies, uint32_t maxCopies,
                                         uint32_t& numParticleCopies, _Out_opt_ TearingMeshEdit* triangleEdits,
                                         uint32_t maxEdits, uint32_t& numTriangleEdits) noexcept = 0;
};
typedef nvrhi::AutoPtr<IPhysTearingCloth> PhysTearingClothHandle;

}  // namespace ephys

#endif /* ETHEREAL_PHYS_EXT_ASSET_H */
