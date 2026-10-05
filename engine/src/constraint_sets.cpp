#include "data_model.h"

#include <nvrhi/core/foundation.h>

#include <new>
#include <vector>

// The four constraint sets. The data half of FleX's Solver::SetSprings / GetSprings, SetRigids / GetRigids,
// SetDynamicTriangles / GetDynamicTriangles and SetInflatables (FleX/src/Solver.cpp). What those
// functions derive (the half-springs, the pressures, the multipliers) is the solver's and is not here.
//
// Every upload works in three stages, so that a refused call changes nothing: check the arguments, replace
// the buffers that are too small (into temporaries first), then record the copies.

namespace ephys {
namespace {

using detail::c_MaxArrayCount;
using detail::DataObject;
using detail::slack;

// One array of a set and what a transfer needs to know about it.
struct ArrayRef {
    const char* name;     // for messages
    IPhysBuffer* array;   // the array of the set
    IPhysBuffer* buffer;  // the buffer of the caller; null: skipped (the caller checked it may be)
    uint32_t stride;
    uint32_t count;
};

// ---- Springs ---------------------------------------------------------------------------------------------

NVRHI_CLASS_CLSID(SpringSet, "f3a9345d-65ef-45ce-bad9-fb9e3cffc48c")
class SpringSet final : public nvrhi::ObjectImpl<ISpringSet>, private DataObject {
 public:
    NVRHI_DECLARE_UUID_TRAITS(SpringSet)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(SpringSet)
    NVRHI_IMPLEMENTS_INTERFACE(ISpringSet)
    NVRHI_IMPLEMENTS_CLASS(SpringSet)
    NVRHI_END_INTERFACE_TABLE()

    explicit SpringSet(IPhysDevice* device)
        : DataObject(device) {}

    nvrhi::FRESULT reserve(uint32_t capacity) {
        if (capacity <= m_Max)
            return nvrhi::FS_OK;
        PhysBufferHandle indices, lengths, stiffness;
        nvrhi::FRESULT result = createArray("ephys spring indices", 4, 2 * capacity, indices);
        if (NVRHI_SUCCEEDED(result))
            result = createArray("ephys spring rest lengths", 4, capacity, lengths);
        if (NVRHI_SUCCEEDED(result))
            result = createArray("ephys spring stiffness", 4, capacity, stiffness);
        if (NVRHI_FAILED(result))
            return result;
        m_Indices = indices;
        m_Lengths = lengths;
        m_Stiffness = stiffness;
        m_Max = capacity;
        ++m_LayoutRevision;
        return nvrhi::FS_OK;
    }

    uint32_t getMaxSprings() const noexcept override { return m_Max; }
    uint32_t getNumSprings() const noexcept override { return m_Num; }
    IPhysBuffer* getIndices() const noexcept override { return m_Indices; }
    IPhysBuffer* getRestLengths() const noexcept override { return m_Lengths; }
    IPhysBuffer* getStiffness() const noexcept override { return m_Stiffness; }

    nvrhi::FRESULT upload(nvrhi::ICommandList* commandList, IPhysBuffer* indices, IPhysBuffer* restLengths,
                          IPhysBuffer* stiffness, uint32_t numSprings) noexcept override {
        if (numSprings == 0) {
            m_Num = 0;
            ++m_ContentRevision;
            return nvrhi::FS_OK;
        }
        if (numSprings > c_MaxArrayCount)
            return fail(nvrhi::FE_INVALID_ARGS, "ISpringSet::upload: %u springs are too many", numSprings);
        if (const char* problem = problemOf(indices, 4, 0, 2ull * numSprings, true))
            return fail(nvrhi::FE_INVALID_ARGS, "ISpringSet::upload: indices: %s", problem);
        if (const char* problem = problemOf(restLengths, 4, 0, numSprings, true))
            return fail(nvrhi::FE_INVALID_ARGS, "ISpringSet::upload: restLengths: %s", problem);
        if (const char* problem = problemOf(stiffness, 4, 0, numSprings, true))
            return fail(nvrhi::FE_INVALID_ARGS, "ISpringSet::upload: stiffness: %s", problem);

        nvrhi::FRESULT result = numSprings > m_Max ? reserve(slack(numSprings)) : nvrhi::FS_OK;
        if (NVRHI_FAILED(result))
            return fail(result, "ISpringSet::upload: cannot create the buffers for %u springs", numSprings);

        result = copy(commandList, m_Indices, 0, indices, 0, 2 * numSprings);
        if (NVRHI_SUCCEEDED(result))
            result = copy(commandList, m_Lengths, 0, restLengths, 0, numSprings);
        if (NVRHI_SUCCEEDED(result))
            result = copy(commandList, m_Stiffness, 0, stiffness, 0, numSprings);
        if (NVRHI_FAILED(result))
            return result;

        m_Num = numSprings;
        ++m_ContentRevision;
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT download(nvrhi::ICommandList* commandList, IPhysBuffer* indices, IPhysBuffer* restLengths,
                            IPhysBuffer* stiffness, uint32_t numSprings) noexcept override {
        if (numSprings > m_Num)
            return fail(nvrhi::FE_INVALID_ARGS, "ISpringSet::download: %u springs; the set holds %u",
                        numSprings, m_Num);
        if (indices)
            if (const char* problem = problemOf(indices, 4, 0, 2ull * numSprings, false))
                return fail(nvrhi::FE_INVALID_ARGS, "ISpringSet::download: indices: %s", problem);
        if (restLengths)
            if (const char* problem = problemOf(restLengths, 4, 0, numSprings, false))
                return fail(nvrhi::FE_INVALID_ARGS, "ISpringSet::download: restLengths: %s", problem);
        if (stiffness)
            if (const char* problem = problemOf(stiffness, 4, 0, numSprings, false))
                return fail(nvrhi::FE_INVALID_ARGS, "ISpringSet::download: stiffness: %s", problem);

        nvrhi::FRESULT result = nvrhi::FS_OK;
        if (indices)
            result = copy(commandList, indices, 0, m_Indices, 0, 2 * numSprings);
        if (NVRHI_SUCCEEDED(result) && restLengths)
            result = copy(commandList, restLengths, 0, m_Lengths, 0, numSprings);
        if (NVRHI_SUCCEEDED(result) && stiffness)
            result = copy(commandList, stiffness, 0, m_Stiffness, 0, numSprings);
        return result;
    }

    void resetRevisions() { m_LayoutRevision = 0; }

    uint64_t getLayoutRevision() const noexcept override { return m_LayoutRevision; }
    uint64_t getContentRevision() const noexcept override { return m_ContentRevision; }
    uint64_t markContentChanged() noexcept override { return ++m_ContentRevision; }

 private:
    PhysBufferHandle m_Indices;
    PhysBufferHandle m_Lengths;
    PhysBufferHandle m_Stiffness;
    uint32_t m_Max = 0;
    uint32_t m_Num = 0;
    uint64_t m_LayoutRevision = 0;
    uint64_t m_ContentRevision = 0;
};

// ---- Shape matching ----------------------------------------------------------------------------------------

NVRHI_CLASS_CLSID(ShapeMatchSet, "32a8f99b-f52a-4639-a9bc-cc655abb6061")
class ShapeMatchSet final : public nvrhi::ObjectImpl<IShapeMatchSet>, private DataObject {
 public:
    NVRHI_DECLARE_UUID_TRAITS(ShapeMatchSet)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(ShapeMatchSet)
    NVRHI_IMPLEMENTS_INTERFACE(IShapeMatchSet)
    NVRHI_IMPLEMENTS_CLASS(ShapeMatchSet)
    NVRHI_END_INTERFACE_TABLE()

    explicit ShapeMatchSet(IPhysDevice* device)
        : DataObject(device) {}

    // The arrays with one element per rigid (the offsets have one more).
    nvrhi::FRESULT reserveRigids(uint32_t capacity) {
        if (capacity <= m_MaxRigids)
            return nvrhi::FS_OK;
        PhysBufferHandle offsets, stiffness, rotations, translations, thresholds, creeps;
        nvrhi::FRESULT result = createArray("ephys rigid offsets", 4, capacity + 1, offsets);
        if (NVRHI_SUCCEEDED(result))
            result = createArray("ephys rigid stiffness", 4, capacity, stiffness);
        if (NVRHI_SUCCEEDED(result))
            result = createArray("ephys rigid rotations", 16, capacity, rotations);
        if (NVRHI_SUCCEEDED(result))
            result = createArray("ephys rigid translations", 12, capacity, translations);
        // The plastic arrays follow the capacity while the set has them.
        if (NVRHI_SUCCEEDED(result) && m_Plastic)
            result = createPlastic(capacity, thresholds, creeps);
        if (NVRHI_FAILED(result))
            return result;
        m_Offsets = offsets;
        m_Stiffness = stiffness;
        m_Rotations = rotations;
        m_Translations = translations;
        if (m_Plastic) {
            m_Thresholds = thresholds;
            m_Creeps = creeps;
        }
        m_MaxRigids = capacity;
        ++m_LayoutRevision;
        return nvrhi::FS_OK;
    }

    // The arrays with one element per particle index.
    nvrhi::FRESULT reserveIndices(uint32_t capacity) {
        if (capacity <= m_MaxIndices)
            return nvrhi::FS_OK;
        PhysBufferHandle indices, positions, normals;
        nvrhi::FRESULT result = createArray("ephys rigid indices", 4, capacity, indices);
        if (NVRHI_SUCCEEDED(result))
            result = createArray("ephys rigid rest positions", 12, capacity, positions);
        if (NVRHI_SUCCEEDED(result))
            result = createArray("ephys rigid rest normals", 16, capacity, normals);
        if (NVRHI_FAILED(result))
            return result;
        m_Indices = indices;
        m_RestPositions = positions;
        m_RestNormals = normals;
        m_MaxIndices = capacity;
        ++m_LayoutRevision;
        return nvrhi::FS_OK;
    }

    uint32_t getMaxRigids() const noexcept override { return m_MaxRigids; }
    uint32_t getNumRigids() const noexcept override { return m_NumRigids; }
    uint32_t getMaxIndices() const noexcept override { return m_MaxIndices; }
    uint32_t getNumIndices() const noexcept override { return m_NumIndices; }
    bool hasPlasticDeformation() const noexcept override { return m_Plastic; }

    IPhysBuffer* getOffsets() const noexcept override { return m_Offsets; }
    IPhysBuffer* getIndices() const noexcept override { return m_Indices; }
    IPhysBuffer* getRestPositions() const noexcept override { return m_RestPositions; }
    IPhysBuffer* getRestNormals() const noexcept override { return m_RestNormals; }
    IPhysBuffer* getStiffness() const noexcept override { return m_Stiffness; }
    IPhysBuffer* getPlasticThresholds() const noexcept override { return m_Thresholds; }
    IPhysBuffer* getPlasticCreeps() const noexcept override { return m_Creeps; }
    IPhysBuffer* getRotations() const noexcept override { return m_Rotations; }
    IPhysBuffer* getTranslations() const noexcept override { return m_Translations; }

    nvrhi::FRESULT upload(nvrhi::ICommandList* commandList, const ShapeMatchBuffers& b, uint32_t numRigids,
                          uint32_t numIndices) noexcept override {
        if (numRigids == 0) {
            m_NumRigids = 0;
            m_NumIndices = 0;
            ++m_ContentRevision;
            return nvrhi::FS_OK;
        }
        if (numRigids > c_MaxArrayCount || numIndices > c_MaxArrayCount)
            return fail(nvrhi::FE_INVALID_ARGS,
                        "IShapeMatchSet::upload: %u rigids over %u indices are too many", numRigids,
                        numIndices);
        if (numIndices == 0)
            return fail(nvrhi::FE_INVALID_ARGS, "IShapeMatchSet::upload: %u rigids without particle indices",
                        numRigids);
        if ((b.plasticThresholds != nullptr) != (b.plasticCreeps != nullptr))
            return fail(
                nvrhi::FE_INVALID_ARGS,
                "IShapeMatchSet::upload: plasticThresholds and plasticCreeps are given together or not at all");
        const bool plastic = b.plasticThresholds != nullptr;

        // The sources. The arrays are filled in below, once the set has room.
        const ArrayRef sources[] = {
            {"offsets",           nullptr, b.offsets,           4,  numRigids + 1},
            {"indices",           nullptr, b.indices,           4,  numIndices   },
            {"restPositions",     nullptr, b.restPositions,     12, numIndices   },
            {"stiffness",         nullptr, b.stiffness,         4,  numRigids    },
            {"rotations",         nullptr, b.rotations,         16, numRigids    },
            {"translations",      nullptr, b.translations,      12, numRigids    },
            {"restNormals",       nullptr, b.restNormals,       16, numIndices   }, // optional
            {"plasticThresholds", nullptr, b.plasticThresholds, 4,  numRigids    }, // optional
            {"plasticCreeps",     nullptr, b.plasticCreeps,     4,  numRigids    }, // optional
        };
        for (size_t i = 0; i < sizeof(sources) / sizeof(sources[0]); ++i) {
            const ArrayRef& s = sources[i];
            if (!s.buffer && i >= 6)
                continue;
            if (const char* problem = problemOf(s.buffer, s.stride, 0, s.count, true))
                return fail(nvrhi::FE_INVALID_ARGS, "IShapeMatchSet::upload: %s: %s", s.name, problem);
        }

        nvrhi::FRESULT result = numRigids > m_MaxRigids ? reserveRigids(slack(numRigids)) : nvrhi::FS_OK;
        if (NVRHI_SUCCEEDED(result) && numIndices > m_MaxIndices)
            result = reserveIndices(slack(numIndices));
        if (NVRHI_SUCCEEDED(result) && plastic && !m_Plastic) {
            result = createPlastic(m_MaxRigids, m_Thresholds, m_Creeps);
            if (NVRHI_SUCCEEDED(result)) {
                m_Plastic = true;
                ++m_LayoutRevision;
            }
        }
        if (NVRHI_FAILED(result))
            return fail(result,
                        "IShapeMatchSet::upload: cannot create the buffers for %u rigids over %u indices",
                        numRigids, numIndices);
        if (!plastic && m_Plastic) {
            // As in FleX: an upload without plastic coefficients drops them.
            m_Thresholds = nullptr;
            m_Creeps = nullptr;
            m_Plastic = false;
            ++m_LayoutRevision;
        }

        result = copy(commandList, m_Rotations, 0, b.rotations, 0, numRigids);
        if (NVRHI_SUCCEEDED(result))
            result = copy(commandList, m_Translations, 0, b.translations, 0, numRigids);
        if (NVRHI_SUCCEEDED(result))
            result = copy(commandList, m_Offsets, 0, b.offsets, 0, numRigids + 1);
        if (NVRHI_SUCCEEDED(result))
            result = copy(commandList, m_Indices, 0, b.indices, 0, numIndices);
        if (NVRHI_SUCCEEDED(result))
            result = copy(commandList, m_Stiffness, 0, b.stiffness, 0, numRigids);
        if (NVRHI_SUCCEEDED(result))
            result = copy(commandList, m_RestPositions, 0, b.restPositions, 0, numIndices);
        if (NVRHI_SUCCEEDED(result)) {
            if (b.restNormals) {
                result = copy(commandList, m_RestNormals, 0, b.restNormals, 0, numIndices);
            } else {
                // FleX clears the normals with a kernel; a set dispatches none and writes zeros instead.
                try {
                    const std::vector<uint8_t> zeros(size_t(numIndices) * 16, 0);
                    result = m_Device->writeBuffer(commandList, m_RestNormals, zeros.data(), 0, numIndices);
                } catch (const std::bad_alloc&) {
                    result = nvrhi::FE_OUT_OF_MEMORY;
                }
            }
        }
        if (NVRHI_SUCCEEDED(result) && plastic) {
            result = copy(commandList, m_Thresholds, 0, b.plasticThresholds, 0, numRigids);
            if (NVRHI_SUCCEEDED(result))
                result = copy(commandList, m_Creeps, 0, b.plasticCreeps, 0, numRigids);
        }
        if (NVRHI_FAILED(result))
            return result;

        m_NumRigids = numRigids;
        m_NumIndices = numIndices;
        ++m_ContentRevision;
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT download(nvrhi::ICommandList* commandList, const ShapeMatchBuffers& b) noexcept override {
        if (!m_Plastic && (b.plasticThresholds || b.plasticCreeps))
            return fail(nvrhi::FE_INVALID_ARGS, "IShapeMatchSet::download: the set has no plastic deformation");
        if (m_NumRigids == 0)
            return nvrhi::FS_OK;

        const ArrayRef targets[] = {
            {"offsets",           m_Offsets,       b.offsets,           4,  m_NumRigids + 1},
            {"indices",           m_Indices,       b.indices,           4,  m_NumIndices   },
            {"restPositions",     m_RestPositions, b.restPositions,     12, m_NumIndices   },
            {"restNormals",       m_RestNormals,   b.restNormals,       16, m_NumIndices   },
            {"stiffness",         m_Stiffness,     b.stiffness,         4,  m_NumRigids    },
            {"plasticThresholds", m_Thresholds,    b.plasticThresholds, 4,  m_NumRigids    },
            {"plasticCreeps",     m_Creeps,        b.plasticCreeps,     4,  m_NumRigids    },
            {"rotations",         m_Rotations,     b.rotations,         16, m_NumRigids    },
            {"translations",      m_Translations,  b.translations,      12, m_NumRigids    },
        };
        for (const ArrayRef& t : targets) {
            if (!t.buffer)
                continue;
            if (const char* problem = problemOf(t.buffer, t.stride, 0, t.count, false))
                return fail(nvrhi::FE_INVALID_ARGS, "IShapeMatchSet::download: %s: %s", t.name, problem);
        }
        for (const ArrayRef& t : targets) {
            if (!t.buffer)
                continue;
            const nvrhi::FRESULT result = copy(commandList, t.buffer, 0, t.array, 0, t.count);
            if (NVRHI_FAILED(result))
                return result;
        }
        return nvrhi::FS_OK;
    }

    void resetRevisions() { m_LayoutRevision = 0; }

    uint64_t getLayoutRevision() const noexcept override { return m_LayoutRevision; }
    uint64_t getContentRevision() const noexcept override { return m_ContentRevision; }
    uint64_t markContentChanged() noexcept override { return ++m_ContentRevision; }

 private:
    nvrhi::FRESULT createPlastic(uint32_t capacity, PhysBufferHandle& thresholds,
                                 PhysBufferHandle& creeps) const {
        PhysBufferHandle t, c;
        nvrhi::FRESULT result = createArray("ephys rigid plastic thresholds", 4, capacity, t);
        if (NVRHI_SUCCEEDED(result))
            result = createArray("ephys rigid plastic creeps", 4, capacity, c);
        if (NVRHI_FAILED(result))
            return result;
        thresholds = t;
        creeps = c;
        return nvrhi::FS_OK;
    }

    PhysBufferHandle m_Offsets;
    PhysBufferHandle m_Indices;
    PhysBufferHandle m_RestPositions;
    PhysBufferHandle m_RestNormals;
    PhysBufferHandle m_Stiffness;
    PhysBufferHandle m_Thresholds;
    PhysBufferHandle m_Creeps;
    PhysBufferHandle m_Rotations;
    PhysBufferHandle m_Translations;
    uint32_t m_MaxRigids = 0;
    uint32_t m_NumRigids = 0;
    uint32_t m_MaxIndices = 0;
    uint32_t m_NumIndices = 0;
    bool m_Plastic = false;
    uint64_t m_LayoutRevision = 0;
    uint64_t m_ContentRevision = 0;
};

// ---- Dynamic triangles -------------------------------------------------------------------------------------

NVRHI_CLASS_CLSID(TriangleSet, "3dd9dca5-d785-4aba-9bcd-97c207260280")
class TriangleSet final : public nvrhi::ObjectImpl<ITriangleSet>, private DataObject {
 public:
    NVRHI_DECLARE_UUID_TRAITS(TriangleSet)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(TriangleSet)
    NVRHI_IMPLEMENTS_INTERFACE(ITriangleSet)
    NVRHI_IMPLEMENTS_CLASS(TriangleSet)
    NVRHI_END_INTERFACE_TABLE()

    explicit TriangleSet(IPhysDevice* device)
        : DataObject(device) {}

    nvrhi::FRESULT reserve(uint32_t capacity) {
        if (capacity <= m_Max)
            return nvrhi::FS_OK;
        PhysBufferHandle indices, normals;
        nvrhi::FRESULT result = createArray("ephys triangle indices", 4, 3 * capacity, indices);
        if (NVRHI_SUCCEEDED(result))
            result = createArray("ephys triangle normals", 12, capacity, normals);
        if (NVRHI_FAILED(result))
            return result;
        m_Indices = indices;
        m_Normals = normals;
        m_Max = capacity;
        ++m_LayoutRevision;
        return nvrhi::FS_OK;
    }

    uint32_t getMaxTriangles() const noexcept override { return m_Max; }
    uint32_t getNumTriangles() const noexcept override { return m_Num; }
    IPhysBuffer* getIndices() const noexcept override { return m_Indices; }
    IPhysBuffer* getNormals() const noexcept override { return m_Normals; }

    nvrhi::FRESULT upload(nvrhi::ICommandList* commandList, IPhysBuffer* indices, IPhysBuffer* normals,
                          uint32_t numTriangles) noexcept override {
        if (numTriangles == 0) {
            m_Num = 0;
            ++m_ContentRevision;
            return nvrhi::FS_OK;
        }
        if (numTriangles > c_MaxArrayCount)
            return fail(nvrhi::FE_INVALID_ARGS, "ITriangleSet::upload: %u triangles are too many",
                        numTriangles);
        if (const char* problem = problemOf(indices, 4, 0, 3ull * numTriangles, true))
            return fail(nvrhi::FE_INVALID_ARGS, "ITriangleSet::upload: indices: %s", problem);
        if (normals)
            if (const char* problem = problemOf(normals, 12, 0, numTriangles, true))
                return fail(nvrhi::FE_INVALID_ARGS, "ITriangleSet::upload: normals: %s", problem);

        nvrhi::FRESULT result = numTriangles > m_Max ? reserve(slack(numTriangles)) : nvrhi::FS_OK;
        if (NVRHI_FAILED(result))
            return fail(result, "ITriangleSet::upload: cannot create the buffers for %u triangles",
                        numTriangles);

        result = copy(commandList, m_Indices, 0, indices, 0, 3 * numTriangles);
        if (NVRHI_SUCCEEDED(result) && normals)
            result = copy(commandList, m_Normals, 0, normals, 0, numTriangles);
        if (NVRHI_FAILED(result))
            return result;

        m_Num = numTriangles;
        ++m_ContentRevision;
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT download(nvrhi::ICommandList* commandList, IPhysBuffer* indices, IPhysBuffer* normals,
                            uint32_t numTriangles) noexcept override {
        if (numTriangles > m_Num)
            return fail(nvrhi::FE_INVALID_ARGS, "ITriangleSet::download: %u triangles; the set holds %u",
                        numTriangles, m_Num);
        if (indices)
            if (const char* problem = problemOf(indices, 4, 0, 3ull * numTriangles, false))
                return fail(nvrhi::FE_INVALID_ARGS, "ITriangleSet::download: indices: %s", problem);
        if (normals)
            if (const char* problem = problemOf(normals, 12, 0, numTriangles, false))
                return fail(nvrhi::FE_INVALID_ARGS, "ITriangleSet::download: normals: %s", problem);

        nvrhi::FRESULT result = nvrhi::FS_OK;
        if (indices)
            result = copy(commandList, indices, 0, m_Indices, 0, 3 * numTriangles);
        if (NVRHI_SUCCEEDED(result) && normals)
            result = copy(commandList, normals, 0, m_Normals, 0, numTriangles);
        return result;
    }

    void resetRevisions() { m_LayoutRevision = 0; }

    uint64_t getLayoutRevision() const noexcept override { return m_LayoutRevision; }
    uint64_t getContentRevision() const noexcept override { return m_ContentRevision; }
    uint64_t markContentChanged() noexcept override { return ++m_ContentRevision; }

 private:
    PhysBufferHandle m_Indices;
    PhysBufferHandle m_Normals;
    uint32_t m_Max = 0;
    uint32_t m_Num = 0;
    uint64_t m_LayoutRevision = 0;
    uint64_t m_ContentRevision = 0;
};

// ---- Inflatables -------------------------------------------------------------------------------------------

NVRHI_CLASS_CLSID(InflatableSet, "48fb4ca8-51ea-4552-aaf2-5d1aad14cc8e")
class InflatableSet final : public nvrhi::ObjectImpl<IInflatableSet>, private DataObject {
 public:
    NVRHI_DECLARE_UUID_TRAITS(InflatableSet)
    NVRHI_BEGIN_INTERFACE_TABLE_INLINE(InflatableSet)
    NVRHI_IMPLEMENTS_INTERFACE(IInflatableSet)
    NVRHI_IMPLEMENTS_CLASS(InflatableSet)
    NVRHI_END_INTERFACE_TABLE()

    explicit InflatableSet(IPhysDevice* device)
        : DataObject(device) {}

    nvrhi::FRESULT reserve(uint32_t capacity) {
        if (capacity <= m_Max)
            return nvrhi::FS_OK;
        PhysBufferHandle inflatables;
        const nvrhi::FRESULT result =
            createArray("ephys inflatables", sizeof(Inflatable), capacity, inflatables);
        if (NVRHI_FAILED(result))
            return result;
        m_Inflatables = inflatables;
        m_Max = capacity;
        ++m_LayoutRevision;
        return nvrhi::FS_OK;
    }

    uint32_t getMaxInflatables() const noexcept override { return m_Max; }
    uint32_t getNumInflatables() const noexcept override { return m_Num; }
    IPhysBuffer* getInflatables() const noexcept override { return m_Inflatables; }

    nvrhi::FRESULT upload(nvrhi::ICommandList* commandList, const InflatableDesc* inflatables,
                          uint32_t numInflatables) noexcept override {
        if (numInflatables == 0) {
            m_Num = 0;
            ++m_ContentRevision;
            return nvrhi::FS_OK;
        }
        if (!inflatables)
            return fail(nvrhi::FE_INVALID_ARGS, "IInflatableSet::upload: inflatables is null");
        if (numInflatables > c_MaxArrayCount)
            return fail(nvrhi::FE_INVALID_ARGS, "IInflatableSet::upload: %u inflatables are too many",
                        numInflatables);

        try {
            // FleX's Solver::SetInflatables: the device form has the over-pressure folded into the volume.
            std::vector<Inflatable> packed(numInflatables);
            for (uint32_t i = 0; i < numInflatables; ++i) {
                packed[i].startTri = inflatables[i].startTri;
                packed[i].numTris = inflatables[i].numTris;
                packed[i].restVolume = inflatables[i].restVolume * inflatables[i].overPressure;
                packed[i].constraintScale = inflatables[i].constraintScale;
            }

            nvrhi::FRESULT result = numInflatables > m_Max ? reserve(slack(numInflatables)) : nvrhi::FS_OK;
            if (NVRHI_FAILED(result))
                return fail(result, "IInflatableSet::upload: cannot create the buffer for %u inflatables",
                            numInflatables);

            result = m_Device->writeBuffer(commandList, m_Inflatables, packed.data(), 0, numInflatables);
            if (NVRHI_FAILED(result))
                return result;
        } catch (const std::bad_alloc&) {
            return fail(nvrhi::FE_OUT_OF_MEMORY, "IInflatableSet::upload: out of memory");
        }

        m_Num = numInflatables;
        ++m_ContentRevision;
        return nvrhi::FS_OK;
    }

    nvrhi::FRESULT download(nvrhi::ICommandList* commandList, IPhysBuffer* inflatables,
                            uint32_t numInflatables) noexcept override {
        if (numInflatables > m_Num)
            return fail(nvrhi::FE_INVALID_ARGS, "IInflatableSet::download: %u inflatables; the set holds %u",
                        numInflatables, m_Num);
        if (const char* problem = problemOf(inflatables, sizeof(Inflatable), 0, numInflatables, false))
            return fail(nvrhi::FE_INVALID_ARGS, "IInflatableSet::download: inflatables: %s", problem);
        return copy(commandList, inflatables, 0, m_Inflatables, 0, numInflatables);
    }

    void resetRevisions() { m_LayoutRevision = 0; }

    uint64_t getLayoutRevision() const noexcept override { return m_LayoutRevision; }
    uint64_t getContentRevision() const noexcept override { return m_ContentRevision; }
    uint64_t markContentChanged() noexcept override { return ++m_ContentRevision; }

 private:
    PhysBufferHandle m_Inflatables;
    uint32_t m_Max = 0;
    uint32_t m_Num = 0;
    uint64_t m_LayoutRevision = 0;
    uint64_t m_ContentRevision = 0;
};

// MAKE_RC_OBJ, then the reserve the domain asked for. A reserve is not counted as a layout change: the
// revisions of a new set start at 0.
template <typename Class, typename Interface, typename Reserve>
nvrhi::FRESULT make(IPhysDevice* device, Interface** ppSet, const char* what, Reserve&& reserve) noexcept {
    if (!ppSet)
        return nvrhi::FE_INVALID_ARGS;
    *ppSet = nullptr;
    try {
        nvrhi::AutoPtr<Class> set = nvrhi::TakeOver(MAKE_RC_OBJ(Class, device));
        const nvrhi::FRESULT result = reserve(*set);
        if (NVRHI_FAILED(result)) {
            detail::report(device->getDesc().messageCallback, PhysMessageSeverity::Error,
                           "createDomain: cannot create the %s", what);
            return result;
        }
        *ppSet = set.Detach();
        return nvrhi::FS_OK;
    } catch (const std::bad_alloc&) {
        return nvrhi::FE_OUT_OF_MEMORY;
    } catch (...) {
        return nvrhi::FE_GENERIC_ERROR;
    }
}

}  // namespace

namespace detail {

nvrhi::FRESULT createSpringSet(IPhysDevice* device, uint32_t reserve, ISpringSet** ppSet) noexcept {
    return make<SpringSet, ISpringSet>(device, ppSet, "spring set", [reserve](SpringSet& set) {
        const nvrhi::FRESULT result = set.reserve(reserve);
        set.resetRevisions();
        return result;
    });
}

nvrhi::FRESULT createShapeMatchSet(IPhysDevice* device, uint32_t reserveRigids, uint32_t reserveIndices,
                                   IShapeMatchSet** ppSet) noexcept {
    return make<ShapeMatchSet, IShapeMatchSet>(device, ppSet, "shape match set",
                                               [reserveRigids, reserveIndices](ShapeMatchSet& set) {
                                                   nvrhi::FRESULT result = set.reserveRigids(reserveRigids);
                                                   if (NVRHI_SUCCEEDED(result))
                                                       result = set.reserveIndices(reserveIndices);
                                                   set.resetRevisions();
                                                   return result;
                                               });
}

nvrhi::FRESULT createTriangleSet(IPhysDevice* device, uint32_t reserve, ITriangleSet** ppSet) noexcept {
    return make<TriangleSet, ITriangleSet>(device, ppSet, "triangle set", [reserve](TriangleSet& set) {
        const nvrhi::FRESULT result = set.reserve(reserve);
        set.resetRevisions();
        return result;
    });
}

nvrhi::FRESULT createInflatableSet(IPhysDevice* device, uint32_t reserve, IInflatableSet** ppSet) noexcept {
    return make<InflatableSet, IInflatableSet>(device, ppSet, "inflatable set", [reserve](InflatableSet& set) {
        const nvrhi::FRESULT result = set.reserve(reserve);
        set.resetRevisions();
        return result;
    });
}

}  // namespace detail
}  // namespace ephys
