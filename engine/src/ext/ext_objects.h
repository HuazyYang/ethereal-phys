#ifndef ETHEREAL_PHYS_SRC_EXT_EXT_OBJECTS_H
#define ETHEREAL_PHYS_SRC_EXT_EXT_OBJECTS_H

// Module-private: how the device creates the container, the force fields and the moving frame of the extension
// layer (ext/container.h, ext/force_field.h, ext/moving_frame.h). The implementation classes stay in
// container.cpp, force_field_set.cpp and moving_frame.cpp. The device has checked that the solver is one of its
// own; each factory returns FS_OK and one reference, or an FE_* code, a message and nullptr.

#include <ethereal/phys/device.h>

namespace ephys::detail {

nvrhi::FRESULT createContainer(IPhysDevice* device, IPhysSolver* solver, uint32_t maxParticles,
                               IPhysContainer** ppContainer) noexcept;

nvrhi::FRESULT createForceFieldSet(IPhysDevice* device, IPhysSolver* solver, IForceFieldSet** ppSet) noexcept;

nvrhi::FRESULT createMovingFrame(IPhysDevice* device, const float* worldTranslation, const float* worldRotation,
                                 IMovingFrame** ppFrame) noexcept;

}  // namespace ephys::detail

#endif /* ETHEREAL_PHYS_SRC_EXT_EXT_OBJECTS_H */
