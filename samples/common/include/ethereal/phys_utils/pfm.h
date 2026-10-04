#ifndef ETHEREAL_PHYS_UTILS_PFM_H
#define ETHEREAL_PHYS_UTILS_PFM_H

// The PFM image format, which FleX's distance field assets use (assets/*.pfm): FleX's core/pfm.h. Part of
// ethereal_phys_utils (samples/common), not of the engine.
//
// A PFM file is the text header "PF\n", "<width> <height>\n" (FleX's extension: "<width> <height> <depth>\n"
// for a volume), "<scale>\n" (FleX: the maximum value of the data), then width * height * depth floats, x
// fastest. A distance field volume goes to ISdfAsset::update (collision.h) as it is.

#include <cstdint>
#include <filesystem>
#include <vector>

namespace ephys::utils {

struct PfmImage {
    uint32_t m_width = 0;
    uint32_t m_height = 0;
    uint32_t m_depth = 1;  // 1 for a standard PFM image, more for a volume

    float m_maxDepth = 0.0f;  // the scale line of the header

    std::vector<float> m_data;  // width * height * depth
};

// False when the file cannot be read, has no PF header, or holds another amount of data than the header says.
bool PfmLoad(const std::filesystem::path& filename, PfmImage& image);

// Writes the image, with its maximum value as the scale (FleX). False when the file cannot be written or the
// data does not match the dimensions.
bool PfmSave(const std::filesystem::path& filename, const PfmImage& image);

}  // namespace ephys::utils

#endif /* ETHEREAL_PHYS_UTILS_PFM_H */
