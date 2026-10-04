#include <ethereal/phys_utils/pfm.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

// A port of reference/flex/core/pfm.cpp over std::ifstream / std::ofstream.

namespace ephys::utils {

bool PfmLoad(const std::filesystem::path& filename, PfmImage& image) {
    image = PfmImage();

    std::ifstream f(filename, std::ios_base::in | std::ios_base::binary);
    if (!f)
        return false;

    std::string line;
    if (!std::getline(f, line) || line != "PF")
        return false;

    if (!std::getline(f, line))
        return false;
    unsigned width = 0, height = 0, depth = 1;
    const int dimensions = std::sscanf(line.c_str(), "%u %u %u", &width, &height, &depth);
    if (dimensions < 2)
        return false;
    if (dimensions == 2)
        depth = 1;

    if (!std::getline(f, line) || std::sscanf(line.c_str(), "%f", &image.m_maxDepth) != 1)
        return false;

    const std::streamoff dataStart = f.tellg();
    f.seekg(0, std::ios_base::end);
    const std::streamoff dataEnd = f.tellg();
    f.seekg(dataStart, std::ios_base::beg);

    const uint64_t count = uint64_t(width) * height * depth;
    if (count == 0 || uint64_t(dataEnd - dataStart) != count * sizeof(float))
        return false;

    image.m_width = width;
    image.m_height = height;
    image.m_depth = depth;
    image.m_data.resize(size_t(count));
    if (!f.read(reinterpret_cast<char*>(image.m_data.data()), std::streamsize(count * sizeof(float))))
        return false;

    return true;
}

bool PfmSave(const std::filesystem::path& filename, const PfmImage& image) {
    const uint64_t count = uint64_t(image.m_width) * image.m_height * image.m_depth;
    if (count == 0 || image.m_data.size() != count)
        return false;

    std::ofstream f(filename, std::ios_base::out | std::ios_base::binary);
    if (!f)
        return false;

    char header[128];
    if (image.m_depth > 1)
        std::snprintf(header, sizeof(header), "PF\n%u %u %u\n", image.m_width, image.m_height, image.m_depth);
    else
        std::snprintf(header, sizeof(header), "PF\n%u %u\n", image.m_width, image.m_height);
    f << header;

    std::snprintf(header, sizeof(header), "%f\n", *std::max_element(image.m_data.begin(), image.m_data.end()));
    f << header;

    f.write(reinterpret_cast<const char*>(image.m_data.data()), std::streamsize(count * sizeof(float)));
    return bool(f);
}

}  // namespace ephys::utils
