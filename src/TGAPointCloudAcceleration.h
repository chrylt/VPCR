#pragma once

#include <tga/tga.hpp>
#include <tga/tga_utils.hpp>
#include <tga/tga_vulkan/tga_vulkan.hpp>

class TGAPointCloudAcceleration final {
public:
    TGAPointCloudAcceleration(tga::Interface &tgai, std::string_view scenePath);

    tga::Buffer GetPointsBuffer() const;
    tga::Buffer GetAccelerationStructureBuffer() const;
    tga::Buffer GetBatchesBuffer() const;

    std::uint32_t GetBatchCount() const;

    ~TGAPointCloudAcceleration();

private:
    tga::Interface &backend_;

    std::uint32_t batchCount_;

    tga::Buffer pointsBuffer_;
    tga::Buffer accelerationBuffer_;
    tga::Buffer batchesBuffer_;
};