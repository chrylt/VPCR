#pragma once

#include <tga/tga.hpp>
#include <tga/tga_utils.hpp>
#include <tga/tga_vulkan/tga_vulkan.hpp>

#include "Utils.h"

class TGAPointCloudAcceleration final {
public:
    struct PointBuffers {
        tga::Buffer positionLowPrecision;
        tga::Buffer positionMediumPrecision;
        tga::Buffer positionHighPrecision;
        tga::Buffer colors;
    };

    TGAPointCloudAcceleration(tga::Interface& tgai, std::string_view scenePath);

    PointBuffers GetPointsBufferPack() const;
    tga::Buffer GetBatchesBuffer() const;

    std::uint32_t GetBatchCount() const;
    std::uint32_t GetMaxTreeDepth() const;

    ~TGAPointCloudAcceleration();

private:
    tga::Interface& backend_;

    std::uint32_t batchCount_;
    std::uint32_t maxTreeDepth_;

    PointBuffers pointsBufferPack_;
    tga::Buffer batchesBuffer_;
};