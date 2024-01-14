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

    TGAPointCloudAcceleration(tga::Interface &tgai, std::string_view scenePath);

    PointBuffers GetPointsBufferPack() const;
    tga::Buffer GetAccelerationStructureBuffer() const;
    tga::Buffer GetBatchesBuffer() const;

    std::uint32_t GetBatchCount() const;

    ~TGAPointCloudAcceleration();

private:
    struct BatchesCompressed {
        std::vector<CompressedPosition> lowPrecisions;
        std::vector<CompressedPosition> mediumPrecisions;
        std::vector<CompressedPosition> highPrecisions;
        std::vector<CompressedColor> colors;
    };

    tga::Interface &backend_;

    std::uint32_t batchCount_;

    PointBuffers pointsBufferPack_;
    tga::Buffer accelerationBuffer_;
    tga::Buffer batchesBuffer_;

    static BatchesCompressed ConvertToAdaptivePrecision(const std::vector<Batch> &batches);
};