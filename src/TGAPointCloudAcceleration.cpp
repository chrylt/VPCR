#include "TGAPointCloudAcceleration.h"

#include "Utils.h"

TGAPointCloudAcceleration::TGAPointCloudAcceleration(tga::Interface& tgai, const std::string_view scenePath)
    : backend_(tgai)
{
    // TODO: @Atzubi

    const auto batches = LoadScene(scenePath);
    batchCount_ = static_cast<std::uint32_t>(batches.size());

    // Create buffers for points
    {
        std::vector<CompressedPosition> lowPrecisions;
        std::vector<CompressedPosition> mediumPrecisions;
        std::vector<CompressedPosition> highPrecisions;
        std::vector<CompressedColor> colors;

        for (const auto& batch : batches) {
            for (const auto& point : batch.points) {

                // Compress position and split into multiple buffers
                const glm::vec3 floatPos = point.position;
                const glm::vec3 aabbSize = batch.aabb.maxV - batch.aabb.minV;

                // convert float position to 30-bit fixed precision relative to BB
                const std::uint32_t x30 = std::min(uint32_t(std::floor((1 << 30) * (floatPos.x - batch.aabb.minV.x) / aabbSize.x)), uint32_t((1 << 30) - 1));
                const std::uint32_t y30 = std::min(uint32_t(std::floor((1 << 30) * (floatPos.y - batch.aabb.minV.y) / aabbSize.y)), uint32_t((1 << 30) - 1));
                const std::uint32_t z30 = std::min(uint32_t(std::floor((1 << 30) * (floatPos.z - batch.aabb.minV.z) / aabbSize.z)), uint32_t((1 << 30) - 1));

                lowPrecisions.emplace_back((x30 >> 20) & 0x3FF, (y30 >> 20) & 0x3FF, (z30 >> 20) & 0x3FF, 0);       // take upmost 10 bit of each coordinate as low precision
                mediumPrecisions.emplace_back((x30 >> 10) & 0x3FF, (y30 >> 10) & 0x3FF, (z30 >> 10) & 0x3FF, 0);    // take next lower 10 bit as medium precision
                highPrecisions.emplace_back(x30 & 0x3FF, y30 & 0x3FF, z30 & 0x3FF, 0);                              // take lowest 10 bit as high precision

                // Pass on colors
                colors.emplace_back(point.color);
            }
        }

        const size_t numberOfPoints = lowPrecisions.size();
        
        // Low Precision
        {
            const tga::StagingBufferInfo stagingInfo{numberOfPoints * sizeof(CompressedPosition),
                                                     reinterpret_cast<const std::uint8_t *>(lowPrecisions.data())};
            const tga::StagingBuffer staging = backend_.createStagingBuffer(stagingInfo);
            const tga::BufferInfo info{tga::BufferUsage::storage, numberOfPoints * sizeof(CompressedPosition), staging};
            pointsBufferPack_.positionLowPrecision = backend_.createBuffer(info);

            backend_.free(staging);
        }

        // Medium Precision
        {
            const tga::StagingBufferInfo stagingInfo{numberOfPoints * sizeof(CompressedPosition),
                                                     reinterpret_cast<const std::uint8_t *>(mediumPrecisions.data())};
            const tga::StagingBuffer staging = backend_.createStagingBuffer(stagingInfo);
            const tga::BufferInfo info{tga::BufferUsage::storage, numberOfPoints * sizeof(CompressedPosition), staging};
            pointsBufferPack_.positionMediumPrecision = backend_.createBuffer(info);

            backend_.free(staging);
        }

        // High Precision
        {
            const tga::StagingBufferInfo stagingInfo{numberOfPoints * sizeof(CompressedPosition),
                                                     reinterpret_cast<const std::uint8_t *>(highPrecisions.data())};
            const tga::StagingBuffer staging = backend_.createStagingBuffer(stagingInfo);
            const tga::BufferInfo info{tga::BufferUsage::storage, numberOfPoints * sizeof(CompressedPosition), staging};
            pointsBufferPack_.positionHighPrecision = backend_.createBuffer(info);

            backend_.free(staging);
        }

        // Colors
        {
            const tga::StagingBufferInfo stagingInfo{numberOfPoints * sizeof(CompressedColor),
                                                     reinterpret_cast<const std::uint8_t *>(colors.data())};
            const tga::StagingBuffer staging = backend_.createStagingBuffer(stagingInfo);
            const tga::BufferInfo info{tga::BufferUsage::storage, numberOfPoints * sizeof(CompressedColor), staging};
            pointsBufferPack_.colors = backend_.createBuffer(info);

            backend_.free(staging);
        }
    }

    // Create buffer for batches
    {
        struct GpuBatch {
            AABB box;
            std::uint32_t pointOffset;
            std::uint32_t pointCount;
        };

        std::vector<GpuBatch> gpuBatches;
        std::uint32_t offset = 0;
        for (const auto& batch : batches) {
            gpuBatches.emplace_back(batch.aabb, offset, static_cast<std::uint32_t>(batch.points.size()));
            offset += static_cast<std::uint32_t>(batch.points.size());
        }

        const tga::StagingBufferInfo stagingInfo{gpuBatches.size() * sizeof(GpuBatch),
                                                 reinterpret_cast<const std::uint8_t *>(gpuBatches.data())};
        const tga::StagingBuffer staging = backend_.createStagingBuffer(stagingInfo);
        const tga::BufferInfo info{tga::BufferUsage::storage, gpuBatches.size() * sizeof(GpuBatch), staging};
        batchesBuffer_ = backend_.createBuffer(info);

        backend_.free(staging);
    }

    // Create empty acceleration structure buffer for now
    {
        const tga::BufferInfo info{tga::BufferUsage::storage, 1};
        accelerationBuffer_ = backend_.createBuffer(info);
    }
}

TGAPointCloudAcceleration::PointBuffers TGAPointCloudAcceleration::GetPointsBufferPack() const
{
    return pointsBufferPack_;
}

tga::Buffer TGAPointCloudAcceleration::GetAccelerationStructureBuffer() const { return accelerationBuffer_; }

tga::Buffer TGAPointCloudAcceleration::GetBatchesBuffer() const { return batchesBuffer_; }

std::uint32_t TGAPointCloudAcceleration::GetBatchCount() const { return batchCount_; }

TGAPointCloudAcceleration::~TGAPointCloudAcceleration()
{
    backend_.free(pointsBufferPack_.positionLowPrecision);
    backend_.free(pointsBufferPack_.positionMediumPrecision);
    backend_.free(pointsBufferPack_.positionHighPrecision);
    backend_.free(pointsBufferPack_.colors);
    backend_.free(batchesBuffer_);
    backend_.free(accelerationBuffer_);
}