#include "TGAPointCloudAcceleration.h"

#include "Utils.h"

namespace
{
struct BatchesCompressed {
    std::vector<CompressedPosition> lowPrecisions;
    std::vector<CompressedPosition> mediumPrecisions;
    std::vector<CompressedPosition> highPrecisions;
    std::vector<CompressedColor> colors;
};

BatchesCompressed ConvertToAdaptivePrecision(const std::vector<Batch>& batches)
{
    std::vector<CompressedPosition> lowPrecisions;
    std::vector<CompressedPosition> mediumPrecisions;
    std::vector<CompressedPosition> highPrecisions;
    std::vector<CompressedColor> colors;

    lowPrecisions.reserve(batches.size() * MaxBatchSize);
    mediumPrecisions.reserve(batches.size() * MaxBatchSize);
    highPrecisions.reserve(batches.size() * MaxBatchSize);
    colors.reserve(batches.size() * MaxBatchSize);

    for (const auto& [aabb, points] : batches) {
        for (const auto& [position, color] : points) {
            // Compress position and split into multiple buffers
            const glm::vec3& floatPos = position;
            const glm::vec3 aabbSize = aabb.maxV - aabb.minV;

            // convert float position to 30-bit fixed precision relative to BB
            const auto x30 =
                std::min(static_cast<std::uint32_t>(std::floor((1 << 30) * (floatPos.x - aabb.minV.x) / aabbSize.x)),
                         static_cast<std::uint32_t>((1 << 30) - 1));
            const auto y30 =
                std::min(static_cast<std::uint32_t>(std::floor((1 << 30) * (floatPos.y - aabb.minV.y) / aabbSize.y)),
                         static_cast<std::uint32_t>((1 << 30) - 1));
            const auto z30 =
                std::min(static_cast<std::uint32_t>(std::floor((1 << 30) * (floatPos.z - aabb.minV.z) / aabbSize.z)),
                         static_cast<std::uint32_t>((1 << 30) - 1));

            lowPrecisions.emplace_back((x30 >> 20) & 0x3FF, (y30 >> 20) & 0x3FF, (z30 >> 20) & 0x3FF,
                                       0);  // take upmost 10 bit of each coordinate as low precision
            mediumPrecisions.emplace_back((x30 >> 10) & 0x3FF, (y30 >> 10) & 0x3FF, (z30 >> 10) & 0x3FF,
                                          0);  // take next lower 10 bit as medium precision
            highPrecisions.emplace_back(x30 & 0x3FF, y30 & 0x3FF, z30 & 0x3FF,
                                        0);  // take lowest 10 bit as high precision

            // Pass on colors
            colors.emplace_back(color);
        }
    }

    return {lowPrecisions, mediumPrecisions, highPrecisions, colors};
}
}  // namespace

TGAPointCloudAcceleration::TGAPointCloudAcceleration(tga::Interface& tgai, const std::string_view scenePath)
    : backend_(tgai)
{
    // TODO: @Atzubi

    const auto batches = LoadScene(scenePath);
    batchCount_ = static_cast<std::uint32_t>(batches.size());

    // Create buffers for points
    {
        const auto [lowPrecisions, mediumPrecisions, highPrecisions, colors] = ConvertToAdaptivePrecision(batches);

        const size_t numberOfPoints = lowPrecisions.size();

        static_assert(sizeof(CompressedPosition) == sizeof(CompressedColor));
        const size_t entrySize = numberOfPoints * sizeof(CompressedPosition);

        const tga::StagingBufferInfo stagingInfo{entrySize};
        const tga::StagingBuffer staging = backend_.createStagingBuffer(stagingInfo);
        auto *stagingPointer = backend_.getMapping(staging);

        const tga::BufferInfo bufferInfo{tga::BufferUsage::storage, entrySize, staging};

        // Low Precision
        std::memcpy(stagingPointer, lowPrecisions.data(), entrySize);
        pointsBufferPack_.positionLowPrecision = backend_.createBuffer(bufferInfo);

        // Medium Precision
        std::memcpy(stagingPointer, mediumPrecisions.data(), entrySize);
        pointsBufferPack_.positionMediumPrecision = backend_.createBuffer(bufferInfo);

        // High Precision
        std::memcpy(stagingPointer, highPrecisions.data(), entrySize);
        pointsBufferPack_.positionHighPrecision = backend_.createBuffer(bufferInfo);

        // Colors
        std::memcpy(stagingPointer, colors.data(), entrySize);
        pointsBufferPack_.colors = backend_.createBuffer(bufferInfo);

        backend_.free(staging);
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
