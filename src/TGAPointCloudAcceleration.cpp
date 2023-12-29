#include "TGAPointCloudAcceleration.h"

#include "Utils.h"

TGAPointCloudAcceleration::TGAPointCloudAcceleration(tga::Interface& tgai, const std::string_view scenePath)
    : backend_(tgai)
{
    // TODO: @Atzubi

    const auto batches = LoadScene(scenePath);
    batchCount_ = static_cast<std::uint32_t>(batches.size());

    // Create some dummy buffers until we have an acceleration structure

    // Create buffer for points
    {
        std::vector<Point> points;
        for (const auto& batch : batches) {
            points.insert(points.end(), batch.points.begin(), batch.points.end());
        }

        const tga::StagingBufferInfo stagingInfo{points.size() * sizeof(Point),
                                                 reinterpret_cast<const std::uint8_t *>(points.data())};
        const tga::StagingBuffer staging = backend_.createStagingBuffer(stagingInfo);
        const tga::BufferInfo info{tga::BufferUsage::storage, points.size() * sizeof(Point), staging};
        pointsBuffer_ = backend_.createBuffer(info);

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

tga::Buffer TGAPointCloudAcceleration::GetPointsBuffer() const { return pointsBuffer_; }

tga::Buffer TGAPointCloudAcceleration::GetAccelerationStructureBuffer() const { return accelerationBuffer_; }

tga::Buffer TGAPointCloudAcceleration::GetBatchesBuffer() const { return batchesBuffer_; }

std::uint32_t TGAPointCloudAcceleration::GetBatchCount() const { return batchCount_; }

TGAPointCloudAcceleration::~TGAPointCloudAcceleration()
{
    backend_.free(pointsBuffer_);
    backend_.free(batchesBuffer_);
    backend_.free(accelerationBuffer_);
}