#include "TGAPointCloudAcceleration.h"

#include <algorithm>
#include <random>

#include "Utils.h"

namespace
{

struct Node {
    AABB box;
    std::uint32_t childrenPointer;
    std::uint32_t pointOffset;
    std::uint32_t childMask : 8;
    std::uint32_t depth : 5;
    std::uint32_t pointCount : 19;  // 2^19 should be a reasonable maximum for the batch size
};

struct FlatLODTree {
    std::vector<Point> points;
    std::vector<Node> batchNodes;
};

// Relies on the code being 3 bits per depth level and being prepended by a 1
std::uint8_t GetDepth(std::uint64_t code)
{
    code >>= 3;
    std::uint8_t depth = 0;
    while (code > 0) {
        code >>= 3;
        ++depth;
    }
    return depth;
}

std::uint64_t CreateMortonWithDepth(std::uint64_t code, const std::uint8_t depth)
{
    assert(depth <= 21);
    // Set the highest bit as indicator that the code starts here
    code ^= 0x8000000000000000;
    code >>= 3 * (21 - depth);
    return code;
}

class MortonTree {
public:
    MortonTree(std::vector<Point> points) : pointCount_(static_cast<std::uint32_t>(points.size()))
    {
        // We want to insert points in random order to generate a LOD tree where each level is a subset of points from
        // the lower levels. We start by inserting points in the highest level, once that has MaxBatchSize many points,
        // the points will overflow into the children as defined by their morton code
        auto rng = std::default_random_engine{};
        std::shuffle(points.begin(), points.end(), rng);
        for (const auto& point : points) {
            std::uint8_t depth = 0;

            while (!InsertPoint(CreateMortonWithDepth(point.mortonCode, depth), point)) {
                ++depth;
            }
        }

        // TODO Sift points into child if node only has one child
    }

    FlatLODTree GetLODTree() const
    {
        FlatLODTree tree;
        tree.points.reserve(pointCount_);
        tree.batchNodes.reserve(batches_.size());

        std::uint32_t offset = 0;
        for (const auto& [code, batch] : batches_) {
            auto& node = tree.batchNodes.emplace_back();
            node.box = CreateInitializerBox();

            for (const auto& point : batch) {
                node.box.minV = glm::min(point.position, node.box.minV);
                node.box.maxV = glm::max(point.position, node.box.maxV);

                tree.points.push_back(point);
            }

            node.pointOffset = offset;
            node.pointCount = static_cast<std::uint32_t>(batch.size());
            node.depth = GetDepth(code);

            offset += node.pointCount;
        }

        // TODO add child relations

        return tree;
    }

private:
    bool InsertPoint(const std::uint64_t code, const Point& point)
    {
        if (batches_[code].size() == MaxBatchSize) {
            return false;
        }

        batches_[code].push_back(point);
        return true;
    }

    std::uint32_t pointCount_;
    std::unordered_map<std::uint64_t, std::vector<Point>> batches_;
};

struct BatchesCompressed {
    std::vector<CompressedPosition> lowPrecisions;
    std::vector<CompressedPosition> mediumPrecisions;
    std::vector<CompressedPosition> highPrecisions;
    std::vector<CompressedColor> colors;
};

BatchesCompressed ConvertToAdaptivePrecision(const std::vector<Node>& batches, const std::vector<Point>& points)
{
    std::vector<CompressedPosition> lowPrecisions;
    std::vector<CompressedPosition> mediumPrecisions;
    std::vector<CompressedPosition> highPrecisions;
    std::vector<CompressedColor> colors;

    lowPrecisions.reserve(batches.size() * MaxBatchSize);
    mediumPrecisions.reserve(batches.size() * MaxBatchSize);
    highPrecisions.reserve(batches.size() * MaxBatchSize);
    colors.reserve(batches.size() * MaxBatchSize);

    for (const auto& batch : batches) {
        for (std::size_t i = batch.pointOffset; i < batch.pointOffset + batch.pointCount; ++i) {
            const auto& [position, color, _] = points[i];
            // Compress position and split into multiple buffers
            const glm::vec3& floatPos = position;
            const glm::vec3 aabbSize = batch.box.maxV - batch.box.minV;

            // convert float position to 30-bit fixed precision relative to BB
            const auto x30 = std::min(
                static_cast<std::uint32_t>(std::floor((1 << 30) * (floatPos.x - batch.box.minV.x) / aabbSize.x)),
                static_cast<std::uint32_t>((1 << 30) - 1));
            const auto y30 = std::min(
                static_cast<std::uint32_t>(std::floor((1 << 30) * (floatPos.y - batch.box.minV.y) / aabbSize.y)),
                static_cast<std::uint32_t>((1 << 30) - 1));
            const auto z30 = std::min(
                static_cast<std::uint32_t>(std::floor((1 << 30) * (floatPos.z - batch.box.minV.z) / aabbSize.z)),
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
    const auto [points, batchNodes] = MortonTree(LoadScene(scenePath)).GetLODTree();

    batchCount_ = static_cast<std::uint32_t>(batchNodes.size());

    // Create buffers for points
    {
        const auto [lowPrecisions, mediumPrecisions, highPrecisions, colors] =
            ConvertToAdaptivePrecision(batchNodes, points);

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
        const tga::StagingBufferInfo stagingInfo{batchNodes.size() * sizeof(Node),
                                                 reinterpret_cast<const std::uint8_t *>(batchNodes.data())};
        const tga::StagingBuffer staging = backend_.createStagingBuffer(stagingInfo);
        const tga::BufferInfo info{tga::BufferUsage::storage, batchNodes.size() * sizeof(Node), staging};
        batchesBuffer_ = backend_.createBuffer(info);

        backend_.free(staging);
    }
}

TGAPointCloudAcceleration::PointBuffers TGAPointCloudAcceleration::GetPointsBufferPack() const
{
    return pointsBufferPack_;
}

tga::Buffer TGAPointCloudAcceleration::GetBatchesBuffer() const { return batchesBuffer_; }

std::uint32_t TGAPointCloudAcceleration::GetBatchCount() const { return batchCount_; }

TGAPointCloudAcceleration::~TGAPointCloudAcceleration()
{
    backend_.free(pointsBufferPack_.positionLowPrecision);
    backend_.free(pointsBufferPack_.positionMediumPrecision);
    backend_.free(pointsBufferPack_.positionHighPrecision);
    backend_.free(pointsBufferPack_.colors);
    backend_.free(batchesBuffer_);
}
