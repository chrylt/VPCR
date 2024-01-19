#include "TGAPointCloudAcceleration.h"

#include <optional>
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

struct BatchesCompressed {
    std::vector<CompressedPosition> lowPrecisions;
    std::vector<CompressedPosition> mediumPrecisions;
    std::vector<CompressedPosition> highPrecisions;
    std::vector<CompressedColor> colors;
};

std::vector<Point> SampleLeafs(const std::uint32_t maxCount, const std::uint64_t prefix, const std::uint8_t iteration,
                               std::vector<Batch>& batches)
{
    // Loop over all batches and find all those that have the matching prefix, then select maxCount random points in all
    // of them, extract and return them

    // Find out how many matching batches there are
    std::vector<Batch *> matches;
    for (auto& batch : batches) {
        if ((batch.points.size() > 0) && (batch.id.iteration >= iteration) &&
            ((batch.GetMortonCode() >> ((batch.id.iteration - iteration) * 3)) == prefix)) {
            matches.push_back(&batch);
        }
    }

    if (matches.size() == 0) {
        return {};
    }

    std::vector<Point> sampledPoints;
    sampledPoints.reserve(maxCount);
    for (std::uint32_t i = 0; i < maxCount; ++i) {
        // Choose random matching batch
        std::random_device r;
        std::default_random_engine rng(r());
        const auto batchChoice =
            std::uniform_int_distribution<std::uint32_t>(0, static_cast<std::uint32_t>(matches.size() - 1))(rng);
        auto *batch = matches[batchChoice];

        // Choose random point and extract it
        const auto pointChoice =
            std::uniform_int_distribution<std::uint32_t>(0, static_cast<std::uint32_t>(batch->points.size() - 1))(rng);
        sampledPoints.push_back(std::move(batch->points[pointChoice]));
        batch->points[pointChoice] = batch->points.back();
        batch->points = batch->points.subspan(0, batch->points.size() - 1);

        // If batch is now empty, remove it
        if (batch->points.size() == 0) {
            matches[batchChoice] = matches.back();
            matches.pop_back();
            if (matches.size() == 0) {
                // If matches are empty there is nothing left to sample
                return sampledPoints;
            }
        }
    }

    return sampledPoints;
}

FlatLODTree CreateLODTree(const std::string_view scenePath)
{
    auto [originalPoints, originalBatches] = LoadScene(scenePath);

    // Use std::map sorting of morton order to our advantage
    std::map<std::uint8_t, std::map<std::uint64_t, Node>> layeredTree;

    // Sort batches as leafes into the layeredTree
    std::uint8_t maxIteration = 0;
    std::uint32_t offset = 0;
    for (const auto& batch : originalBatches) {
        layeredTree[batch.id.iteration][batch.GetMortonCode()] = {};
        offset += static_cast<std::uint32_t>(batch.points.size());
        maxIteration = std::max(static_cast<std::uint8_t>(batch.id.iteration), maxIteration);
    }

    // Create node hierarchy by iterating the layers in reverse order
    std::uint8_t rootLevel = 0;
    for (std::uint8_t i = maxIteration; i > 0; --i) {
        const auto& layer = layeredTree[i];
        if (layer.size() == 1) {
            rootLevel = i;
            break;
        }
        for (const auto& [code, mortonNodes] : layer) {
            const auto parentCode = code >> 3;
            if (!layeredTree[i - 1].contains(parentCode)) {
                layeredTree[i - 1][parentCode] = {};
            }
        }
    }

    // Pull up MaxBatchSize many random points from children as LOD for each node, proceed top down
    std::vector<Node> batchNodes;
    std::vector<Point> points;
    batchNodes.reserve(originalPoints.size() / MaxBatchSize + 1);
    points.reserve(originalPoints.size());
    for (std::uint8_t i = rootLevel; i <= maxIteration; ++i) {
        auto& layer = layeredTree[i];
        for (auto& [code, node] : layer) {
            // Pull points from leaf level
            const auto sampledPoints = SampleLeafs(MaxBatchSize, code, i, originalBatches);
            if (sampledPoints.empty()) {
                continue;
            }

            // Get AABB and write points to global buffer
            AABB box{
                {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
                 std::numeric_limits<float>::infinity()},
                {-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
                 -std::numeric_limits<float>::infinity()},
            };
            node.pointOffset = static_cast<std::uint32_t>(points.size());
            for (const auto& point : sampledPoints) {
                box.minV = glm::min(box.minV, point.position);
                box.maxV = glm::max(box.maxV, point.position);
                points.push_back(point);
            }

            if (i != rootLevel) {
                // Set current node as active child of parent
                const std::uint8_t childBit = 1 << (code & 0b111);  // Mask out the morton index for the child octant
                auto& parent = layeredTree[i - 1][code >> 3];
                parent.childMask |= childBit;
                if (parent.childrenPointer == 0) {
                    parent.childrenPointer = static_cast<std::uint32_t>(batchNodes.size());
                }
            }
            node.depth = i - rootLevel;
            node.pointCount = points.size() - node.pointOffset;
            node.box = std::move(box);
            node.childrenPointer = 0;  // Set to 0 until first child sets it active

            // Due to children being sorted by morton code in the map, we are guaranteed that all children of a
            // parent are pushed back after another, hence having a single child pointer to the first child in the
            // parent is sufficient and no further sorting is necessary
            batchNodes.push_back(node);
        }
    }

    // Update children relations
    std::uint32_t batchPointer = 0;
    for (std::uint8_t i = rootLevel; i <= maxIteration; ++i) {
        const auto& layer = layeredTree[i];
        for (const auto& [code, node] : layer) {
            if (node.pointCount == 0) {
                continue;
            }

            batchNodes[batchPointer].childrenPointer = node.childrenPointer;
            batchNodes[batchPointer].childMask = node.childMask;
            ++batchPointer;
        }
    }

    // std::vector<Node> batches;
    // std::uint32_t offset = 0;
    // for (const auto& batch : originalBatches) {
    //     Node node{};
    //     node.box = batch.aabb;
    //     node.pointOffset = offset;
    //     node.pointCount = static_cast<std::uint32_t>(batch.points.size());
    //     batches.push_back(std::move(node));
    //     offset += static_cast<std::uint32_t>(batch.points.size());
    // }

    return {points, batchNodes};
}

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
            const auto& [position, color] = points[i];
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
    const auto [points, batchNodes] = CreateLODTree(scenePath);

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
