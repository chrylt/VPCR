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

std::vector<Point> SampleLeafs(const std::uint32_t maxCount, const std::uint64_t prefix, const std::uint8_t iteration,
                               std::vector<Batch>& batches, std::vector<Point>& points)
{
    // Loop over all batches and find all those that have the matching prefix, then select maxCount random points in all
    // of them, extract and return them

    // Find out how many matching batches there are
    std::vector<Batch *> matches;
    for (auto& batch : batches) {
        if ((batch.points.size() > 0) &&
            ((batch.mortonOrder.code >> ((batch.mortonOrder.iteration - iteration) * 3)) == prefix)) {
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
        batch->points.pop_back();

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
}  // namespace

TGAPointCloudAcceleration::TGAPointCloudAcceleration(tga::Interface& tgai, const std::string_view scenePath)
    : backend_(tgai)
{
    auto originalBatches = LoadScene(scenePath);

    std::vector<Point> originalPoints;
    originalPoints.reserve(originalBatches.size() * MaxBatchSize);

    // Use std::map sorting of morton order to our advantage
    std::map<std::uint8_t, std::map<std::uint64_t, Node>> layeredTree;

    // Sort batches as leafes into the layeredTree
    std::uint8_t maxIteration = 0;
    std::uint32_t offset = 0;
    for (const auto& batch : originalBatches) {
        originalPoints.insert(originalPoints.end(), batch.points.begin(), batch.points.end());
        layeredTree[batch.mortonOrder.iteration][batch.mortonOrder.code] = {
            batch.box, offset, static_cast<std::uint32_t>(batch.points.size()), 0};
        offset += static_cast<std::uint32_t>(batch.points.size());
        maxIteration = std::max(static_cast<std::uint8_t>(batch.mortonOrder.iteration), maxIteration);
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
            auto sampledPoints = SampleLeafs(MaxBatchSize, code, i, originalBatches, originalPoints);
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
            for (auto& point : sampledPoints) {
                box.minV = glm::min(box.minV, point.position);
                box.maxV = glm::max(box.maxV, point.position);
                points.push_back(std::move(point));
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

    batchCount_ = static_cast<std::uint32_t>(batchNodes.size());

    // Create buffer for points
    {
        const tga::StagingBufferInfo stagingInfo{points.size() * sizeof(Point),
                                                 reinterpret_cast<const std::uint8_t *>(points.data())};
        const tga::StagingBuffer staging = backend_.createStagingBuffer(stagingInfo);
        const tga::BufferInfo info{tga::BufferUsage::storage, points.size() * sizeof(Point), staging};
        pointsBuffer_ = backend_.createBuffer(info);

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

tga::Buffer TGAPointCloudAcceleration::GetPointsBuffer() const { return pointsBuffer_; }

tga::Buffer TGAPointCloudAcceleration::GetBatchesBuffer() const { return batchesBuffer_; }

std::uint32_t TGAPointCloudAcceleration::GetBatchCount() const { return batchCount_; }

TGAPointCloudAcceleration::~TGAPointCloudAcceleration()
{
    backend_.free(pointsBuffer_);
    backend_.free(batchesBuffer_);
    backend_.free(accelerationBuffer_);
}