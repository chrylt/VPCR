#pragma once

#include <glm/glm.hpp>
#include <span>
#include <string_view>
#include <vector>

constexpr auto MaxBatchSize = 256;

struct CompressedPosition {
    std::uint32_t x : 10;
    std::uint32_t y : 10;
    std::uint32_t z : 10;
    std::uint32_t padding : 2;
};

struct CompressedColor {
    std::uint32_t r : 8;
    std::uint32_t g : 8;
    std::uint32_t b : 8;
    std::uint32_t a : 8;
};

struct Point {
    glm::vec3 position;
    CompressedColor color;

    bool operator<(const Point& q) const;

    // https://stackoverflow.com/questions/26856268/morton-index-from-2d-point-with-floats
    static_assert((sizeof(std::uint32_t) == sizeof(float)) && (sizeof(std::uint32_t) * CHAR_BIT == 32) &&
                      (sizeof(std::uint64_t) * CHAR_BIT == 64),
                  "We need 32-bit ints and floats, and 64-bit long longs!");

    std::uint64_t MortonIndex() const;
};

struct AABB {
    alignas(16) glm::vec3 minV;
    alignas(16) glm::vec3 maxV;

    AABB()
    {
        minV = {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
                std::numeric_limits<float>::infinity()};
        maxV = {-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
                -std::numeric_limits<float>::infinity()};
    }
};

struct BatchID {
    std::uint64_t mortonCode_ : 57;
    std::uint64_t iteration_ : 5;
    std::uint64_t leaf_ : 1;
    std::uint64_t pad_ : 1;
};

struct Batch {
    BatchID id_;

    AABB aabb_;
    std::span<Point> points_;

    Batch(const std::uint32_t iteration, const std::uint64_t mortonCode, const std::span<Point> points,
          const AABB aabb = AABB(), const bool leaf = false);

    std::vector<Batch> Subdivide() const;

private:
    std::uint32_t MortonToNodeID(const std::uint64_t mortonCode) const;
    std::uint64_t NodeIDToMorton(const std::uint32_t nodeID) const;
};

class BatchedPointCloud {
    std::vector<Point> points_;
    std::vector<Batch> batches_;

public:
    BatchedPointCloud(std::vector<Point>&& points);

    const std::vector<Point> GetPoints();
    const std::vector<Batch> GetBatches();
};

BatchedPointCloud LoadScene(const std::string_view scene);
