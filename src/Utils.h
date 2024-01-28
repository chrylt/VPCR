#pragma once

#include <span>
#include <glm/glm.hpp>
#include <string_view>
#include <vector>

constexpr auto MaxBatchSize = 8192;

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
    std::uint64_t mortonCode;

    bool operator<(const Point& q) const;
};

struct AABB {
    glm::vec3 minV;
    glm::vec3 maxV;
};

struct BatchID {
    std::uint64_t mortonCode : 57;
    std::uint64_t iteration : 5;
    std::uint64_t leaf : 1;
    std::uint64_t pad : 1;
};

struct Batch {
    BatchID id;

    AABB aabb;
    std::span<Point> points;

    Batch(std::uint32_t iteration, std::uint64_t mortonCode, std::span<Point> points, AABB aabb,
                bool leaf = false);

    std::vector<Batch> Subdivide() const;

private:
    std::uint32_t MortonToNodeID(std::uint64_t mortonCode) const;
    std::uint64_t NodeIDToMorton(std::uint32_t nodeID) const;
};

struct BatchedPointCloud {
    std::vector<Point> points;
    std::vector<Batch> batches;
};

struct Bucket {
    uint32_t bucketID;
    int32_t nextIdx;
    uint64_t acc;
};

struct Histogram {
    int32_t startIdx;
    uint32_t bucketCount;
    Bucket buckets[100];  // maximal 100 filled buckets per pixel
};

std::vector<Point> LoadScene(std::string_view scene);


AABB CreateInitializerBox();
