#pragma once

#include <glm/glm.hpp>
#include <string_view>
#include <vector>

// Should match compute shaders
constexpr std::uint32_t ComputeLaneCount = 1024;

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

struct Bucket {
    uint32_t bucketID;
    int32_t nextIdx;
    uint64_t acc;
};

#define BUCKET_COUNT_OPDAA 100
struct Histogram {
    int32_t startIdx;
    uint32_t bucketCount;
    Bucket buckets[BUCKET_COUNT_OPDAA];  // maximal 100 filled buckets per pixel
};

struct BucketTP {
    uint64_t acc;
};

#define BUCKET_COUNT_TPDAA 5
struct HistogramTP {
    BucketTP buckets[BUCKET_COUNT_TPDAA];  // maximal 5 division of depth of world-space pixel
};

std::vector<Point> LoadScene(std::string_view scene);

AABB CreateInitializerBox();
