#pragma once

#include <boost/multiprecision/cpp_int.hpp>
#include <glm/glm.hpp>
#include <span>
#include <string_view>
#include <vector>
namespace bmp = boost::multiprecision;

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
    bmp::uint1024_t mortonCode;

    bool operator<(const Point& q) const;
};

struct AABB {
    alignas(16) glm::vec3 minV;
    alignas(16) glm::vec3 maxV;
};

struct BatchID {
    bmp::uint1024_t mortonCode;
    std::uint16_t iteration : 12;
    std::uint16_t leaf : 1;
    std::uint16_t padding : 3;
};

struct Batch {
    BatchID id;

    AABB aabb;
    std::span<Point> points;

    Batch(std::uint32_t iteration, bmp::uint1024_t mortonCode, std::span<Point> points, AABB aabb, bool leaf = false);

    std::vector<Batch> Subdivide() const;

private:
    std::uint32_t MortonToNodeID(bmp::uint1024_t mortonCode) const;
    bmp::uint1024_t NodeIDToMorton(std::uint32_t nodeID) const;
};

struct BatchedPointCloud {
    std::vector<Point> points;
    std::vector<Batch> batches;
};

BatchedPointCloud LoadScene(std::string_view scene);

AABB CreateInitializerBox();
