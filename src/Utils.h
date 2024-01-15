#pragma once

#include <glm/glm.hpp>
#include <string_view>
#include <vector>
#include <span>

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

    const bool operator<(const Point q) const { return this->mortonIndex() < q.mortonIndex(); }

    // https://stackoverflow.com/questions/26856268/morton-index-from-2d-point-with-floats
    static_assert((sizeof(std::uint32_t) == sizeof(float)) && (sizeof(std::uint32_t) * CHAR_BIT == 32) &&
                      (sizeof(std::uint64_t) * CHAR_BIT == 64),
                  "We need 32-bit ints and floats, and 64-bit long longs!");

    std::uint64_t mortonIndex() const;
};

struct AABB {
    alignas(16) glm::vec3 minV;
    alignas(16) glm::vec3 maxV;
};

class Batch {
public:
    std::uint64_t mortonCode : 57;
    std::uint64_t iteration : 5;
    std::uint64_t leaf : 1;

    AABB aabb;
    std::span<Point> points;

    Batch();
    Batch(auto _iteration, auto _mortonCode, auto _points);

    std::vector<Batch> Subdivide();
    const std::uint32_t MortonToNodeID(const std::uint64_t mortonCode) const;
};

std::vector<Batch> LoadScene(std::string_view scene, std::vector<Point>& points);
