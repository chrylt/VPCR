#pragma once

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

std::vector<Point> LoadScene(std::string_view scene);

AABB CreateInitializerBox();
