#pragma once

#include <glm/glm.hpp>
#include <string_view>
#include <vector>

struct Point {
    glm::vec3 position;
    std::uint32_t r : 8;
    std::uint32_t g : 8;
    std::uint32_t b : 8;
    std::uint32_t a : 8;
};

struct MortonOrder {
    std::uint64_t code : 57;      // Max tree depth of 19
    std::uint64_t reserved : 2;   // 2 unused bits we could use for flags
    std::uint64_t iteration : 5;  // Up to 32 morton iterations but we can only use 19 anyways
};

struct AABB {
    glm::vec3 minV;
    glm::vec3 maxV;
};

struct Batch {
    MortonOrder mortonOrder;
    AABB box;
    std::vector<Point> points;
};

constexpr std::uint32_t MaxBatchSize = 256;

std::vector<Batch> LoadScene(std::string_view scene);
