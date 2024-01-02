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

struct AABB {
    alignas(16) glm::vec3 minV;
    alignas(16) glm::vec3 maxV;
};

struct Batch {
    AABB aabb;
    std::vector<Point> points;
};

std::vector<Batch> LoadScene(std::string_view scene);
