#pragma once

#include <glm/glm.hpp>
#include <string_view>
#include <vector>

class Point {
public:
    glm::vec3 position;
    std::uint32_t r : 8;
    std::uint32_t g : 8;
    std::uint32_t b : 8;
    std::uint32_t a : 8;

    bool operator<(Point q) { 
        return this->mortonIndex() < q.mortonIndex(); 
    }

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

class BatchInfo {
public:
    bool leaf = false;

    std::uint32_t iteration = 0U;
    std::uint64_t mortonCode = 0ULL;

    std::uint32_t count = 0U;
    std::uint32_t startId = 0U;

    const Point *points = nullptr;
    const std::uint32_t maxBatchCount = 0U;

    BatchInfo(auto _maxBatchCount, auto _iteration, auto _mortonCode, auto _count, auto _points);

    std::vector<BatchInfo> Subdivide();
};

struct Batch {
    AABB aabb;
    std::vector<Point> points;
};

std::vector<Batch> LoadScene(std::string_view scene);
