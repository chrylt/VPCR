#pragma once

#include <glm/glm.hpp>
#include <optional>
#include <span>
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

    bool operator<(const Point& q) const;

    // https://stackoverflow.com/questions/26856268/morton-index-from-2d-point-with-floats
    static_assert((sizeof(std::uint32_t) == sizeof(float)) && (sizeof(std::uint32_t) * CHAR_BIT == 32) &&
                      (sizeof(std::uint64_t) * CHAR_BIT == 64),
                  "We need 32-bit ints and floats, and 64-bit long longs!");

    std::uint64_t MortonIndex() const;

private:
    mutable std::optional<std::uint64_t> mortonCache_;
};

struct AABB {
    alignas(16) glm::vec3 minV;
    alignas(16) glm::vec3 maxV;
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

    Batch(std::uint32_t iteration, std::uint64_t mortonCode, std::span<Point> points, AABB aabb, bool leaf = false);

    std::vector<Batch> Subdivide() const;

private:
    std::uint32_t MortonToNodeID(std::uint64_t mortonCode) const;
    std::uint64_t NodeIDToMorton(std::uint32_t nodeID) const;
};

struct BatchedPointCloud {
    std::vector<Point> points;
    std::vector<Batch> batches;
};

BatchedPointCloud LoadScene(std::string_view scene);

AABB CreateInitializerBox();
