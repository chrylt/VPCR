#pragma once

#include <boost/multiprecision/cpp_int.hpp>
#include <glm/glm.hpp>
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

    bool operator==(const Point& q) const;
};

namespace std
{
template <>
struct hash<Point> {
    std::size_t operator()(const Point& p) const
    {
        return std::hash<float>{}(p.position.x) ^ std::hash<float>{}(p.position.y) ^ std::hash<float>{}(p.position.z);
    }
};
}  // namespace std

struct MortonPoint {
    Point point;
    bmp::uint1024_t mortonCode;

    bool operator<(const MortonPoint& q) const;
};

struct AABB {
    alignas(16) glm::vec3 minV;
    alignas(16) glm::vec3 maxV;
};

std::vector<MortonPoint> LoadScene(std::string_view scene);

AABB CreateInitializerBox();
