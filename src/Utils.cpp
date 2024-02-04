#include "Utils.h"

#define TINYGLTF_IMPLEMENTATION
#include <tinygltf/tiny_gltf.h>

#include <execution>
#include <iostream>
#include <optional>

namespace
{
// Throws exception if model was not loaded. Either no file or faulty gltf.
tinygltf::Model LoadModel(const std::string_view filename)
{
    tinygltf::Model model;

    tinygltf::TinyGLTF loader;
    std::string err;
    std::string warn;

    const auto res = loader.LoadASCIIFromFile(&model, &err, &warn, filename.data());
    if (!res) {
        throw std::runtime_error("Failed to load glTF");
    }

    if (!warn.empty()) {
        std::cout << "WARN: " << warn << std::endl;
    }

    if (!err.empty()) {
        std::cout << "ERR: " << err << std::endl;
    }

    return model;
}

std::vector<Point> LoadScenePoints(const std::string_view scene)
{
    std::vector<Point> points;

    const auto model = LoadModel(scene);

    auto modelMatrix = glm::mat4(1.0);
    if (!model.nodes.empty()) {
        const auto& matrix = model.nodes[0].matrix;
        if (!matrix.empty()) {
            modelMatrix =
                glm::mat4(matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5], matrix[6], matrix[7],
                          matrix[8], matrix[9], matrix[10], matrix[11], matrix[12], matrix[13], matrix[14], matrix[15]);
        }
    }

    std::size_t totalPointCount = 0;
    for (const auto& mesh : model.meshes) {
        for (const auto& primitive : mesh.primitives) {
            for (const auto& [accessorName, accessorID] : primitive.attributes) {
                totalPointCount += model.accessors[accessorID].count;
                break;
            }
        }
    }
    points.resize(totalPointCount);

    std::size_t currentPointCount = 0;
    for (const auto& mesh : model.meshes) {
        for (const auto& primitive : mesh.primitives) {
            for (const auto& [accessorName, accessorID] : primitive.attributes) {
                const auto& accessor = model.accessors[accessorID];

                const auto& bufferViewID = accessor.bufferView;
                const auto& bufferView = model.bufferViews[bufferViewID];

                const auto& bufferID = bufferView.buffer;
                const auto& buffer = model.buffers[bufferID].data;

                const auto& primPointCount = accessor.count;
                const auto bufferOffset = accessor.byteOffset + bufferView.byteOffset;
                const auto stride = accessor.ByteStride(bufferView);

                if (accessorName == "POSITION") {
                    for (std::uint32_t i = 0; i < primPointCount; ++i) {
                        glm::vec4 position = glm::vec4(0, 0, 0, 1);
                        memcpy(&position, &buffer[bufferOffset + i * stride], stride);

                        points[currentPointCount + i].position = modelMatrix * position;
                    }
                } else if (accessorName.substr(0, 5) == "COLOR") {
                    for (std::uint32_t i = 0; i < primPointCount; ++i) {
                        glm::vec4 fColor;
                        memcpy(&fColor, &buffer[bufferOffset + i * stride], stride);

                        points[currentPointCount + i].color.r = static_cast<std::uint8_t>(fColor.r * 255);
                        points[currentPointCount + i].color.g = static_cast<std::uint8_t>(fColor.g * 255);
                        points[currentPointCount + i].color.b = static_cast<std::uint8_t>(fColor.b * 255);
                        points[currentPointCount + i].color.a = 255;
                    }
                }
            }

            // get count from first attribute
            if (!primitive.attributes.empty()) {
                const auto& attribute = primitive.attributes.begin()->second;
                currentPointCount += model.accessors[attribute].count;
            }
        }
    }

    return points;
}

std::uint64_t MortonIndex64(const Point& p, const double scale)
{
    const std::int64_t ix = static_cast<int64_t>(p.position.x * scale);
    const std::int64_t iy = static_cast<int64_t>(p.position.y * scale);
    const std::int64_t iz = static_cast<int64_t>(p.position.z * scale);

    // Test whether the scene, after scaling is too large to represent with a 63 bit morton code
    // The code may not break regardless but we should make sure that our scenes we want to present fit
    if (((ix >= (1 << 20)) || (ix < -(1 << 20)) || (iy >= (1 << 20)) || (iy < -(1 << 20)) || (iz >= (1 << 20)) ||
         (iz < -(1 << 20)))) {
        throw std::runtime_error("The scene is too large and can not be represented by a 63 bit morton code!");
    }

    // Mask out lowest 21 bits
    std::uint64_t xx = ix & (0b111111111111111111111);
    std::uint64_t yy = iy & (0b111111111111111111111);
    std::uint64_t zz = iz & (0b111111111111111111111);

    // Dilate and combine
    xx = (xx | (xx << 32U)) & 0x001f00000000ffffLL;
    yy = (yy | (yy << 32U)) & 0x001f00000000ffffLL;
    zz = (zz | (zz << 32U)) & 0x001f00000000ffffLL;

    xx = (xx | (xx << 16U)) & 0x001f0000ff0000ffLL;
    yy = (yy | (yy << 16U)) & 0x001f0000ff0000ffLL;
    zz = (zz | (zz << 16U)) & 0x001f0000ff0000ffLL;

    xx = (xx | (xx << 8U)) & 0x100f00f00f00f00fLL;
    yy = (yy | (yy << 8U)) & 0x100f00f00f00f00fLL;
    zz = (zz | (zz << 8U)) & 0x100f00f00f00f00fLL;

    xx = (xx | (xx << 4U)) & 0x10c30c30c30c30c3LL;
    yy = (yy | (yy << 4U)) & 0x10c30c30c30c30c3LL;
    zz = (zz | (zz << 4U)) & 0x10c30c30c30c30c3LL;

    xx = (xx | (xx << 2U)) & 0x1249249249249249LL;
    yy = (yy | (yy << 2U)) & 0x1249249249249249LL;
    zz = (zz | (zz << 2U)) & 0x1249249249249249LL;

    return xx | (yy << 1U) | (zz << 2U);
}
}  // namespace

AABB CreateInitializerBox()
{
    return {{std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
             std::numeric_limits<float>::infinity()},
            {-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
             -std::numeric_limits<float>::infinity()}};
}

std::vector<Point> LoadScene(const std::string_view scene)
{
    auto points = LoadScenePoints(scene);

    float minDistance = std::numeric_limits<float>::max();
    std::optional<glm::vec3> lastPos;
    for (const auto& [position, _, __] : points) {
        if (lastPos.has_value()) {
            // Ignore points with the same position
            if (const auto dist = glm::distance(position, lastPos.value()); dist != 0) {
                minDistance = std::min(minDistance, glm::distance(position, lastPos.value()));
            }
        }
        lastPos = position;
    }
    // Apply safety factor
    const double scale = 1.2 / minDistance;

    std::for_each(std::execution::par_unseq, points.begin(), points.end(),
                  [&scale](Point& p) { p.mortonCode = MortonIndex64(p, scale); });

    return points;
}

bool Point::operator<(const Point& q) const { return mortonCode < q.mortonCode; }
