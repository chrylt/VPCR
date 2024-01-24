#include "Utils.h"

#define TINYGLTF_IMPLEMENTATION
#include <tinygltf/tiny_gltf.h>

#include <execution>
#include <unordered_set>

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

bmp::uint1024_t MortonIndex768(const Point p, const double quantizationFactor)
{
    using namespace bmp::literals;

    bmp::int256_t ix, iy, iz;
    ix.assign(p.position.x / quantizationFactor);
    iy.assign(p.position.y / quantizationFactor);
    iz.assign(p.position.z / quantizationFactor);

    // Converts a signed value to an unsigned value.
    const bmp::uint256_t bias = 0x7fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff_cppui256;
    ix = ix.backend().isneg() ? -ix : ix + bias;
    iy = iy.backend().isneg() ? -iy : iy + bias;
    iz = iz.backend().isneg() ? -iz : iz + bias;

    bmp::uint1024_t xx = ix.convert_to<bmp::uint1024_t>();
    bmp::uint1024_t yy = iy.convert_to<bmp::uint1024_t>();
    bmp::uint1024_t zz = iz.convert_to<bmp::uint1024_t>();

    // Dilate and combine
    xx =
        (xx | (xx << 256U)) &
        0x0000000000000000000000000000000000000000000000000000000000000000ffffffffffffffffffffffffffffffff0000000000000000000000000000000000000000000000000000000000000000ffffffffffffffffffffffffffffffff_cppui1024;
    yy =
        (yy | (yy << 256U)) &
        0x0000000000000000000000000000000000000000000000000000000000000000ffffffffffffffffffffffffffffffff0000000000000000000000000000000000000000000000000000000000000000ffffffffffffffffffffffffffffffff_cppui1024;
    zz =
        (zz | (zz << 256U)) &
        0x0000000000000000000000000000000000000000000000000000000000000000ffffffffffffffffffffffffffffffff0000000000000000000000000000000000000000000000000000000000000000ffffffffffffffffffffffffffffffff_cppui1024;

    xx =
        (xx | (xx << 128U)) &
        0x00000000000000000000000000000000ffffffffffffffff00000000000000000000000000000000ffffffffffffffff00000000000000000000000000000000ffffffffffffffff00000000000000000000000000000000ffffffffffffffff_cppui1024;
    yy =
        (yy | (yy << 128U)) &
        0x00000000000000000000000000000000ffffffffffffffff00000000000000000000000000000000ffffffffffffffff00000000000000000000000000000000ffffffffffffffff00000000000000000000000000000000ffffffffffffffff_cppui1024;
    zz =
        (zz | (zz << 128U)) &
        0x00000000000000000000000000000000ffffffffffffffff00000000000000000000000000000000ffffffffffffffff00000000000000000000000000000000ffffffffffffffff00000000000000000000000000000000ffffffffffffffff_cppui1024;

    xx =
        (xx | (xx << 64U)) &
        0x0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff_cppui1024;
    yy =
        (yy | (yy << 64U)) &
        0x0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff_cppui1024;
    zz =
        (zz | (zz << 64U)) &
        0x0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff0000000000000000ffffffff_cppui1024;

    xx =
        (xx | (xx << 32U)) &
        0x00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff_cppui1024;
    yy =
        (yy | (yy << 32U)) &
        0x00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff_cppui1024;
    zz =
        (zz | (zz << 32U)) &
        0x00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff00000000ffff_cppui1024;

    xx =
        (xx | (xx << 16U)) &
        0x0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff_cppui1024;
    yy =
        (yy | (yy << 16U)) &
        0x0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff_cppui1024;
    zz =
        (zz | (zz << 16U)) &
        0x0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff0000ff_cppui1024;

    xx =
        (xx | (xx << 8U)) &
        0x00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f_cppui1024;
    yy =
        (yy | (yy << 8U)) &
        0x00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f_cppui1024;
    zz =
        (zz | (zz << 8U)) &
        0x00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f00f_cppui1024;

    xx =
        (xx | (xx << 4U)) &
        0x0c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c3_cppui1024;
    yy =
        (yy | (yy << 4U)) &
        0x0c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c3_cppui1024;
    zz =
        (zz | (zz << 4U)) &
        0x0c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c30c3_cppui1024;

    xx =
        (xx | (xx << 2U)) &
        0x249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249_cppui1024;
    yy =
        (yy | (yy << 2U)) &
        0x249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249_cppui1024;
    zz =
        (zz | (zz << 2U)) &
        0x249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249249_cppui1024;

    return xx | (yy << 1U) | (zz << 2U);
}
}  // namespace

AABB CreateInitializerBox()
{
    AABB aabb = {{std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
                  std::numeric_limits<float>::infinity()},
                 {-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
                  -std::numeric_limits<float>::infinity()}};

    return aabb;
}

std::vector<MortonPoint> LoadScene(std::string_view scene)
{
    const auto model = LoadModel(scene);

    std::unordered_set<Point> points;
    std::vector<Point> pointBuffer;
    for (const auto& mesh : model.meshes) {
        for (const auto& primitive : mesh.primitives) {
            if (primitive.attributes.empty()) {
                continue;
            }

            auto modelMatrix = glm::mat4(1.0);
            if (!model.nodes.empty()) {
                const auto& matrix = model.nodes[0].matrix;
                modelMatrix = glm::mat4(matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5], matrix[6],
                                        matrix[7], matrix[8], matrix[9], matrix[10], matrix[11], matrix[12], matrix[13],
                                        matrix[14], matrix[15]);
            }

            pointBuffer.resize(model.accessors[primitive.attributes.begin()->second].count);

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
                        memcpy(&pointBuffer[i].position, &buffer[bufferOffset + i * stride], stride);
                        pointBuffer[i].position = modelMatrix * glm::vec4(pointBuffer[i].position, 1);
                    }
                } else if (accessorName.substr(0, 5) == "COLOR") {
                    for (std::uint32_t i = 0; i < primPointCount; ++i) {
                        glm::vec4 fColor;
                        memcpy(&fColor, &buffer[bufferOffset + i * stride], stride);

                        pointBuffer[i].color.r = static_cast<std::uint8_t>(fColor.r * 255);
                        pointBuffer[i].color.g = static_cast<std::uint8_t>(fColor.g * 255);
                        pointBuffer[i].color.b = static_cast<std::uint8_t>(fColor.b * 255);
                        pointBuffer[i].color.a = 255;
                    }
                }
            }

            points.insert(pointBuffer.begin(), pointBuffer.end());
            pointBuffer.clear();
        }
    }

    float minDistance = std::numeric_limits<float>::max();
    std::optional<glm::vec3> lastPos;
    for (const auto& point : points) {
        if (lastPos.has_value()) {
            minDistance = std::min(minDistance, glm::distance(point.position, lastPos.value()));
        }
        lastPos = point.position;
    }
    // Apply safety factor
    double scale = 10.0 / minDistance;

    std::vector<MortonPoint> mortonPoints(points.begin(), points.end());

    std::for_each(std::execution::par_unseq, mortonPoints.begin(), mortonPoints.end(),
                  [&scale](MortonPoint& p) { p.mortonCode = MortonIndex768(p.point, scale); });

    std::sort(std::execution::par_unseq, mortonPoints.begin(), mortonPoints.end());

    return mortonPoints;
}

bool Point::operator==(const Point& q) const { return position == q.position; }

bool MortonPoint::operator<(const MortonPoint& q) const { return mortonCode < mortonCode; }