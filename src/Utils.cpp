#include "Utils.h"

#define TINYGLTF_IMPLEMENTATION
#include <tinygltf/tiny_gltf.h>

#include <iostream>

namespace
{
// Throws exception if model was not loaded. Either no file or faulty gltf.
auto LoadModel(const std::string_view filename)
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
        modelMatrix =
            glm::mat4(matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5], matrix[6], matrix[7], matrix[8],
                      matrix[9], matrix[10], matrix[11], matrix[12], matrix[13], matrix[14], matrix[15]);
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

                const auto bufferViewID = accessor.bufferView;
                const auto& bufferView = model.bufferViews[bufferViewID];

                const auto bufferID = bufferView.buffer;
                const auto& buffer = model.buffers[bufferID].data;

                const auto primPointCount = accessor.count;
                const auto bufferOffset = accessor.byteOffset + bufferView.byteOffset;
                const auto stride = accessor.ByteStride(bufferView);

                if (accessorName.compare("POSITION") == 0) {
                    for (std::uint32_t i = 0; i < primPointCount; ++i) {
                        glm::vec4 position = glm::vec4(0, 0, 0, 1);
                        memcpy(&position, &buffer[bufferOffset + i * stride], stride);

                        points[currentPointCount + i].position = modelMatrix * position;
                    }
                } else if (accessorName.compare(0, 5, "COLOR") == 0) {
                    for (std::uint32_t i = 0; i < primPointCount; ++i) {
                        glm::vec4 fColor;
                        memcpy(&fColor, &buffer[bufferOffset + i * stride], stride);

                        points[currentPointCount + i].r = static_cast<std::uint8_t>(fColor.r * 255);
                        points[currentPointCount + i].g = static_cast<std::uint8_t>(fColor.g * 255);
                        points[currentPointCount + i].b = static_cast<std::uint8_t>(fColor.b * 255);
                        points[currentPointCount + i].a = 255;
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

}  // namespace

std::vector<Batch> LoadScene(std::string_view scene, std::vector<Point>& points)
{
    points = LoadScenePoints(scene);

    // TODO add a hashmap for morton code to make this faster or
    // Implement this https://ieeexplore.ieee.org/document/5383353
    std::sort(points.begin(), points.end());

    constexpr auto maxBatchSize = 4;
    auto batch = Batch(0U, 0ULL, std::span(points));
    return batch.Subdivide(maxBatchSize);
}

std::uint64_t Point::mortonIndex() const
{
    auto ix = reinterpret_cast<const std::uint32_t&>(position.x);
    auto iy = reinterpret_cast<const std::uint32_t&>(position.y);
    auto iz = reinterpret_cast<const std::uint32_t&>(position.z);

    // Get signed bit
    const auto ixs = static_cast<const std::int32_t>(ix) >> 31U;
    const auto iys = static_cast<const std::int32_t>(iy) >> 31U;
    const auto izs = static_cast<const std::int32_t>(iz) >> 31U;

    // This is a combination of a fast absolute value and a bias.
    //
    // We need to adjust the values so -FLT_MAX is close to 0.
    //
    ix = (((ix & 0x7FFFFFFFUL) ^ ixs) - ixs) + 0x7FFFFFFFUL;
    iy = (((iy & 0x7FFFFFFFUL) ^ iys) - iys) + 0x7FFFFFFFUL;
    iz = (((iz & 0x7FFFFFFFUL) ^ izs) - izs) + 0x7FFFFFFFUL;

    // We will only use the 21 MSBs
    std::uint64_t xx = ix >> 11U;
    std::uint64_t yy = iy >> 11U;
    std::uint64_t zz = iz >> 11U;

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

Batch::Batch() : points(std::span<Point>()), iteration(0), mortonCode(0), leaf(false)
{
    aabb = {
        {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
         std::numeric_limits<float>::infinity()},
        {-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
         -std::numeric_limits<float>::infinity()},
    };
};

Batch::Batch(auto _iteration, auto _mortonCode, auto _points)
    : points(_points), iteration(_iteration), mortonCode(_mortonCode), leaf(false)
{
    aabb = {
        {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
         std::numeric_limits<float>::infinity()},
        {-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
         -std::numeric_limits<float>::infinity()},
    };
}

std::vector<Batch> Batch::Subdivide(const std::uint32_t maxBatchCount)
{
    std::vector<Batch> result;

    if (points.size() < maxBatchCount) {
        leaf = true;
        result.push_back(*this);
    } else {
        std::array<Batch, 8> children;
        for (size_t i = 0; i < children.size(); i++) {
            children[i] = Batch(iteration + 1, i << (54U - iteration * 3U) | mortonCode, points.subspan(0, 0));
        }

        // splitting points into octree nodes
        auto prevNodeStartIT = points.begin();
        auto prevPointNodeID = MortonToNodeID(points.begin()->mortonIndex());
        for (auto it = points.begin(); it != points.end(); ++it) {
            const auto currPointNodeID = MortonToNodeID(it->mortonIndex());

            auto& box = children[currPointNodeID].aabb;
            box.minV = glm::min(box.minV, it->position);
            box.maxV = glm::max(box.maxV, it->position);

            // if NodeID changed or first run
            if (prevPointNodeID != currPointNodeID || it == prevNodeStartIT) {
                children[prevPointNodeID].points = std::span<Point>(prevNodeStartIT, it);
                children[currPointNodeID].points = std::span<Point>(it, points.end());

                prevNodeStartIT = it;
                prevPointNodeID = currPointNodeID;
            }
        }

        // filter subdivide results
        for (auto i = 0; i < children.size(); i++) {
            auto res = children[i].Subdivide(maxBatchCount);

            // return just leafs and non-empty nodes
            for (auto& node : res) {
                if (!node.leaf || node.points.size() == 0) {
                    continue;
                }

                result.push_back(node);
            }
        }
    }

    return result;
}

const std::uint32_t Batch::MortonToNodeID(const std::uint64_t mortonCode) const
{
    return (mortonCode >> (60U - iteration * 3U)) & 0x7U;
}
