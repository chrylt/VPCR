#include "Utils.h"

#define TINYGLTF_IMPLEMENTATION
#include <tinygltf/tiny_gltf.h>

#include <iostream>

namespace
{
//Throws exception if model was not loaded. Either no file or faulty gltf.
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
        modelMatrix = glm::mat4(matrix[ 0], matrix[ 1], matrix[ 2], matrix[ 3], 
                                matrix[ 4], matrix[ 5], matrix[ 6], matrix[ 7],
                                matrix[ 8], matrix[ 9], matrix[10], matrix[11], 
                                matrix[12], matrix[13], matrix[14], matrix[15]);
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
                        glm::vec4 position = glm::vec4(0,0,0,1);
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

            //get count from first attribute
            if (!primitive.attributes.empty()) {
                const auto& attribute = primitive.attributes.begin()->second;
                currentPointCount += model.accessors[attribute].count;
            }
        }
    }

    return points;
}
}  // namespace

std::vector<Batch> LoadScene(const std::string_view scene)
{
    // TODO @J3oss
    // Vertex order optimization

    // Create dummy batches until we have vertex order optimization
    constexpr auto maxBatchSize = 4;
    const auto points = LoadScenePoints(scene);
    auto iterator = points.begin();

    std::vector<Batch> batches;
    batches.reserve(points.size() / maxBatchSize + 1);
    while (iterator != points.end()) {
        auto& batch = batches.emplace_back();
        batch.points.reserve(maxBatchSize);

        AABB box{
            {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
             std::numeric_limits<float>::infinity()},
            {-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
             -std::numeric_limits<float>::infinity()},
        };
        for (std::uint32_t i = 0; i < maxBatchSize; ++i) {
            if (iterator == points.end()) {
                break;
            }

            box.minV = glm::min(box.minV, (*iterator).position);
            box.maxV = glm::max(box.maxV, (*iterator).position);

            batch.points.push_back(*iterator);
            ++iterator;
        }
        batch.aabb = box;
    }

    return batches;
}