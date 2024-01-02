#include "Utils.h"

#define TINYGLTF_IMPLEMENTATION
#include "../tinygltf/tiny_gltf.h"
#include <iostream>

namespace
{
bool LoadModel(tinygltf::Model& model, const char *filename)
{
    tinygltf::TinyGLTF loader;
    std::string err;
    std::string warn;

    bool res = loader.LoadASCIIFromFile(&model, &err, &warn, filename);
    if (!warn.empty()) {
        std::cout << "WARN: " << warn << std::endl;
    }

    if (!err.empty()) {
        std::cout << "ERR: " << err << std::endl;
    }

    if (!res)
        std::cout << "Failed to load glTF: " << filename << std::endl;
    else
        std::cout << "Loaded glTF: " << filename << std::endl;

    return res;
}

std::vector<Point> LoadScenePoints(const std::string_view scene)
{
    std::vector<Point> points;

    tinygltf::Model model;
    LoadModel(model, scene.data());

    auto matrix = model.nodes[0].matrix;
    glm::mat4 modelMatrix(
        matrix[0],  matrix[1],  matrix[2],  matrix[3], 
        matrix[4],  matrix[5],  matrix[6],  matrix[7], 
        matrix[8],  matrix[9],  matrix[10], matrix[11], 
        matrix[12], matrix[13], matrix[14], matrix[15]);

    for (auto& mesh : model.meshes) {
        for (auto& primitive : mesh.primitives) {
            size_t currentPointsCount = points.size();

            for (auto& attribute : primitive.attributes) 
            {
                if (attribute.first.compare("POSITION") == 0) {
                    auto& accessor = model.accessors[attribute.second];

                    uint32_t bufferViewID = accessor.bufferView;
                    auto& bufferView = model.bufferViews[bufferViewID];

                    uint32_t bufferID = bufferView.buffer;
                    auto& buffer = model.buffers[bufferID].data;

                    uint32_t primPointCount = accessor.count;
                    uint32_t bufferOffset = accessor.byteOffset + bufferView.byteOffset;
                    uint32_t stride = accessor.ByteStride(bufferView);

                    //resize points if we have not
                    if (currentPointsCount == points.size()) {
                        points.resize(currentPointsCount + primPointCount);
                    }

                    for (size_t i = 0; i < primPointCount; i++) {
                        glm::vec4 fColor = glm::vec4(0,0,0,1);
                        memcpy(&fColor, &buffer[bufferOffset + i * stride], stride);

                        points[currentPointsCount + i].position = modelMatrix * fColor;
                    }
                }

                if (attribute.first.compare(0, 5, "COLOR") == 0) {
                    auto& accessor = model.accessors[attribute.second];

                    uint32_t bufferViewID = accessor.bufferView;
                    auto& bufferView = model.bufferViews[bufferViewID];

                    uint32_t bufferID = bufferView.buffer;
                    auto& buffer = model.buffers[bufferID].data;

                    uint32_t pointCount = accessor.count;
                    uint32_t bufferOffset = accessor.byteOffset + bufferView.byteOffset;
                    uint32_t stride = accessor.ByteStride(bufferView);

                    // resize points if we have not
                    if (currentPointsCount == points.size()) {
                        points.resize(currentPointsCount + pointCount);
                    }

                    for (size_t i = 0; i < pointCount; i++) {
                        glm::vec4 fColor;
                        memcpy(&fColor, &buffer[bufferOffset + i * stride], stride);

                        points[currentPointsCount + i].r = fColor.r * 255;
                        points[currentPointsCount + i].g = fColor.g * 255;
                        points[currentPointsCount + i].b = fColor.b * 255;
                        points[currentPointsCount + i].a = 255;
                    }
                }
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
    while (iterator != points.end()) {
        auto& batch = batches.emplace_back();

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