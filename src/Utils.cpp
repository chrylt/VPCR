#include "Utils.h"

namespace
{
std::vector<Point> LoadScenePoints(const std::string_view scene)
{
    // TODO @J3oss
    // File loading

    // Add some dummy points until we can load from file
    const std::vector<Point> pointCube{{{-1, -1, -1}, 0, 0, 0, 255},   {{-1, -1, 1}, 0, 0, 255, 255},
                                       {{-1, 1, -1}, 0, 255, 0, 255},  {{-1, 1, 1}, 0, 255, 255, 255},
                                       {{1, -1, -1}, 255, 0, 0, 255},  {{1, -1, 1}, 255, 0, 255, 255},
                                       {{1, 1, -1}, 255, 255, 0, 255}, {{1, 1, 1}, 255, 255, 255, 255}};

    return pointCube;
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