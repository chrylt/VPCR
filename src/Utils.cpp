#include "Utils.h"

#define TINYGLTF_IMPLEMENTATION
#include <tinygltf/tiny_gltf.h>

#include <execution>
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

                if (accessorName == "POSITION") {
                    for (std::uint32_t i = 0; i < primPointCount; ++i) {
                        memcpy(&points[currentPointCount + i].position, &buffer[bufferOffset + i * stride], stride);
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

bmp::uint1024_t MortonIndex768(const Point p, const float quantizationFactor)
{
    using namespace bmp::literals;

    bmp::int256_t ix, iy, iz;
    ix.assign(static_cast<double>(p.position.x) / quantizationFactor);
    iy.assign(static_cast<double>(p.position.y) / quantizationFactor);
    iz.assign(static_cast<double>(p.position.z) / quantizationFactor);

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

BatchedPointCloud LoadScene(std::string_view scene)
{
    auto points = LoadScenePoints(scene);

    // Get smallest distance between points.
    float smallestDistance = 1.0f;
    for (std::uint64_t i = 0; i < points.size() - 1; ++i) {
        smallestDistance = glm::min(smallestDistance, glm::distance(points[i].position, points[i + 1].position));
    }
    smallestDistance *= 10.0f;

    for (auto& point : points) {
        point.mortonCode = MortonIndex768(point, smallestDistance);
    }
    std::sort(std::execution::par_unseq, points.begin(), points.end());

    auto batch = Batch(0U, 0ULL, std::span(points), CreateInitializerBox());
    BatchedPointCloud batched{std::move(points), std::move(batch.Subdivide())};
    return batched;
}

bool Point::operator<(const Point& q) const { return mortonCode < q.mortonCode; }

Batch::Batch(const std::uint32_t iteration, const bmp::uint1024_t mortonCode, const std::span<Point> inPoints,
             const AABB box, const bool isLeaf)
{
    // we only support tree depth up to 256
    if (iteration > 256) {
        throw std::runtime_error("Tree exceded maximum supported depth");
    }

    aabb = box;
    points = inPoints;

    id.leaf = isLeaf;
    id.iteration = iteration;
    id.mortonCode = mortonCode;
}

std::vector<Batch> Batch::Subdivide() const
{
    std::vector<Batch> result;

    if (points.size() < MaxBatchSize) {
        result.emplace_back(id.iteration, id.mortonCode, points, aabb, true);
    } else {
        std::vector<Batch> children;

        std::uint32_t count = 0;
        std::uint32_t offset = 0;
        AABB box = CreateInitializerBox();
        auto prevPointNodeID = MortonToNodeID(points.begin()->mortonCode);

        // splitting points into octree nodes
        // points_ is already sorted by morton code so points related to the same node are in a contiguous range.
        // Just find the range and set the span
        for (auto& point : points) {
            const auto currPointNodeID = MortonToNodeID(point.mortonCode);

            // if NodeID changed
            if (prevPointNodeID != currPointNodeID) {
                children.emplace_back(id.iteration + 1, NodeIDToMorton(prevPointNodeID), points.subspan(offset, count),
                                      box);

                offset += count;
                count = 0;
                box = CreateInitializerBox();
                prevPointNodeID = currPointNodeID;
            }
            ++count;

            box.minV = glm::min(box.minV, point.position);
            box.maxV = glm::max(box.maxV, point.position);
        }
        // check if last node has elements and add it
        if (count != 0) {
            children.emplace_back(id.iteration + 1, NodeIDToMorton(prevPointNodeID), points.subspan(offset, count),
                                  box);
        }

        // combine subdivide results
        for (auto& child : children) {
            const auto res = child.Subdivide();
            result.insert(result.end(), res.begin(), res.end());
        }
    }

    return result;
}

std::uint32_t Batch::MortonToNodeID(const bmp::uint1024_t mortonCode) const
{
    // NodeID is a three bit number taken of the morton code according to node level in the tree.
    // iteration(Node Level) 0 means the root Node. MortonCode used is a 768 bit mortoncode. Mask with 0b0111.
    return ((mortonCode >> (765U - id.iteration * 3U)) & 0x7U).convert_to<uint32_t>();
}

bmp::uint1024_t Batch::NodeIDToMorton(const std::uint32_t nodeID) const
{
    // MortonCode limit used by the tree is 768 bits.
    // To get the Morton Code: shift the nodeID to correct iteartion bits and combine with parents Morton code.
    return (static_cast<bmp::uint1024_t>(nodeID) << (765U - id.iteration * 3U)) | id.mortonCode;
}