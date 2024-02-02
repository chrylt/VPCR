#include <tga/tga.hpp>
#include <tga/tga_utils.hpp>
#include <tga/tga_vulkan/tga_vulkan.hpp>

#include "Config.h"
#include "IPipeline.h"

class PipelineFactory final {
public:
    using Resources = std::span<const std::variant<tga::Buffer, tga::Texture, tga::ext::TopLevelAccelerationStructure>>;

    std::unique_ptr<IPipeline> CreateBasicPipeline(const Config& config, tga::Interface& backend,
                                                   const tga::Window& window, Resources resources,
                                                   std::uint32_t batchCount);
    std::unique_ptr<IPipeline> CreateTPAAPipeline(const Config& config, tga::Interface& backend,
                                                  const tga::Window& window, Resources resources,
                                                  std::uint32_t batchCount);
    std::unique_ptr<IPipeline> CreateOPDAAPipeline(const Config& config, tga::Interface& backend,
                                                   const tga::Window& window, Resources resources,
                                                   std::uint32_t batchCount);
    std::unique_ptr<IPipeline> CreateTPDAAPipeline(const Config& config, tga::Interface& backend,
                                                   const tga::Window& window, Resources resources,
                                                   std::uint32_t batchCount);

private:
    // TODO: resources shared by pipelines
};