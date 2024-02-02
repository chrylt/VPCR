#include "PipelineFactory.h"

#include "BasicPipeline.h"
#include "OPDAAPipeline.h"
#include "TPAAPipeline.h"
#include "TPDAAPipeline.h"

std::unique_ptr<IPipeline> PipelineFactory::CreateBasicPipeline(const Config& config, tga::Interface& backend,
                                                                const tga::Window& window, const Resources resources,
                                                                const std::uint32_t batchCount)
{
    return std::make_unique<BasicPipeline>(config, backend, window, resources, batchCount);
}

std::unique_ptr<IPipeline> PipelineFactory::CreateTPAAPipeline(const Config& config, tga::Interface& backend,
                                                               const tga::Window& window, const Resources resources,
                                                               const std::uint32_t batchCount)
{
    return std::make_unique<TPAAPipeline>(config, backend, window, resources, batchCount);
}

std::unique_ptr<IPipeline> PipelineFactory::CreateOPDAAPipeline(const Config& config, tga::Interface& backend,
                                                                const tga::Window& window, const Resources resources,
                                                                const std::uint32_t batchCount)
{
    return std::make_unique<OPDAAPipeline>(config, backend, window, resources, batchCount);
}

std::unique_ptr<IPipeline> PipelineFactory::CreateTPDAAPipeline(const Config& config, tga::Interface& backend,
                                                                const tga::Window& window, Resources resources,
                                                                std::uint32_t batchCount)
{
    return std::make_unique<TPDAAPipeline>(config, backend, window, resources, batchCount);
}
