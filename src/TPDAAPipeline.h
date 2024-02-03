#include <tga/tga.hpp>
#include <tga/tga_utils.hpp>
#include <tga/tga_vulkan/tga_vulkan.hpp>

#include "Config.h"
#include "IPipeline.h"

class TGAComputePass;
class TGAQuadPass;

class TPDAAPipeline : public IPipeline {
public:
    using Resources = std::span<const std::variant<tga::Buffer, tga::Texture, tga::ext::TopLevelAccelerationStructure>>;

    TPDAAPipeline(const Config& config, tga::Interface& backend, const tga::Window& window, Resources resources,
                  std::uint32_t batchCount);

    void Execute(std::uint32_t frameIndex, std::span<const UploadData *const> uploads,
                 std::span<DownloadData *const> downloads) override;

    ~TPDAAPipeline();

private:
    struct Pipeline {
        tga::Buffer batchList;
        tga::Buffer renderTarget;
        tga::Texture depthBuffer;
        std::unique_ptr<TGAComputePass> clearPass;
        std::unique_ptr<TGAComputePass> lodPass;
        std::unique_ptr<TGAComputePass> projectionDepthPass;
        std::unique_ptr<TGAComputePass> projectionColorPass;
        std::unique_ptr<TGAQuadPass> displayPass;
        tga::CommandBuffer commandBuffer;
    };

    void CreateBatchList();
    void CreateRenderTarget();
    void CreateDepthBuffer();

    void CreateClearPass(Resources resources);
    void CreateLODPass(Resources resources);
    void CreateProjectionDepthPass(Resources resources);
    void CreateProjectionColorPass(Resources resources);
    void CreateDisplayPass(Resources resources);

    const Config& config_;
    tga::Interface& backend_;
    const tga::Window& window_;

    std::uint32_t batchCount_;

    // One pipeline per frame buffer
    std::vector<Pipeline> pipelines_;
};