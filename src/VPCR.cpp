#include "VPCR.h"

#include <chrono>

#include "TGACamera.h"
#include "TGAGpuPass.h"
#include "TGAPointCloudAcceleration.h"
#include "Utils.h"

namespace
{
// Should match compute shaders
constexpr std::uint32_t ComputeLaneCount = 1024;

class VPCRImpl final : public VPCR {
public:
    explicit VPCRImpl(Config config);

    void Run() override;

private:
    struct UserInputCache {
        glm::i32vec2 mousePosition;
        float jaw;
        float pitch;
    };

    struct Pipeline {
        tga::Buffer batchList;
        tga::Buffer statistics;
        tga::StagingBuffer statisticsDownload;
        tga::Texture renderTarget;
        tga::Texture depthBuffer;
        std::unique_ptr<TGAComputePass> clearPass;
        std::unique_ptr<TGAComputePass> lodPass;
        std::unique_ptr<TGAComputePass> projectionPass;
        std::unique_ptr<TGAQuadPass> displayPass;
        tga::CommandBuffer commandBuffer;
    };

    struct PipelineTwoPassAntiAliasing {
        tga::Buffer batchList;
        tga::Buffer statistics;
        tga::StagingBuffer statisticsDownload;
        tga::Buffer renderTarget;
        tga::Texture depthBuffer;
        std::unique_ptr<TGAComputePass> clearPass;
        std::unique_ptr<TGAComputePass> lodPass;
        std::unique_ptr<TGAComputePass> projectionDepthPass;
        std::unique_ptr<TGAComputePass> projectionColorPass;
        std::unique_ptr<TGAQuadPass> displayPass;
        tga::CommandBuffer commandBuffer;
    };

    struct PipelineOnePassDensityAntiAliasing {
        tga::Buffer batchList;
        tga::Buffer statistics;
        tga::StagingBuffer statisticsDownload;
        tga::Buffer depthBuffer;
        std::unique_ptr<TGAComputePass> clearPass;
        std::unique_ptr<TGAComputePass> lodPass;
        std::unique_ptr<TGAComputePass> projectionPass;
        std::unique_ptr<TGAQuadPass> displayPass;
        tga::CommandBuffer commandBuffer;
    };

    struct DynamicConst {
        // Misc information we need on gpu
        std::uint32_t totalBatchCount;
        std::uint32_t depthDiscSteps;
        float lodExtend;
    };

    struct Statistics {
        std::uint32_t drawnBatches;
    };

    enum AntiAliasingMode { kOff, kTwoPass, kDensityOnePass, AA_MODE_COUNT /* always keep this as the last element */ };
    const char *AntiAliasingModeStrings[3] = {"OFF", "TWO PASS", "DENSITY"};

    void OnUpdate(std::uint32_t frameIndex);
    void OnRender(std::uint32_t frameIndex);
    void OnRenderTPAA(std::uint32_t frameIndex);
    void OnRenderOPDAA(std::uint32_t frameIndex);

    void CreateDynamicConst();
    void CreateBatchList();
    void CreateRenderTarget();
    void CreateRenderTargetTPAA();
    void CreateDepthBuffer();
    void CreateDepthBufferTPAA();
    void CreateDepthBufferOPDAA();
    void CreateStatisticsReadbackBuffer();

    void CreateClearPass();
    void CreateClearPassTPAA();
    void CreateClearPassOPDAA();
    void CreateLODPass();
    void CreateProjectionPass();
    void CreateProjectionPassDepthTPAA();
    void CreateProjectionPassColorTPAA();
    void CreateProjectionPassOPDAA();
    void CreateDisplayPass();
    void CreateDisplayPassTPAA();
    void CreateDisplayPassOPDAA();

    Config config_;
    tga::Interface backend_;

    tga::Window window_;

    std::chrono::time_point<std::chrono::high_resolution_clock> lastFrameTimeStamp_;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastTitleUpdate_;
    std::uint32_t frameCounter_ = 0;

    std::optional<UserInputCache> userInputCache_;

    std::unique_ptr<TGACamera> camera_;
    tga::Buffer dynamicConst_;

    AntiAliasingMode currAntiAliasingMode = kOff;

    // One pipeline per frame buffer
    std::vector<Pipeline> pipelines_;
    std::vector<PipelineTwoPassAntiAliasing> pipelinesTPAA_;
    std::vector<PipelineOnePassDensityAntiAliasing> pipelinesOPDAA_;

    std::unique_ptr<TGAPointCloudAcceleration> pointCloudAcceleration_;

};

}  // namespace

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////IMPLEMENTATION/////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace
{

VPCRImpl::VPCRImpl(Config config) : config_(std::move(config))
{
    const auto scenePath = config_.Get<std::string>("scenePath");
    if (!scenePath.has_value()) {
        throw std::runtime_error("Missing scene!");
    }

    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();

    // Create Window
    window_ = backend_.createWindow({res[0], res[1]});
    backend_.setWindowTitle(window_, "TGA Vulkan RayTracer");
    pipelines_.resize(backend_.backbufferCount(window_));
    pipelinesTPAA_.resize(backend_.backbufferCount(window_));
    pipelinesOPDAA_.resize(backend_.backbufferCount(window_));

    // Create Resources
    pointCloudAcceleration_ = std::make_unique<TGAPointCloudAcceleration>(backend_, scenePath.value());
    camera_ = std::make_unique<TGACamera>(backend_, res[0], res[1], glm::vec3(0, 0, 3), -90.f);
    CreateDynamicConst();
    CreateBatchList();
    CreateRenderTarget();
    CreateRenderTargetTPAA();
    CreateDepthBuffer();
    CreateDepthBufferTPAA();
    CreateDepthBufferOPDAA();
    CreateStatisticsReadbackBuffer();

    // Create Passes
    CreateClearPass();
    CreateClearPassTPAA();
    CreateClearPassOPDAA();
    CreateLODPass();
    CreateProjectionPass();
    CreateProjectionPassDepthTPAA();
    CreateProjectionPassColorTPAA();
    CreateProjectionPassOPDAA();
    CreateDisplayPass();
    CreateDisplayPassTPAA();
    CreateDisplayPassOPDAA();
}

void VPCRImpl::Run()
{
    config_.Set("ShouldClose", false);
    while (!config_.Get<bool>("ShouldClose").value_or(true)) {
        const auto nextFrame = backend_.nextFrame(window_);
        OnUpdate(nextFrame);

        switch (currAntiAliasingMode) {
            case kOff: OnRender(nextFrame); break;
            case kTwoPass: OnRenderTPAA(nextFrame); break;
            case kDensityOnePass: OnRenderOPDAA(nextFrame); break;
            default: OnRender(nextFrame);
        }
    }
}

void VPCRImpl::OnUpdate(std::uint32_t frameIndex)
{
    const auto currentTime = std::chrono::high_resolution_clock::now();
    const auto timeSinceLastUpdate = (currentTime - lastFrameTimeStamp_).count() / 1000000000.f;  // In seconds
    lastFrameTimeStamp_ = currentTime;

    ++frameCounter_;

    // Toggle Features bases on user input
    {

        static auto start = std::chrono::high_resolution_clock::now();
        constexpr auto keyPressSpacing = std::chrono::seconds(1);

        if (backend_.keyDown(window_, tga::Key::J)) {
            const auto now = std::chrono::high_resolution_clock::now();
            const auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start);

            if (duration >= keyPressSpacing) {
                currAntiAliasingMode = static_cast<AntiAliasingMode>((currAntiAliasingMode + 1) % AA_MODE_COUNT);
                start = std::chrono::high_resolution_clock::now();
            }
        }
    }

    // Print FPS and statistics on window title
    {
        const auto *statistics =
            static_cast<const Statistics *>(backend_.getMapping(pipelines_[frameIndex].statisticsDownload));
        if ((currentTime - lastTitleUpdate_).count() / 1000000000.f >= 1.f) {
            lastTitleUpdate_ = std::chrono::high_resolution_clock::now();

            backend_.setWindowTitle(window_,
                                    "VPCR, Frame rate: " + std::to_string(frameCounter_) +
                                        " FPS, Drawn batches: " + std::to_string(statistics->drawnBatches) + "/" +
                                        std::to_string(pointCloudAcceleration_->GetBatchCount()) +
                                        ", Anti-Aliasing-Mode: " + AntiAliasingModeStrings[currAntiAliasingMode]);
            frameCounter_ = 0;
        }
    }

    // Update camera based on user input
    {
        const auto moveSpeed = config_.Get<float>("camera.moveSpeed").value_or(0) * timeSinceLastUpdate;
        const auto lookSpeed = config_.Get<float>("camera.lookSpeed").value_or(0);

        auto position = camera_->GetPosition();
        const auto& direction = camera_->GetDirection();
        auto jaw = camera_->GetJaw();
        auto pitch = camera_->GetPitch();

        // Collect user input
        if (backend_.keyDown(window_, tga::Key::W)) {
            position += direction * moveSpeed;
        } else if (backend_.keyDown(window_, tga::Key::S)) {
            position -= direction * moveSpeed;
        }

        if (backend_.keyDown(window_, tga::Key::D)) {
            position += glm::normalize(glm::cross(direction, glm::vec3(0, 1, 0))) * moveSpeed;
        } else if (backend_.keyDown(window_, tga::Key::A)) {
            position -= glm::normalize(glm::cross(direction, glm::vec3(0, 1, 0))) * moveSpeed;
        }

        if (backend_.keyDown(window_, tga::Key::Space)) {
            position += glm::vec3(0, 1, 0) * moveSpeed;
        } else if (backend_.keyDown(window_, tga::Key::Shift_Left)) {
            position -= glm::vec3(0, 1, 0) * moveSpeed;
        }

        if (backend_.keyDown(window_, tga::Key::MouseRight)) {
            const auto& [mouseX, mouseY] = backend_.mousePosition(window_);
            if (!userInputCache_.has_value()) {
                userInputCache_ = UserInputCache{};
                userInputCache_->mousePosition = glm::i32vec2(-mouseX, mouseY);
                userInputCache_->jaw = jaw;
                userInputCache_->pitch = pitch;
            }

            jaw = userInputCache_->jaw + (userInputCache_->mousePosition.x + mouseX) * lookSpeed;
            pitch = userInputCache_->pitch + (userInputCache_->mousePosition.y - mouseY) * lookSpeed;
        } else {
            userInputCache_ = std::nullopt;
        }

        camera_->Update(position, jaw, pitch);
    }
}

void VPCRImpl::OnRender(std::uint32_t frameIndex)
{
    auto& commandBuffer = pipelines_[frameIndex].commandBuffer;
    const auto& clearPass = pipelines_[frameIndex].clearPass;
    const auto& lodPass = pipelines_[frameIndex].lodPass;
    const auto& projectionPass = pipelines_[frameIndex].projectionPass;
    const auto& displayPass = pipelines_[frameIndex].displayPass;

    auto commandRecorder = tga::CommandRecorder{backend_, commandBuffer};

    // Readback
    commandRecorder.bufferDownload(pipelines_[frameIndex].statistics, pipelines_[frameIndex].statisticsDownload,
                                   sizeof(Statistics));

    // Upload
    camera_->Upload(commandRecorder);
    // Set the batch count to 0 as the LOD pass will determine the batches to be rendered
    constexpr std::uint32_t zero = 0;
    commandRecorder.inlineBufferUpdate(pipelines_[frameIndex].batchList, &zero, sizeof(zero));
    commandRecorder.barrier(tga::PipelineStage::Transfer, tga::PipelineStage::ComputeShader);

    // Collect Execution Commands
    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();
    const auto batchCount = pointCloudAcceleration_->GetBatchCount();
    clearPass->Execute(commandRecorder, res[0] / ComputeLaneCount + 1, res[1]);
    lodPass->Execute(commandRecorder, batchCount);
    commandRecorder.barrier(tga::PipelineStage::ComputeShader, tga::PipelineStage::ComputeShader);
    projectionPass->Execute(commandRecorder, batchCount);
    commandRecorder.barrier(tga::PipelineStage::ComputeShader, tga::PipelineStage::FragmentShader);
    displayPass->Execute(commandRecorder, frameIndex);

    // Execute
    commandBuffer = commandRecorder.endRecording();
    backend_.execute(commandBuffer);

    // Present
    backend_.present(window_, frameIndex);

    config_.Set("ShouldClose", backend_.windowShouldClose(window_));
}

void VPCRImpl::OnRenderTPAA(std::uint32_t frameIndex)
{
    auto& commandBuffer = pipelinesTPAA_[frameIndex].commandBuffer;
    const auto& clearPass = pipelinesTPAA_[frameIndex].clearPass;
    const auto& lodPass = pipelinesTPAA_[frameIndex].lodPass;
    const auto& projectionDepthPass = pipelinesTPAA_[frameIndex].projectionDepthPass;
    const auto& projectionColorPass = pipelinesTPAA_[frameIndex].projectionColorPass;
    const auto& displayPass = pipelinesTPAA_[frameIndex].displayPass;

    auto commandRecorder = tga::CommandRecorder{backend_, commandBuffer};

    // Readback
    commandRecorder.bufferDownload(pipelinesTPAA_[frameIndex].statistics, pipelinesTPAA_[frameIndex].statisticsDownload,
                                   sizeof(Statistics));

    // Upload
    camera_->Upload(commandRecorder);
    // Set the batch count to 0 as the LOD pass will determine the batches to be rendered
    constexpr std::uint32_t zero = 0;
    commandRecorder.inlineBufferUpdate(pipelinesTPAA_[frameIndex].batchList, &zero, sizeof(zero));
    commandRecorder.barrier(tga::PipelineStage::Transfer, tga::PipelineStage::ComputeShader);

    // Collect Execution Commands
    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();
    const auto batchCount = pointCloudAcceleration_->GetBatchCount();
    clearPass->Execute(commandRecorder, res[0] / ComputeLaneCount + 1, res[1]);
    lodPass->Execute(commandRecorder, batchCount);
    commandRecorder.barrier(tga::PipelineStage::ComputeShader, tga::PipelineStage::ComputeShader);
    projectionDepthPass->Execute(commandRecorder, batchCount);
    commandRecorder.barrier(tga::PipelineStage::ComputeShader, tga::PipelineStage::ComputeShader);
    projectionColorPass->Execute(commandRecorder, batchCount);
    commandRecorder.barrier(tga::PipelineStage::ComputeShader, tga::PipelineStage::FragmentShader);
    displayPass->Execute(commandRecorder, frameIndex);

    // Execute
    commandBuffer = commandRecorder.endRecording();
    backend_.execute(commandBuffer);

    // Present
    backend_.present(window_, frameIndex);

    config_.Set("ShouldClose", backend_.windowShouldClose(window_));
}

void VPCRImpl::OnRenderOPDAA(std::uint32_t frameIndex)
{
    auto& commandBuffer = pipelinesOPDAA_[frameIndex].commandBuffer;
    const auto& clearPass = pipelinesOPDAA_[frameIndex].clearPass;
    const auto& lodPass = pipelinesOPDAA_[frameIndex].lodPass;
    const auto& projectionPass = pipelinesOPDAA_[frameIndex].projectionPass;
    const auto& displayPass = pipelinesOPDAA_[frameIndex].displayPass;

    auto commandRecorder = tga::CommandRecorder{backend_, commandBuffer};

    // Readback
    commandRecorder.bufferDownload(pipelinesOPDAA_[frameIndex].statistics,
                                   pipelinesOPDAA_[frameIndex].statisticsDownload, sizeof(Statistics));

    // Upload
    camera_->Upload(commandRecorder);
    // Set the batch count to 0 as the LOD pass will determine the batches to be rendered
    constexpr std::uint32_t zero = 0;
    commandRecorder.inlineBufferUpdate(pipelinesOPDAA_[frameIndex].batchList, &zero, sizeof(zero));
    commandRecorder.barrier(tga::PipelineStage::Transfer, tga::PipelineStage::ComputeShader);

    // Collect Execution Commands
    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();
    const auto batchCount = pointCloudAcceleration_->GetBatchCount();
    clearPass->Execute(commandRecorder, res[0] / ComputeLaneCount + 1, res[1]);
    lodPass->Execute(commandRecorder, batchCount);
    commandRecorder.barrier(tga::PipelineStage::ComputeShader, tga::PipelineStage::ComputeShader);
    projectionPass->Execute(commandRecorder, batchCount);
    commandRecorder.barrier(tga::PipelineStage::ComputeShader, tga::PipelineStage::FragmentShader);
    displayPass->Execute(commandRecorder, frameIndex);

    // Execute
    commandBuffer = commandRecorder.endRecording();
    backend_.execute(commandBuffer);

    // Present
    backend_.present(window_, frameIndex);

    config_.Set("ShouldClose", backend_.windowShouldClose(window_));
}

void VPCRImpl::CreateDynamicConst()
{
    // We are using the cubic root of the MaxBatchSize as a heuristic for the size of a batch before it loses precision
    // Generally we would like to know the area in pixels of a projected batch that can be coverd by its content before
    // leaving holes
    const DynamicConst dynamicConst{pointCloudAcceleration_->GetBatchCount(), 1u << 31, std::cbrtf(MaxBatchSize)};

    tga::StagingBufferInfo stagingInfo{sizeof(DynamicConst), reinterpret_cast<const std::uint8_t *>(&dynamicConst)};
    const auto staging = backend_.createStagingBuffer(stagingInfo);
    const tga::BufferInfo info{tga::BufferUsage::uniform, sizeof(DynamicConst), staging};
    dynamicConst_ = backend_.createBuffer(info);

    backend_.free(staging);
}

void VPCRImpl::CreateBatchList()
{
    // The first entry of the batch list is reserved for the batch count in the list, all other are indices into the
    // batch buffer
    const auto batchCount = pointCloudAcceleration_->GetBatchCount();
    for (auto& pipeline : pipelines_) {
        std::vector<std::uint32_t> batchList(batchCount + 1);
        batchList[0] = batchCount;
        for (std::uint32_t i = 0; i < batchCount; ++i) {
            batchList[i + 1] = i;
        }

        tga::StagingBufferInfo stagingInfo{(batchCount + 1) * sizeof(std::uint32_t),
                                           reinterpret_cast<const std::uint8_t *>(batchList.data())};
        const auto staging = backend_.createStagingBuffer(stagingInfo);
        const tga::BufferInfo info{tga::BufferUsage::storage, (batchCount + 1) * sizeof(std::uint32_t), staging};
        pipeline.batchList = backend_.createBuffer(info);

        backend_.free(staging);
    }
    for (auto& pipeline : pipelinesTPAA_) {
        std::vector<std::uint32_t> batchList(batchCount + 1);
        batchList[0] = batchCount;
        for (std::uint32_t i = 0; i < batchCount; ++i) {
            batchList[i + 1] = i;
        }

        tga::StagingBufferInfo stagingInfo{(batchCount + 1) * sizeof(std::uint32_t),
                                           reinterpret_cast<const std::uint8_t *>(batchList.data())};
        const auto staging = backend_.createStagingBuffer(stagingInfo);
        const tga::BufferInfo info{tga::BufferUsage::storage, (batchCount + 1) * sizeof(std::uint32_t), staging};
        pipeline.batchList = backend_.createBuffer(info);

        backend_.free(staging);
    }
    for (auto& pipeline : pipelinesOPDAA_) {
        std::vector<std::uint32_t> batchList(batchCount + 1);
        batchList[0] = batchCount;
        for (std::uint32_t i = 0; i < batchCount; ++i) {
            batchList[i + 1] = i;
        }

        tga::StagingBufferInfo stagingInfo{(batchCount + 1) * sizeof(std::uint32_t),
                                           reinterpret_cast<const std::uint8_t *>(batchList.data())};
        const auto staging = backend_.createStagingBuffer(stagingInfo);
        const tga::BufferInfo info{tga::BufferUsage::storage, (batchCount + 1) * sizeof(std::uint32_t), staging};
        pipeline.batchList = backend_.createBuffer(info);

        backend_.free(staging);
    }
}

void VPCRImpl::CreateRenderTarget()
{
    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();
    for (auto& pipeline : pipelines_) {
        const tga::TextureInfo textureInfo(res[0], res[1], tga::Format::r32_uint);
        pipeline.renderTarget = backend_.createTexture(textureInfo);
    }
}

void VPCRImpl::CreateRenderTargetTPAA()
{
    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();
    for (auto& pipeline : pipelinesTPAA_) {
        const glm::vec2 resolution = camera_->GetResolution();
        const tga::BufferInfo info{tga::BufferUsage::storage,
                                   static_cast<size_t>(resolution.x * resolution.y) * sizeof(std::uint64_t)};
        pipeline.renderTarget = backend_.createBuffer(info);
    }
}

void VPCRImpl::CreateDepthBuffer()
{
    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();
    for (auto& pipeline : pipelines_) {
        const tga::TextureInfo textureInfo(res[0], res[1], tga::Format::r32_uint);
        pipeline.depthBuffer = backend_.createTexture(textureInfo);
    }
}

void VPCRImpl::CreateDepthBufferTPAA()
{
    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();
    for (auto& pipeline : pipelinesTPAA_) {
        const tga::TextureInfo textureInfo(res[0], res[1], tga::Format::r32_uint);
        pipeline.depthBuffer = backend_.createTexture(textureInfo);
    }
}

void VPCRImpl::CreateDepthBufferOPDAA()
{
    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();
    for (auto& pipeline : pipelinesOPDAA_) {
        const glm::vec2 resolution = camera_->GetResolution();
        const tga::BufferInfo info{tga::BufferUsage::storage,
                                   static_cast<size_t>(resolution.x * resolution.y) * sizeof(Histogram)};
        pipeline.depthBuffer = backend_.createBuffer(info);
    }
}

void VPCRImpl::CreateStatisticsReadbackBuffer()
{
    for (auto& pipeline : pipelines_) {
        tga::StagingBufferInfo stagingInfo{sizeof(Statistics)};
        pipeline.statisticsDownload = backend_.createStagingBuffer(stagingInfo);
        const tga::BufferInfo info{tga::BufferUsage::storage, sizeof(Statistics), pipeline.statisticsDownload};
        pipeline.statistics = backend_.createBuffer(info);
    }
    for (auto& pipeline : pipelinesTPAA_) {
        tga::StagingBufferInfo stagingInfo{sizeof(Statistics)};
        pipeline.statisticsDownload = backend_.createStagingBuffer(stagingInfo);
        const tga::BufferInfo info{tga::BufferUsage::storage, sizeof(Statistics), pipeline.statisticsDownload};
        pipeline.statistics = backend_.createBuffer(info);
    }
    for (auto& pipeline : pipelinesOPDAA_) {
        tga::StagingBufferInfo stagingInfo{sizeof(Statistics)};
        pipeline.statisticsDownload = backend_.createStagingBuffer(stagingInfo);
        const tga::BufferInfo info{tga::BufferUsage::storage, sizeof(Statistics), pipeline.statisticsDownload};
        pipeline.statistics = backend_.createBuffer(info);
    }
}

void VPCRImpl::CreateClearPass()
{
    for (auto& pipeline : pipelines_) {
        auto& clearPass = pipeline.clearPass;

        // Use utility function to load Shader from File
        const auto computeShader = tga::loadShader("../shaders/clear_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}, {tga::BindingType::uniformBuffer}}},
                                            // Set = 1: RenderTarget, DepthBuffer
                                            {{{tga::BindingType::storageImage}, {tga::BindingType::storageImage}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        clearPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        clearPass->BindInput(camera_->GetBuffer(), 0, 0);
        clearPass->BindInput(dynamicConst_, 0, 1);
        clearPass->BindInput(pipeline.renderTarget, 1, 0);
        clearPass->BindInput(pipeline.depthBuffer, 1, 1);
    }
}

void VPCRImpl::CreateClearPassTPAA()
{
    for (auto& pipeline : pipelinesTPAA_) {
        auto& clearPass = pipeline.clearPass;

        // Use utility function to load Shader from File
        const auto computeShader = tga::loadShader("../shaders/clearTPAA_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}, {tga::BindingType::uniformBuffer}}},
                                            // Set = 1: RenderTarget, DepthBuffer
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageImage}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        clearPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        clearPass->BindInput(camera_->GetBuffer(), 0, 0);
        clearPass->BindInput(dynamicConst_, 0, 1);
        clearPass->BindInput(pipeline.renderTarget, 1, 0);
        clearPass->BindInput(pipeline.depthBuffer, 1, 1);
    }
}

void VPCRImpl::CreateClearPassOPDAA()
{
    for (auto& pipeline : pipelinesOPDAA_) {
        auto& clearPass = pipeline.clearPass;

        // Use utility function to load Shader from File
        const auto computeShader =
            tga::loadShader("../shaders/clearOPDAA_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}, {tga::BindingType::uniformBuffer}}},
                                            // Set = 1: DepthBuffer
                                            {{{tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        clearPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        clearPass->BindInput(camera_->GetBuffer(), 0, 0);
        clearPass->BindInput(dynamicConst_, 0, 1);
        clearPass->BindInput(pipeline.depthBuffer, 1, 0);
    }
}

void VPCRImpl::CreateLODPass()
{
    for (auto& pipeline : pipelines_) {
        auto& lodPass = pipeline.lodPass;

        const auto computeShader = tga::loadShader("../shaders/lod_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}, {tga::BindingType::uniformBuffer}}},
                                            // Set = 1: Batches, Batch List
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        lodPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        lodPass->BindInput(camera_->GetBuffer(), 0, 0);
        lodPass->BindInput(dynamicConst_, 0, 1);
        lodPass->BindInput(pointCloudAcceleration_->GetBatchesBuffer(), 1, 0);
        lodPass->BindInput(pipeline.batchList, 1, 1);
    }
    for (auto& pipeline : pipelinesTPAA_) {
        auto& lodPass = pipeline.lodPass;

        const auto computeShader = tga::loadShader("../shaders/lod_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}, {tga::BindingType::uniformBuffer}}},
                                            // Set = 1: Batches, Batch List
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        lodPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        lodPass->BindInput(camera_->GetBuffer(), 0, 0);
        lodPass->BindInput(dynamicConst_, 0, 1);
        lodPass->BindInput(pointCloudAcceleration_->GetBatchesBuffer(), 1, 0);
        lodPass->BindInput(pipeline.batchList, 1, 1);
    }
    for (auto& pipeline : pipelinesOPDAA_) {
        auto& lodPass = pipeline.lodPass;

        const auto computeShader = tga::loadShader("../shaders/lod_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}, {tga::BindingType::uniformBuffer}}},
                                            // Set = 1: Batches, Batch List
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        lodPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        lodPass->BindInput(camera_->GetBuffer(), 0, 0);
        lodPass->BindInput(dynamicConst_, 0, 1);
        lodPass->BindInput(pointCloudAcceleration_->GetBatchesBuffer(), 1, 0);
        lodPass->BindInput(pipeline.batchList, 1, 1);
    }
}

void VPCRImpl::CreateProjectionPass()
{
    for (auto& pipeline : pipelines_) {
        auto& projectionPass = pipeline.projectionPass;

        // Use utility function to load Shader from File
        const auto computeShader =
            tga::loadShader("../shaders/projection_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}, {tga::BindingType::uniformBuffer}}},
                                            // Set = 1: Rendertarget, Depthbuffer
                                            {{{tga::BindingType::storageImage}, {tga::BindingType::storageImage}}},
                                            // Set = 2: PointsPositionLowPrecision, PointsPositionMediumPrecision,
                                            // PointsPositionHighPrecision, PointsColor
                                            {{{tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer}}},
                                            // Set = 3: Batches, Batch list
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageBuffer}}},
                                            // Set = 4: Statistics
                                            {{{tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        projectionPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        projectionPass->BindInput(camera_->GetBuffer(), 0, 0);
        projectionPass->BindInput(dynamicConst_, 0, 1);
        projectionPass->BindInput(pipeline.renderTarget, 1, 0);
        projectionPass->BindInput(pipeline.depthBuffer, 1, 1);

        // Point information buffers
        const auto& pointsBufferPack = pointCloudAcceleration_->GetPointsBufferPack();
        projectionPass->BindInput(pointsBufferPack.positionLowPrecision, 2, 0);
        projectionPass->BindInput(pointsBufferPack.positionMediumPrecision, 2, 1);
        projectionPass->BindInput(pointsBufferPack.positionHighPrecision, 2, 2);
        projectionPass->BindInput(pointsBufferPack.colors, 2, 3);

        projectionPass->BindInput(pointCloudAcceleration_->GetBatchesBuffer(), 3, 0);
        projectionPass->BindInput(pipeline.batchList, 3, 1);
        projectionPass->BindInput(pipeline.statistics, 4, 0);
    }
}

void VPCRImpl::CreateProjectionPassDepthTPAA()
{
    for (auto& pipeline : pipelinesTPAA_) {
        auto& projectionDepthPass = pipeline.projectionDepthPass;

        // Use utility function to load Shader from File
        const auto computeShader =
            tga::loadShader("../shaders/projectionDepthTPAA_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}, {tga::BindingType::uniformBuffer}}},
                                            // Set = 1: Rendertarget, Depthbuffer
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageImage}}},
                                            // Set = 2: PointsPositionLowPrecision, PointsPositionMediumPrecision,
                                            // PointsPositionHighPrecision, PointsColor
                                            {{{tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer}}},
                                            // Set = 3: Batches, Batch list
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageBuffer}}},
                                            // Set = 4: Statistics
                                            {{{tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        projectionDepthPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        projectionDepthPass->BindInput(camera_->GetBuffer(), 0, 0);
        projectionDepthPass->BindInput(dynamicConst_, 0, 1);
        projectionDepthPass->BindInput(pipeline.renderTarget, 1, 0);
        projectionDepthPass->BindInput(pipeline.depthBuffer, 1, 1);

        // Point information buffers
        const auto& pointsBufferPack = pointCloudAcceleration_->GetPointsBufferPack();
        projectionDepthPass->BindInput(pointsBufferPack.positionLowPrecision, 2, 0);
        projectionDepthPass->BindInput(pointsBufferPack.positionMediumPrecision, 2, 1);
        projectionDepthPass->BindInput(pointsBufferPack.positionHighPrecision, 2, 2);
        projectionDepthPass->BindInput(pointsBufferPack.colors, 2, 3);

        projectionDepthPass->BindInput(pointCloudAcceleration_->GetBatchesBuffer(), 3, 0);
        projectionDepthPass->BindInput(pipeline.batchList, 3, 1);
        projectionDepthPass->BindInput(pipeline.statistics, 4, 0);
    }
}

void VPCRImpl::CreateProjectionPassColorTPAA()
{
    for (auto& pipeline : pipelinesTPAA_) {
        auto& projectionColorPass = pipeline.projectionColorPass;

        // Use utility function to load Shader from File
        const auto computeShader =
            tga::loadShader("../shaders/projectionColorTPAA_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}, {tga::BindingType::uniformBuffer}}},
                                            // Set = 1: Rendertarget, Depthbuffer
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageImage}}},
                                            // Set = 2: PointsPositionLowPrecision, PointsPositionMediumPrecision,
                                            // PointsPositionHighPrecision, PointsColor
                                            {{{tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer}}},
                                            // Set = 3: Batches, Batch list
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageBuffer}}},
                                            // Set = 4: Statistics
                                            {{{tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        projectionColorPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        projectionColorPass->BindInput(camera_->GetBuffer(), 0, 0);
        projectionColorPass->BindInput(dynamicConst_, 0, 1);
        projectionColorPass->BindInput(pipeline.renderTarget, 1, 0);
        projectionColorPass->BindInput(pipeline.depthBuffer, 1, 1);

        // Point information buffers
        const auto& pointsBufferPack = pointCloudAcceleration_->GetPointsBufferPack();
        projectionColorPass->BindInput(pointsBufferPack.positionLowPrecision, 2, 0);
        projectionColorPass->BindInput(pointsBufferPack.positionMediumPrecision, 2, 1);
        projectionColorPass->BindInput(pointsBufferPack.positionHighPrecision, 2, 2);
        projectionColorPass->BindInput(pointsBufferPack.colors, 2, 3);

        projectionColorPass->BindInput(pointCloudAcceleration_->GetBatchesBuffer(), 3, 0);
        projectionColorPass->BindInput(pipeline.batchList, 3, 1);
        projectionColorPass->BindInput(pipeline.statistics, 4, 0);
    }
}

void VPCRImpl::CreateProjectionPassOPDAA()
{
    for (auto& pipeline : pipelinesOPDAA_) {
        auto& projectionPass = pipeline.projectionPass;

        // Use utility function to load Shader from File
        const auto computeShader =
            tga::loadShader("../shaders/projectionOPDAA_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}, {tga::BindingType::uniformBuffer}}},
                                            // Set = 1: Depthbuffer
                                            {{{tga::BindingType::storageBuffer}}},
                                            // Set = 2: PointsPositionLowPrecision, PointsPositionMediumPrecision,
                                            // PointsPositionHighPrecision, PointsColor
                                            {{{tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer}}},
                                            // Set = 3: Batches, Batch list
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageBuffer}}},
                                            // Set = 4: Statistics
                                            {{{tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        projectionPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        projectionPass->BindInput(camera_->GetBuffer(), 0, 0);
        projectionPass->BindInput(dynamicConst_, 0, 1);
        projectionPass->BindInput(pipeline.depthBuffer, 1, 0);

        // Point information buffers
        const auto& pointsBufferPack = pointCloudAcceleration_->GetPointsBufferPack();
        projectionPass->BindInput(pointsBufferPack.positionLowPrecision, 2, 0);
        projectionPass->BindInput(pointsBufferPack.positionMediumPrecision, 2, 1);
        projectionPass->BindInput(pointsBufferPack.positionHighPrecision, 2, 2);
        projectionPass->BindInput(pointsBufferPack.colors, 2, 3);

        projectionPass->BindInput(pointCloudAcceleration_->GetBatchesBuffer(), 3, 0);
        projectionPass->BindInput(pipeline.batchList, 3, 1);
        projectionPass->BindInput(pipeline.statistics, 4, 0);
    }
}

void VPCRImpl::CreateDisplayPass()
{
    for (auto& pipeline : pipelines_) {
        auto& displayPass = pipeline.displayPass;

        // Use utility function to load Shader from File
        const auto vertexShader =
            tga::loadShader("../shaders/fullScreenTriangle_vert.spv", tga::ShaderType::vertex, backend_);
        const auto fragmentShader =
            tga::loadShader("../shaders/renderTarget_frag.spv", tga::ShaderType::fragment, backend_);

        const tga::InputLayout inputLayout({// Set = 0: RenderTarget
                                            {{{tga::BindingType::storageImage}}}});

        const tga::RenderPassInfo passInfo(vertexShader, fragmentShader, window_, {}, inputLayout,
                                           tga::ClearOperation::none);
        displayPass = std::make_unique<TGAQuadPass>(backend_, passInfo);

        displayPass->BindInput(camera_->GetBuffer(), 0, 0);
        displayPass->BindInput(pipeline.renderTarget, 0);
    }
}

void VPCRImpl::CreateDisplayPassTPAA()
{
    for (auto& pipeline : pipelinesTPAA_) {
        auto& displayPass = pipeline.displayPass;

        // Use utility function to load Shader from File
        const auto vertexShader =
            tga::loadShader("../shaders/fullScreenTriangle_vert.spv", tga::ShaderType::vertex, backend_);
        const auto fragmentShader =
            tga::loadShader("../shaders/renderTargetTPAA_frag.spv", tga::ShaderType::fragment, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}}},
                                            // Set = 1: RenderTarget
                                            {{{tga::BindingType::storageBuffer}}}});

        const tga::RenderPassInfo passInfo(vertexShader, fragmentShader, window_, {}, inputLayout,
                                           tga::ClearOperation::none);
        displayPass = std::make_unique<TGAQuadPass>(backend_, passInfo);

        displayPass->BindInput(camera_->GetBuffer(), 0, 0);
        displayPass->BindInput(pipeline.renderTarget, 1);
    }
}

void VPCRImpl::CreateDisplayPassOPDAA()
{
    for (auto& pipeline : pipelinesOPDAA_) {
        auto& displayPass = pipeline.displayPass;

        // Use utility function to load Shader from File
        const auto vertexShader =
            tga::loadShader("../shaders/fullScreenTriangle_vert.spv", tga::ShaderType::vertex, backend_);
        const auto fragmentShader =
            tga::loadShader("../shaders/renderTargetOPDAA_frag.spv", tga::ShaderType::fragment, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}}},
                                            // Set = 1: RenderTarget
                                            {{{tga::BindingType::storageBuffer}}}});

        const tga::RenderPassInfo passInfo(vertexShader, fragmentShader, window_, {}, inputLayout,
                                           tga::ClearOperation::none);
        displayPass = std::make_unique<TGAQuadPass>(backend_, passInfo);

        displayPass->BindInput(camera_->GetBuffer(), 0, 0);
        displayPass->BindInput(pipeline.depthBuffer, 1);
    }
}

}  // namespace

std::unique_ptr<VPCR> CreateVPCR(Config config) { return std::make_unique<VPCRImpl>(std::move(config)); }