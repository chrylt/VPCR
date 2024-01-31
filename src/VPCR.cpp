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
        tga::Buffer renderTarget;
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
        float depthStepSize;
        float lodExtend;
    };

    struct Statistics {
        std::uint32_t drawnBatches;
    };

    void OnUpdate(std::uint32_t frameIndex);
    void OnRender(std::uint32_t frameIndex);

    void CreateDynamicConst();
    void CreateBatchList();
    void CreateRenderTarget();
    void CreateDepthBuffer();
    void CreateStatisticsReadbackBuffer();

    void CreateClearPass();
    void CreateLODPass();
    void CreateProjectionPass();
    void CreateDisplayPass();

    Config config_;
    tga::Interface backend_;

    tga::Window window_;

    std::chrono::time_point<std::chrono::high_resolution_clock> lastFrameTimeStamp_;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastTitleUpdate_;
    std::uint32_t frameCounter_ = 0;

    std::optional<UserInputCache> userInputCache_;

    std::unique_ptr<TGACamera> camera_;
    tga::Buffer dynamicConst_;

    // One pipeline per frame buffer
    std::vector<Pipeline> pipelines_;

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

    // Create Resources
    pointCloudAcceleration_ = std::make_unique<TGAPointCloudAcceleration>(backend_, scenePath.value());
    camera_ = std::make_unique<TGACamera>(backend_, res[0], res[1], glm::vec3(0, 0, 3), -90.f);
    CreateDynamicConst();
    CreateBatchList();
    CreateRenderTarget();
    CreateDepthBuffer();
    CreateStatisticsReadbackBuffer();

    // Create Passes
    CreateClearPass();
    CreateLODPass();
    CreateProjectionPass();
    CreateDisplayPass();
}

void VPCRImpl::Run()
{
    config_.Set("ShouldClose", false);
    while (!config_.Get<bool>("ShouldClose").value_or(true)) {
        const auto nextFrame = backend_.nextFrame(window_);
        OnUpdate(nextFrame);
        OnRender(nextFrame);
    }
}

void VPCRImpl::OnUpdate(std::uint32_t frameIndex)
{
    const auto currentTime = std::chrono::high_resolution_clock::now();
    const auto timeSinceLastUpdate = (currentTime - lastFrameTimeStamp_).count() / 1000000000.f;  // In seconds
    lastFrameTimeStamp_ = currentTime;

    ++frameCounter_;

    // Print FPS and statistics on window title
    {
        const auto *statistics =
            static_cast<const Statistics *>(backend_.getMapping(pipelines_[frameIndex].statisticsDownload));
        if ((currentTime - lastTitleUpdate_).count() / 1000000000.f >= 1.f) {
            lastTitleUpdate_ = std::chrono::high_resolution_clock::now();

            backend_.setWindowTitle(window_, "VPCR, Frame rate: " + std::to_string(frameCounter_) +
                                                 " FPS, Drawn batches: " + std::to_string(statistics->drawnBatches) +
                                                 "/" + std::to_string(pointCloudAcceleration_->GetBatchCount()));
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
    const auto& depthPass = pipelines_[frameIndex].projectionPass;
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
    depthPass->Execute(commandRecorder, batchCount);
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
    const DynamicConst dynamicConst{pointCloudAcceleration_->GetBatchCount(), 1000.0f/static_cast<float>(std::numeric_limits<int>::max()),std ::cbrtf(MaxBatchSize)};

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
}

void VPCRImpl::CreateRenderTarget()
{
    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();
    for (auto& pipeline : pipelines_) {
        const glm::vec2 resolution = camera_->GetResolution();
        const tga::BufferInfo info{tga::BufferUsage::storage, static_cast<size_t>(resolution.x * resolution.y) * sizeof(std::uint64_t)};
        pipeline.renderTarget = backend_.createBuffer(info);
    }
}

void VPCRImpl::CreateDepthBuffer()
{
    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();
    for (auto& pipeline : pipelines_) {
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
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        clearPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        clearPass->BindInput(camera_->GetBuffer(), 0, 0);
        clearPass->BindInput(dynamicConst_, 0, 1);
        clearPass->BindInput(pipeline.renderTarget, 1, 0);
        clearPass->BindInput(pipeline.depthBuffer, 1, 1);
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
}

/**
 * \brief
 */
void VPCRImpl::CreateProjectionPass()
{
    for (auto& pipeline : pipelines_) {
        auto& projectionPass = pipeline.projectionPass;

        // Use utility function to load Shader from File
        const auto computeShader = tga::loadShader("../shaders/projection_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}, {tga::BindingType::uniformBuffer}}},
                                            // Set = 1: RenderTarget, DepthBuffer
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageBuffer}}},
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

void VPCRImpl::CreateDisplayPass()
{
    for (auto& pipeline : pipelines_) {
        auto& displayPass = pipeline.displayPass;

        // Use utility function to load Shader from File
        const auto vertexShader =
            tga::loadShader("../shaders/fullScreenTriangle_vert.spv", tga::ShaderType::vertex, backend_);
        const auto fragmentShader =
            tga::loadShader("../shaders/renderTarget_frag.spv", tga::ShaderType::fragment, backend_);

        const tga::InputLayout inputLayout({
            // Set = 0: Camera, DynamicConst
            {{{tga::BindingType::uniformBuffer}, {tga::BindingType::uniformBuffer}}},
            // Set = 1: RenderTarget, DepthBuffer
            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageBuffer}}}
        });

        const tga::RenderPassInfo passInfo(vertexShader, fragmentShader, window_, {}, inputLayout,
                                           tga::ClearOperation::none);
        displayPass = std::make_unique<TGAQuadPass>(backend_, passInfo);

        displayPass->BindInput(camera_->GetBuffer(), 0, 0);
        displayPass->BindInput(dynamicConst_, 0, 1);
        displayPass->BindInput(pipeline.renderTarget, 1, 0);
        displayPass->BindInput(pipeline.depthBuffer, 1, 1);
    }
}

}  // namespace

std::unique_ptr<VPCR> CreateVPCR(Config config) { return std::make_unique<VPCRImpl>(std::move(config)); }