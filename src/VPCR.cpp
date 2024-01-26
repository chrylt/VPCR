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
        tga::Buffer renderTarget;
        tga::Buffer depthBuffer;
        std::unique_ptr<TGAComputePass> clearPass;
        std::unique_ptr<TGAComputePass> lodPass;
        std::unique_ptr<TGAComputePass> depthPass;
        std::unique_ptr<TGAComputePass> colorPass;
        std::unique_ptr<TGAQuadPass> displayPass;
        tga::CommandBuffer commandBuffer;
    };

    struct DynamicConst {
        // Misc information we need on gpu
        std::uint32_t totalBatchCount;
        std::uint32_t depthDiscSteps;
    };

    void OnUpdate(std::uint32_t frameIndex);
    void OnRender(std::uint32_t frameIndex);

    void CreateDynamicConst();
    void CreateBatchList();
    void CreateRenderTarget();
    void CreateDepthBuffer();

    void CreateClearPass();
    void CreateLODPass();
    void CreateDepthPass();
    void CreateColorPass();
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

    // Create Passes
    CreateClearPass();
    CreateLODPass();
    CreateDepthPass();
    CreateColorPass();
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

    // Print FPS on window title
    {
        if ((currentTime - lastTitleUpdate_).count() / 1000000000.f >= 1.f) {
            lastTitleUpdate_ = std::chrono::high_resolution_clock::now();

            backend_.setWindowTitle(window_, "VPCR, Framerate: " + std::to_string(frameCounter_) + " FPS");
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
    const auto& depthPass = pipelines_[frameIndex].depthPass;
    const auto& colorPass = pipelines_[frameIndex].colorPass;
    const auto& displayPass = pipelines_[frameIndex].displayPass;

    auto commandRecorder = tga::CommandRecorder{backend_, commandBuffer};

    // Upload
    camera_->Upload(commandRecorder);
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
    DynamicConst dynamicConst{pointCloudAcceleration_->GetBatchCount(), 1000};

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

void VPCRImpl::CreateClearPass()
{
    for (auto& pipeline : pipelines_) {
        auto& clearPass = pipeline.clearPass;

        // Use utility function to load Shader from File
        const auto computeShader = tga::loadShader("../shaders/clear_comp.spv", tga::ShaderType::compute, backend_);

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

void VPCRImpl::CreateLODPass()
{
    for (auto& pipeline : pipelines_) {
        auto& lodPass = pipeline.lodPass;

        const auto computeShader = tga::loadShader("../shaders/lod_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}, {tga::BindingType::uniformBuffer}}},
                                            // Set = 1: Acceleration structure
                                            {{{tga::BindingType::storageBuffer}}},
                                            // Set = 2: Batches, Batch List
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        lodPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        lodPass->BindInput(camera_->GetBuffer(), 0, 0);
        lodPass->BindInput(dynamicConst_, 0, 1);
        lodPass->BindInput(pointCloudAcceleration_->GetAccelerationStructureBuffer(), 1);
        lodPass->BindInput(pointCloudAcceleration_->GetBatchesBuffer(), 2, 0);
        lodPass->BindInput(pipeline.batchList, 2, 1);
    }
}

/**
 * \brief
 */
void VPCRImpl::CreateDepthPass()
{
    for (auto& pipeline : pipelines_) {
        auto& depthPass = pipeline.depthPass;

        // Use utility function to load Shader from File
        const auto computeShader = tga::loadShader("../shaders/depth_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}, {tga::BindingType::uniformBuffer}}},
                                            // Set = 1: RenderTarget, DepthBuffer
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageImage}}},
                                            // Set = 2: PointsPositionLowPrecision, PointsPositionMediumPrecision,
                                            // PointsPositionHighPrecision, PointsColor
                                            {{{tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer}}},
                                            // Set = 3: Batches, Batch list
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        depthPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        depthPass->BindInput(camera_->GetBuffer(), 0, 0);
        depthPass->BindInput(dynamicConst_, 0, 1);
        depthPass->BindInput(pipeline.renderTarget, 1, 0);
        depthPass->BindInput(pipeline.depthBuffer, 1, 1);

        // Point information buffers
        const auto& pointsBufferPack = pointCloudAcceleration_->GetPointsBufferPack();
        depthPass->BindInput(pointsBufferPack.positionLowPrecision, 2, 0);
        depthPass->BindInput(pointsBufferPack.positionMediumPrecision, 2, 1);
        depthPass->BindInput(pointsBufferPack.positionHighPrecision, 2, 2);
        depthPass->BindInput(pointsBufferPack.colors, 2, 3);

        depthPass->BindInput(pointCloudAcceleration_->GetBatchesBuffer(), 3, 0);
        depthPass->BindInput(pipeline.batchList, 3, 1);
    }
}

void VPCRImpl::CreateColorPass()
{
    for (auto& pipeline : pipelines_) {
        auto& colorPass = pipeline.colorPass;

        // Use utility function to load Shader from File
        const auto computeShader =
            tga::loadShader("../shaders/color_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout({// Set = 0: Camera, DynamicConst
                                            {{{tga::BindingType::uniformBuffer}, {tga::BindingType::uniformBuffer}}},
                                            // Set = 1: RenderTarget, DepthBuffer
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageImage}}},
                                            // Set = 2: PointsPositionLowPrecision, PointsPositionMediumPrecision,
                                            // PointsPositionHighPrecision, PointsColor
                                            {{{tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer},
                                              {tga::BindingType::storageBuffer}}},
                                            // Set = 3: Batches, Batch list
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        colorPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        colorPass->BindInput(camera_->GetBuffer(), 0, 0);
        colorPass->BindInput(dynamicConst_, 0, 1);
        colorPass->BindInput(pipeline.renderTarget, 1, 0);
        colorPass->BindInput(pipeline.depthBuffer, 1, 1);

        // Point information buffers
        const auto& pointsBufferPack = pointCloudAcceleration_->GetPointsBufferPack();
        colorPass->BindInput(pointsBufferPack.positionLowPrecision, 2, 0);
        colorPass->BindInput(pointsBufferPack.positionMediumPrecision, 2, 1);
        colorPass->BindInput(pointsBufferPack.positionHighPrecision, 2, 2);
        colorPass->BindInput(pointsBufferPack.colors, 2, 3);

        colorPass->BindInput(pointCloudAcceleration_->GetBatchesBuffer(), 3, 0);
        colorPass->BindInput(pipeline.batchList, 3, 1);
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
            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageImage}}}
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