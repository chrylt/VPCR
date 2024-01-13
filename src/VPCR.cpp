#include "VPCR.h"

#include <chrono>

#include "TGACamera.h"
#include "TGAGpuPass.h"
#include "TGAPointCloudAcceleration.h"
#include "Utils.h"

namespace
{
// Should match compute shaders
constexpr std::uint32_t ComputeLaneCount = 128;
constexpr std::uint32_t WorkerCount = 1536;  // TODO: get hardware thread count + heuristic

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
        tga::Texture renderTarget;
        tga::Texture depthBuffer;
        std::unique_ptr<TGAComputePass> clearPass;
        std::unique_ptr<TGAComputePass> lodPass;
        std::unique_ptr<TGAComputePass> projectionPass;
        std::unique_ptr<TGAQuadPass> displayPass;
        tga::CommandBuffer commandBuffer;
    };

    struct DynamicConst {
        // Misc information we need on gpu
        std::uint32_t threadCount;
    };

    void OnUpdate(std::uint32_t frameIndex);
    void OnRender(std::uint32_t frameIndex);

    void CreateDynamicConst();
    void CreateBatchList();
    void CreateRenderTarget();
    void CreateDepthBuffer();

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
    auto& clearPass = pipelines_[frameIndex].clearPass;
    auto& lodPass = pipelines_[frameIndex].lodPass;
    auto& projectionPass = pipelines_[frameIndex].projectionPass;
    auto& displayPass = pipelines_[frameIndex].displayPass;

    auto commandRecorder = tga::CommandRecorder{backend_, commandBuffer};

    // Upload
    camera_->Upload(commandRecorder);
    commandRecorder.barrier(tga::PipelineStage::Transfer, tga::PipelineStage::ComputeShader);

    // Collect Execution Commands
    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();
    clearPass->Execute(commandRecorder, res[0] / ComputeLaneCount + 1, res[1]);
    lodPass->Execute(commandRecorder, WorkerCount / ComputeLaneCount + 1);
    commandRecorder.barrier(tga::PipelineStage::ComputeShader, tga::PipelineStage::ComputeShader);
    projectionPass->Execute(commandRecorder, WorkerCount / ComputeLaneCount + 1);
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
    DynamicConst dynamicConst{WorkerCount};

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
        const tga::TextureInfo textureInfo(res[0], res[1], tga::Format::r32_uint);
        pipeline.renderTarget = backend_.createTexture(textureInfo);
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
                                            // Set = 2: Points
                                            {{{tga::BindingType::storageBuffer}}},
                                            // Set = 3: Batches, Batch list
                                            {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        projectionPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        projectionPass->BindInput(camera_->GetBuffer(), 0, 0);
        projectionPass->BindInput(dynamicConst_, 0, 1);
        projectionPass->BindInput(pipeline.renderTarget, 1, 0);
        projectionPass->BindInput(pipeline.depthBuffer, 1, 1);
        projectionPass->BindInput(pointCloudAcceleration_->GetPointsBuffer(), 2);
        projectionPass->BindInput(pointCloudAcceleration_->GetBatchesBuffer(), 3, 0);
        projectionPass->BindInput(pipeline.batchList, 3, 1);
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

        displayPass->BindInput(pipeline.renderTarget, 0);
    }
}

}  // namespace

std::unique_ptr<VPCR> CreateVPCR(Config config) { return std::make_unique<VPCRImpl>(std::move(config)); }