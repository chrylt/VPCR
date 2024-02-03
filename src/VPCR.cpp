#include "VPCR.h"

#include <chrono>

#include "PipelineFactory.h"
#include "TGACamera.h"
#include "TGAPointCloudAcceleration.h"
#include "Utils.h"

namespace
{

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

    struct KeyPressed {
        bool j = false;
    };

    enum WarpWideDeduplicationSetting : std::uint32_t { None, Pairs, Full, COUNT };

    struct DynamicConst {
        // Misc information we need on gpu
        std::uint32_t totalBatchCount;
        float depthStepSize;
        float lodExtend;
        WarpWideDeduplicationSetting warpWideDeduplication;
    };

    class DynamicConstBuffer : public IPipeline::UploadData {
    public:
        DynamicConstBuffer(tga::Interface& backend, std::uint32_t batchCount);
        tga::CommandRecorder& Upload(tga::CommandRecorder& recorder) const override;
        tga::Buffer GetBuffer() const;

        DynamicConst data;

    private:
        tga::Interface& backend_;

        tga::Buffer buffer_;
    };

    struct Statistics {
        std::uint32_t drawnBatches;
    };

    class StatisticsBuffer : public IPipeline::DownloadData {
    public:
        StatisticsBuffer(tga::Interface& backend);
        tga::CommandRecorder& Download(tga::CommandRecorder& recorder) override;
        tga::Buffer GetBuffer() const;

        Statistics data;

    private:
        tga::Interface& backend_;

        tga::Buffer buffer_;
        tga::StagingBuffer stagingBuffer_;
    };

    enum AntiAliasingMode : std::uint32_t {
        Off = 0,
        TwoPass = 1,
        DensityOnePass = 2,
        DensityTwoPass = 3,
        AA_MODE_COUNT /* always keep this as the last element */
    };
    constexpr inline static char const *AntiAliasingModeStrings[AA_MODE_COUNT] = {"OFF", "TWO PASS", "DENSITY", "DENSITY TWO PASS"};

    void OnUpdate(std::uint32_t frameIndex);
    void OnRender(std::uint32_t frameIndex);

    Config config_;
    tga::Interface backend_;

    tga::Window window_;

    std::chrono::time_point<std::chrono::high_resolution_clock> lastFrameTimeStamp_;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastTitleUpdate_;
    std::uint32_t frameCounter_ = 0;

    std::optional<UserInputCache> userInputCache_;
    KeyPressed keyPressed_;

    std::unique_ptr<TGACamera> camera_;
    std::unique_ptr<DynamicConstBuffer> dynamicConst_;
    std::unique_ptr<StatisticsBuffer> statistics_;

    std::unique_ptr<PipelineFactory> pipelineFactory_;
    std::unique_ptr<IPipeline> pipeline_;

    AntiAliasingMode currAntiAliasingMode = AntiAliasingMode::Off;

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

    // Create Resources
    pointCloudAcceleration_ = std::make_unique<TGAPointCloudAcceleration>(backend_, scenePath.value());
    camera_ = std::make_unique<TGACamera>(backend_, res[0], res[1], glm::vec3(0, 0, 3), -90.f);
    dynamicConst_ = std::make_unique<DynamicConstBuffer>(backend_, pointCloudAcceleration_->GetBatchCount());
    statistics_ = std::make_unique<StatisticsBuffer>(backend_);

    pipelineFactory_ = std::make_unique<PipelineFactory>();

    const auto [low, medium, high, color] = pointCloudAcceleration_->GetPointsBufferPack();
    pipeline_ = pipelineFactory_->CreateBasicPipeline(
        config_, backend_, window_,
        std::vector<std::variant<tga::Buffer, tga::Texture, tga::ext::TopLevelAccelerationStructure>>{
            camera_->GetBuffer(), dynamicConst_->GetBuffer(), statistics_->GetBuffer(), low, medium, high, color,
            pointCloudAcceleration_->GetBatchesBuffer()},
        pointCloudAcceleration_->GetBatchCount());

    // Init GUI
    backend_.initGUI(window_);
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

    // Toggle Features bases on user input
    {
        if (keyPressed_.j && !backend_.keyDown(window_, tga::Key::J)) {
            currAntiAliasingMode = static_cast<AntiAliasingMode>((currAntiAliasingMode + 1) % AA_MODE_COUNT);
            const auto [low, medium, high, color] = pointCloudAcceleration_->GetPointsBufferPack();
            switch (currAntiAliasingMode) {
                case AntiAliasingMode::Off: {
                    pipeline_ = pipelineFactory_->CreateBasicPipeline(
                        config_, backend_, window_,
                        std::vector<std::variant<tga::Buffer, tga::Texture, tga::ext::TopLevelAccelerationStructure>>{
                            camera_->GetBuffer(), dynamicConst_->GetBuffer(), statistics_->GetBuffer(), low, medium,
                            high, color, pointCloudAcceleration_->GetBatchesBuffer()},
                        pointCloudAcceleration_->GetBatchCount());
                    break;
                }
                case AntiAliasingMode::TwoPass: {
                    pipeline_ = pipelineFactory_->CreateTPAAPipeline(
                        config_, backend_, window_,
                        std::vector<std::variant<tga::Buffer, tga::Texture, tga::ext::TopLevelAccelerationStructure>>{
                            camera_->GetBuffer(), dynamicConst_->GetBuffer(), statistics_->GetBuffer(), low, medium,
                            high, color, pointCloudAcceleration_->GetBatchesBuffer()},
                        pointCloudAcceleration_->GetBatchCount());
                    break;
                }
                case AntiAliasingMode::DensityOnePass: {
                    pipeline_ = pipelineFactory_->CreateOPDAAPipeline(
                        config_, backend_, window_,
                        std::vector<std::variant<tga::Buffer, tga::Texture, tga::ext::TopLevelAccelerationStructure>>{
                            camera_->GetBuffer(), dynamicConst_->GetBuffer(), statistics_->GetBuffer(), low, medium,
                            high, color, pointCloudAcceleration_->GetBatchesBuffer()},
                        pointCloudAcceleration_->GetBatchCount());
                    break;
                }
                case AntiAliasingMode::DensityTwoPass: {
                    pipeline_ = pipelineFactory_->CreateTPDAAPipeline(
                        config_, backend_, window_,
                        std::vector<std::variant<tga::Buffer, tga::Texture, tga::ext::TopLevelAccelerationStructure>>{
                            camera_->GetBuffer(), dynamicConst_->GetBuffer(), statistics_->GetBuffer(), low, medium,
                            high, color, pointCloudAcceleration_->GetBatchesBuffer()},
                        pointCloudAcceleration_->GetBatchCount());
                    break;
                }
                default: {
                    assert(false);
                }
            }
        }
        keyPressed_.j = backend_.keyDown(window_, tga::Key::J);
    }

    // Print FPS and statistics on window title
    {
        if ((currentTime - lastTitleUpdate_).count() / 1000000000.f >= 1.f) {
            lastTitleUpdate_ = std::chrono::high_resolution_clock::now();

            backend_.setWindowTitle(window_,
                                    "VPCR, Frame rate: " + std::to_string(frameCounter_) +
                                        " FPS, Drawn batches: " + std::to_string(statistics_->data.drawnBatches) + "/" +
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

    // Update DynamicConst
    {
        if (backend_.keyDown(window_, tga::Key::W)) {
            const auto newMode = static_cast<WarpWideDeduplicationSetting>((dynamicConst_.warpWideDeduplication + 1) %
                                                                           WarpWideDeduplicationSetting::COUNT);
            dynamicConst_.warpWideDeduplication = newMode;
        }
    }
}

void VPCRImpl::OnRender(std::uint32_t frameIndex)
{
    const std::array<const IPipeline::UploadData *, 2> uploads{camera_.get(), dynamicConst_.get()};
    const std::array<IPipeline::DownloadData *, 1> downloads{statistics_.get()};

    pipeline_->Execute(frameIndex, uploads, downloads);

    config_.Set("ShouldClose", backend_.windowShouldClose(window_));
}

VPCRImpl::DynamicConstBuffer::DynamicConstBuffer(tga::Interface& backend, const std::uint32_t batchCount)
    : backend_(backend)
{
    // We are using the cubic root of the MaxBatchSize as a heuristic for the size of a batch before it loses
    // precision Generally we would like to know the area in pixels of a projected batch that can be coverd by its
    // content before leaving holes
    constexpr float depthStepSize = (1000.0f / static_cast<float>(std::numeric_limits<int>::max())) * 10'000;
    data = {batchCount, depthStepSize, std::cbrtf(MaxBatchSize)};

    const tga::StagingBufferInfo stagingInfo{sizeof(data), reinterpret_cast<const std::uint8_t *>(&data)};
    const auto staging = backend_.createStagingBuffer(stagingInfo);
    const tga::BufferInfo info{tga::BufferUsage::uniform, sizeof(DynamicConst), staging};
    buffer_ = backend_.createBuffer(info);

    backend_.free(staging);
}

tga::CommandRecorder& VPCRImpl::DynamicConstBuffer::Upload(tga::CommandRecorder& recorder) const
{
    recorder.inlineBufferUpdate(buffer_, &data, sizeof(data));
    return recorder;
}

tga::Buffer VPCRImpl::DynamicConstBuffer::GetBuffer() const { return buffer_; }

VPCRImpl::StatisticsBuffer::StatisticsBuffer(tga::Interface& backend) : backend_(backend)
{
    const tga::StagingBufferInfo stagingInfo{sizeof(Statistics)};
    stagingBuffer_ = backend_.createStagingBuffer(stagingInfo);
    const tga::BufferInfo info{tga::BufferUsage::storage, sizeof(Statistics), stagingBuffer_};
    buffer_ = backend_.createBuffer(info);
}

tga::CommandRecorder& VPCRImpl::StatisticsBuffer::Download(tga::CommandRecorder& recorder)
{
    recorder.bufferDownload(buffer_, stagingBuffer_, sizeof(data));
    data = *static_cast<VPCRImpl::Statistics *>(backend_.getMapping(stagingBuffer_));
    return recorder;
}

tga::Buffer VPCRImpl::StatisticsBuffer::GetBuffer() const { return buffer_; }

}  // namespace

std::unique_ptr<VPCR> CreateVPCR(Config config) { return std::make_unique<VPCRImpl>(std::move(config)); }