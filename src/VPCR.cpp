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
        bool e = false;
    };

    enum WarpWideDedMode : std::int32_t { None, Pairs, Full, WARPWIDE_MODE_COUNT };
    constexpr inline static char const *warpWideDedModeStrings[WARPWIDE_MODE_COUNT] = {"NONE", "PAIRS", "FULL"};

    struct DynamicConst {
        // Misc information we need on gpu
        std::uint32_t totalBatchCount;
        float depthStepSize;
        float lodExtend;
        WarpWideDedMode warpWideDeduplication;
        float cullingFovY;
        std::uint32_t showTreeDepth;  // 0 means show all layers
        struct {
            std::uint32_t colorBatchById : 1;
            std::uint32_t colorTreeByDepth : 1;
            std::uint32_t enableFrustumCulling : 1;
            std::uint32_t enableLOD : 1;
            std::uint32_t colorVertexPrecision : 1;
            std::uint32_t AAErrorCodes : 1;
            std::uint32_t AApreventOverflow : 1;
            std::uint32_t AApreventedOverflowVis : 1;
            std::uint32_t DAABucketVis : 1;
            std::uint32_t padding : 23;
        };  // toggleFlags
        std::uint32_t TPDAABucketsShowPerID;
        float depthPercTPAA;
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

    std::unique_ptr<TGAPointCloudAcceleration> pointCloudAcceleration_;

    AntiAliasingMode currAntiAliasingMode = AntiAliasingMode::Off;
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
    backend_.setWindowTitle(window_, "Vulkan Point Cloud Renderer");

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

    config_.Set("LOD.maxTreeDepth", static_cast<int>(pointCloudAcceleration_->GetMaxTreeDepth()));
    config_.Set("LOD.maxSelection", std::sqrt(static_cast<float>(res[0] * res[0] + res[1] * res[1])));
    // We are using the cubic root of the MaxBatchSize as a heuristic for the size of a batch before it loses
    // precision Generally we would like to know the area in pixels of a projected batch that can be coverd by its
    // content before leaving holes
    config_.Set("LOD.selection", std::cbrt(static_cast<float>(MaxBatchSize)));
    config_.Set("LOD.defaultSelection", std::cbrt(static_cast<float>(MaxBatchSize)));
    config_.Set("LOD.warpWideDeduplication", 0);
    config_.Set("VP.vis", false);
    config_.Set("AA.currAAMode", AntiAliasingMode::Off);
    config_.Set("AA.errorShow", false);
    config_.Set("AA.preventOverflow", true);
    config_.Set("AA.preventedOverflowVis", false);
    config_.Set("DAA.visualizeDensityBuckets", false);
    config_.Set("TPDAA.bucketIDToShow", 0);
    config_.Set("TPAA.depthPerc", 0.01f);
    config_.Set("TitleBar.update", std::string("0"));
    config_.Set("TitleBar.fps", std::string("0"));
    config_.Set("TitleBar.DrawnBatches", std::string("0"));
    config_.Set("TitleBar.TotalBatches", std::string("0"));
}

void VPCRImpl::Run()
{
    config_.Set("ShouldClose", false);
    while (!config_.Get<bool>("ShouldClose").value_or(true)) {
        const auto nextFrame = backend_.nextFrame(window_);
        config_.ApplyDirty();
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

    // Apply config
    {
        dynamicConst_->data.enableFrustumCulling = config_.Get<bool>("LOD.culling").value();
        dynamicConst_->data.enableLOD = config_.Get<bool>("LOD.acceleration").value();
        dynamicConst_->data.colorBatchById = config_.Get<bool>("LOD.colorBatch").value();
        dynamicConst_->data.colorTreeByDepth = config_.Get<bool>("LOD.colorDepth").value();
        dynamicConst_->data.showTreeDepth = config_.Get<int>("LOD.level").value();
        dynamicConst_->data.lodExtend = config_.Get<float>("LOD.selection").value();
        dynamicConst_->data.cullingFovY = config_.Get<float>("LOD.cullingFov").value();
        dynamicConst_->data.warpWideDeduplication =
            static_cast<WarpWideDedMode>(config_.Get<int>("LOD.warpWideDeduplication").value());
        dynamicConst_->data.colorVertexPrecision = config_.Get<bool>("VP.vis").value();
        dynamicConst_->data.AAErrorCodes = config_.Get<bool>("AA.errorShow").value();
        dynamicConst_->data.AApreventOverflow = config_.Get<bool>("AA.preventOverflow").value();
        dynamicConst_->data.AApreventedOverflowVis = config_.Get<bool>("AA.preventedOverflowVis").value();
        dynamicConst_->data.DAABucketVis = config_.Get<bool>("DAA.visualizeDensityBuckets").value();
        dynamicConst_->data.TPDAABucketsShowPerID = config_.Get<int>("TPDAA.bucketIDToShow").value();
        dynamicConst_->data.depthStepSize = config_.Get<float>("OPDAA.bucketSize").value();
        dynamicConst_->data.depthPercTPAA = config_.Get<float>("TPAA.depthPerc").value();
    }

    // Toggle Features bases on user input
    {
        const auto configAntiAliasingMode = config_.Get<AntiAliasingMode>("AA.currAAMode").value();
        if (currAntiAliasingMode != configAntiAliasingMode) {
            currAntiAliasingMode = configAntiAliasingMode;
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
    }

    // Write statistics to config for GUI
    {
        if ((currentTime - lastTitleUpdate_).count() / 1000000000.f >= 1.f) {
            lastTitleUpdate_ = std::chrono::high_resolution_clock::now();
            config_.Set("TitleBar.fps", std::to_string(frameCounter_));
            config_.Set("TitleBar.DrawnBatches", std::to_string(statistics_->data.drawnBatches));
            config_.Set("TitleBar.TotalBatches", std::to_string(pointCloudAcceleration_->GetBatchCount()));
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
    data = {batchCount, 0.1f, std::cbrtf(MaxBatchSize)};

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