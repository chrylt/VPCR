#include "BasicPipeline.h"

#include "Gui.h"
#include "TGAGpuPass.h"
#include "Utils.h"

BasicPipeline::BasicPipeline(const Config& config, tga::Interface& backend, const tga::Window& window,
                             const Resources resources, const std::uint32_t batchCount)
    : config_(config), backend_(backend), window_(window), batchCount_(batchCount)
{
    pipelines_.resize(backend_.backbufferCount(window_));

    CreateBatchList();
    CreateFrameBuffer();

    // Create Passes
    CreateClearPass(resources);
    CreateLODPass(resources);
    CreateProjectionPass(resources);
    CreateDisplayPass(resources);
}

BasicPipeline::~BasicPipeline()
{
    for (const auto& pipeline : pipelines_) {
        backend_.free(pipeline.batchList);
        backend_.free(pipeline.frameBuffer);
        backend_.free(pipeline.commandBuffer);
    }
}

void BasicPipeline::Execute(const std::uint32_t frameIndex, const std::span<const UploadData *const> uploads,
                            const std::span<DownloadData *const> downloads)
{
    auto& commandBuffer = pipelines_[frameIndex].commandBuffer;
    const auto& clearPass = pipelines_[frameIndex].clearPass;
    const auto& lodPass = pipelines_[frameIndex].lodPass;
    const auto& projectionPass = pipelines_[frameIndex].projectionPass;
    const auto& displayPass = pipelines_[frameIndex].displayPass;

    auto commandRecorder = tga::CommandRecorder{backend_, commandBuffer};

    // Readback
    for (auto *downloadData : downloads) {
        downloadData->Download(commandRecorder);
    }

    // Upload
    for (const auto *uploadData : uploads) {
        uploadData->Upload(commandRecorder);
    }
    // Set the batch count to 0 as the LOD pass will determine the batches to be rendered
    constexpr std::uint32_t zero = 0;
    commandRecorder.inlineBufferUpdate(pipelines_[frameIndex].batchList, &zero, sizeof(zero));
    commandRecorder.barrier(tga::PipelineStage::Transfer, tga::PipelineStage::ComputeShader);

    // Collect Execution Commands
    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();
    clearPass->Execute(commandRecorder, res[0] / ComputeLaneCount + 1, res[1]);
    lodPass->Execute(commandRecorder, batchCount_);
    commandRecorder.barrier(tga::PipelineStage::ComputeShader, tga::PipelineStage::ComputeShader);
    projectionPass->Execute(commandRecorder, batchCount_);
    commandRecorder.barrier(tga::PipelineStage::ComputeShader, tga::PipelineStage::FragmentShader);
    displayPass->Execute(commandRecorder, frameIndex);

    // GUI Execution Commands
    commandRecorder.guiStartFrame();
    RenderGui(config_);
    commandRecorder.guiEndFrame();

    // Execute
    commandBuffer = commandRecorder.endRecording();
    backend_.execute(commandBuffer);

    // Present
    backend_.present(window_, frameIndex);
}

void BasicPipeline::CreateBatchList()
{
    // The first entry of the batch list is reserved for the batch count in the list, all other are indices into the
    // batch buffer
    for (auto& pipeline : pipelines_) {
        std::vector<std::uint32_t> batchList(batchCount_ + 1);
        batchList[0] = batchCount_;
        for (std::uint32_t i = 0; i < batchCount_; ++i) {
            batchList[i + 1] = i;
        }

        tga::StagingBufferInfo stagingInfo{(batchCount_ + 1) * sizeof(std::uint32_t),
                                           reinterpret_cast<const std::uint8_t *>(batchList.data())};
        const auto staging = backend_.createStagingBuffer(stagingInfo);
        const tga::BufferInfo info{tga::BufferUsage::storage, (batchCount_ + 1) * sizeof(std::uint32_t), staging};
        pipeline.batchList = backend_.createBuffer(info);

        backend_.free(staging);
    }
}

void BasicPipeline::CreateFrameBuffer()
{
    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();
    for (auto& pipeline : pipelines_) {
        const tga::BufferInfo info{tga::BufferUsage::storage, static_cast<size_t>(res[0] * res[1]) * sizeof(uint64_t)};
        pipeline.frameBuffer = backend_.createBuffer(info);
    }
}

void BasicPipeline::CreateClearPass(const Resources resources)
{
    for (auto& pipeline : pipelines_) {
        auto& clearPass = pipeline.clearPass;

        // Use utility function to load Shader from File
        const auto computeShader = tga::loadShader("../shaders/clear_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout(
            {// Set = 0: Camera, DynamicConst, Statistics, PointsPositionLowPrecision, PointsPositionMediumPrecision,
             // PointsPositionHighPrecision, PointsColor, Batches
             {{
                 {tga::BindingType::uniformBuffer},
                 {tga::BindingType::uniformBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
             }},
             // Set = 1: FrameBuffer
             {{{tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        clearPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        std::uint32_t i = 0;
        for (const auto& resource : resources) {
            clearPass->BindInput({resource, 0, i});
            ++i;
        }
        clearPass->BindInput({pipeline.frameBuffer, 1, 0});
    }
}

void BasicPipeline::CreateLODPass(const Resources resources)
{
    for (auto& pipeline : pipelines_) {
        auto& lodPass = pipeline.lodPass;

        const auto computeShader = tga::loadShader("../shaders/lod_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout(
            {// Set = 0: Camera, DynamicConst, Statistics, PointsPositionLowPrecision, PointsPositionMediumPrecision,
             // PointsPositionHighPrecision, PointsColor, Batches
             {{
                 {tga::BindingType::uniformBuffer},
                 {tga::BindingType::uniformBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
             }},
             // Set = 1: Batch List
             {{{tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        lodPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        std::uint32_t i = 0;
        for (const auto& resource : resources) {
            lodPass->BindInput({resource, 0, i});
            ++i;
        }
        lodPass->BindInput({pipeline.batchList, 1, 0});
    }
}

void BasicPipeline::CreateProjectionPass(const Resources resources)
{
    for (auto& pipeline : pipelines_) {
        auto& projectionPass = pipeline.projectionPass;

        // Use utility function to load Shader from File
        const auto computeShader =
            tga::loadShader("../shaders/projection_comp.spv", tga::ShaderType::compute, backend_);

        const tga::InputLayout inputLayout(
            {// Set = 0: Camera, DynamicConst, Statistics, PointsPositionLowPrecision, PointsPositionMediumPrecision,
             // PointsPositionHighPrecision, PointsColor, Batches
             {{
                 {tga::BindingType::uniformBuffer},
                 {tga::BindingType::uniformBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
             }},
             // Set = 1: FrameBuffer
             {{{tga::BindingType::storageBuffer}}},
             // Set = 3: Batch list
             {{{tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        projectionPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        std::uint32_t i = 0;
        for (const auto& resource : resources) {
            projectionPass->BindInput({resource, 0, i});
            ++i;
        }
        projectionPass->BindInput({pipeline.frameBuffer, 1, 0});
        projectionPass->BindInput({pipeline.batchList, 2, 0});
    }
}

void BasicPipeline::CreateDisplayPass(const Resources resources)
{
    for (auto& pipeline : pipelines_) {
        auto& displayPass = pipeline.displayPass;

        // Use utility function to load Shader from File
        const auto vertexShader =
            tga::loadShader("../shaders/fullScreenTriangle_vert.spv", tga::ShaderType::vertex, backend_);
        const auto fragmentShader =
            tga::loadShader("../shaders/renderTarget_frag.spv", tga::ShaderType::fragment, backend_);

        const tga::InputLayout inputLayout(
            {// Set = 0: Camera, DynamicConst, Statistics, PointsPositionLowPrecision, PointsPositionMediumPrecision,
             // PointsPositionHighPrecision, PointsColor, Batches
             {{
                 {tga::BindingType::uniformBuffer},
                 {tga::BindingType::uniformBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
                 {tga::BindingType::storageBuffer},
             }},
             // Set = 1: FrameBuffer
             {{{tga::BindingType::storageBuffer}}}});

        const tga::RenderPassInfo passInfo(vertexShader, fragmentShader, window_, {}, inputLayout,
                                           tga::ClearOperation::none);
        displayPass = std::make_unique<TGAQuadPass>(backend_, passInfo);

        std::uint32_t i = 0;
        for (const auto& resource : resources) {
            displayPass->BindInput({resource, 0, i});
            ++i;
        }
        displayPass->BindInput({pipeline.frameBuffer, 1});
    }
}