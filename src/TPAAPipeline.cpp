#include "TPAAPipeline.h"

#include "TGAGpuPass.h"
#include "Utils.h"
#include "imgui/imgui.h"

TPAAPipeline::TPAAPipeline(const Config& config, tga::Interface& backend, const tga::Window& window,
                           const Resources resources, const std::uint32_t batchCount)
    : config_(config), backend_(backend), window_(window), batchCount_(batchCount)
{
    pipelines_.resize(backend_.backbufferCount(window_));

    CreateBatchList();
    CreateRenderTarget();
    CreateDepthBuffer();

    // Create Passes
    CreateClearPass(resources);
    CreateLODPass(resources);
    CreateProjectionDepthPass(resources);
    CreateProjectionColorPass(resources);
    CreateDisplayPass(resources);
}

TPAAPipeline::~TPAAPipeline()
{
    for (const auto& pipeline : pipelines_) {
        backend_.free(pipeline.batchList);
        backend_.free(pipeline.renderTarget);
        backend_.free(pipeline.depthBuffer);
        backend_.free(pipeline.commandBuffer);
    }
}

void TPAAPipeline::Execute(const std::uint32_t frameIndex, const std::span<const UploadData *const> uploads,
                           const std::span<DownloadData *const> downloads)
{
    auto& commandBuffer = pipelines_[frameIndex].commandBuffer;
    const auto& clearPass = pipelines_[frameIndex].clearPass;
    const auto& lodPass = pipelines_[frameIndex].lodPass;
    const auto& projectionDepthPass = pipelines_[frameIndex].projectionDepthPass;
    const auto& projectionColorPass = pipelines_[frameIndex].projectionColorPass;
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
    projectionDepthPass->Execute(commandRecorder, batchCount_);
    commandRecorder.barrier(tga::PipelineStage::ComputeShader, tga::PipelineStage::ComputeShader);
    projectionColorPass->Execute(commandRecorder, batchCount_);
    commandRecorder.barrier(tga::PipelineStage::ComputeShader, tga::PipelineStage::FragmentShader);
    displayPass->Execute(commandRecorder, frameIndex);

    // GUI Execution Commands
    commandRecorder.guiStartFrame();
    ImGui::ShowDemoWindow(0);
    commandRecorder.guiEndFrame();

    // Execute
    commandBuffer = commandRecorder.endRecording();
    backend_.execute(commandBuffer);

    // Present
    backend_.present(window_, frameIndex);
}

void TPAAPipeline::CreateBatchList()
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

void TPAAPipeline::CreateRenderTarget()
{
    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();
    for (auto& pipeline : pipelines_) {
        const tga::BufferInfo info{tga::BufferUsage::storage,
                                   static_cast<size_t>(res[0] * res[1]) * sizeof(std::uint64_t)};
        pipeline.renderTarget = backend_.createBuffer(info);
    }
}

void TPAAPipeline::CreateDepthBuffer()
{
    const auto res = config_.Get<std::vector<std::uint32_t>>("resolution").value();
    for (auto& pipeline : pipelines_) {
        const tga::TextureInfo textureInfo(res[0], res[1], tga::Format::r32_uint);
        pipeline.depthBuffer = backend_.createTexture(textureInfo);
    }
}

void TPAAPipeline::CreateClearPass(const Resources resources)
{
    for (auto& pipeline : pipelines_) {
        auto& clearPass = pipeline.clearPass;

        // Use utility function to load Shader from File
        const auto computeShader = tga::loadShader("../shaders/clearTPAA_comp.spv", tga::ShaderType::compute, backend_);

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
             // Set = 1: RenderTarget, DepthBuffer
             {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageImage}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        clearPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        std::uint32_t i = 0;
        for (const auto& resource : resources) {
            clearPass->BindInput({resource, 0, i});
            ++i;
        }
        clearPass->BindInput({pipeline.renderTarget, 1, 0});
        clearPass->BindInput({pipeline.depthBuffer, 1, 1});
    }
}

void TPAAPipeline::CreateLODPass(const Resources resources)
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

void TPAAPipeline::CreateProjectionDepthPass(const Resources resources)
{
    for (auto& pipeline : pipelines_) {
        auto& projectionDepthPass = pipeline.projectionDepthPass;

        // Use utility function to load Shader from File
        const auto computeShader =
            tga::loadShader("../shaders/projectionDepthTPAA_comp.spv", tga::ShaderType::compute, backend_);

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
             // Set = 1: Rendertarget, Depthbuffer
             {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageImage}}},
             // Set = 2: Batch list
             {{{tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        projectionDepthPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        std::uint32_t i = 0;
        for (const auto& resource : resources) {
            projectionDepthPass->BindInput({resource, 0, i});
            ++i;
        }
        projectionDepthPass->BindInput({pipeline.renderTarget, 1, 0});
        projectionDepthPass->BindInput({pipeline.depthBuffer, 1, 1});
        projectionDepthPass->BindInput({pipeline.batchList, 2, 0});
    }
}

void TPAAPipeline::CreateProjectionColorPass(const Resources resources)
{
    for (auto& pipeline : pipelines_) {
        auto& projectionColorPass = pipeline.projectionColorPass;

        // Use utility function to load Shader from File
        const auto computeShader =
            tga::loadShader("../shaders/projectionColorTPAA_comp.spv", tga::ShaderType::compute, backend_);

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
             // Set = 1: Rendertarget, Depthbuffer
             {{{tga::BindingType::storageBuffer}, {tga::BindingType::storageImage}}},
             // Set = 2: Batch list
             {{{tga::BindingType::storageBuffer}}}});

        const tga::ComputePassInfo passInfo(computeShader, inputLayout);
        projectionColorPass = std::make_unique<TGAComputePass>(backend_, passInfo);

        std::uint32_t i = 0;
        for (const auto& resource : resources) {
            projectionColorPass->BindInput({resource, 0, i});
            ++i;
        }
        projectionColorPass->BindInput({pipeline.renderTarget, 1, 0});
        projectionColorPass->BindInput({pipeline.depthBuffer, 1, 1});
        projectionColorPass->BindInput({pipeline.batchList, 2, 0});
    }
}

void TPAAPipeline::CreateDisplayPass(const Resources resources)
{
    for (auto& pipeline : pipelines_) {
        auto& displayPass = pipeline.displayPass;

        // Use utility function to load Shader from File
        const auto vertexShader =
            tga::loadShader("../shaders/fullScreenTriangle_vert.spv", tga::ShaderType::vertex, backend_);
        const auto fragmentShader =
            tga::loadShader("../shaders/renderTargetTPAA_frag.spv", tga::ShaderType::fragment, backend_);

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
             // Set = 1: RenderTarget
             {{{tga::BindingType::storageBuffer}}}});

        const tga::RenderPassInfo passInfo(vertexShader, fragmentShader, window_, {}, inputLayout,
                                           tga::ClearOperation::none);
        displayPass = std::make_unique<TGAQuadPass>(backend_, passInfo);

        std::uint32_t i = 0;
        for (const auto& resource : resources) {
            displayPass->BindInput({resource, 0, i});
            ++i;
        }
        displayPass->BindInput({pipeline.renderTarget, 1});
    }
}