#include "TGAGpuPass.h"

TGAGpuPass::TGAGpuPass(tga::Interface& tgai, const std::uint32_t swapCount)
    : backend_(tgai), swapCount_(swapCount), swapIndex_(0), needsRebind_(true)
{}

void TGAGpuPass::BindInput(
    const std::variant<tga::Buffer, tga::Texture, tga::ext::TopLevelAccelerationStructure> resource,
    const std::uint32_t set, const std::uint32_t slot, const std::uint32_t arrayIndex, const std::uint32_t swapIndex)
{
    needsRebind_ = true;

    // Check whether we are updating an existing binding
    for (auto& binding : inputInfos_.at(swapIndex).at(set).bindings) {
        if ((binding.slot == slot) && (binding.arrayElement == arrayIndex)) {
            // Overwrite existing binding
            binding = {resource, slot, arrayIndex};
            return;
        }
    }

    // Add new binding
    inputInfos_.at(swapIndex).at(set).bindings.emplace_back(resource, slot, arrayIndex);
}

void TGAGpuPass::Init()
{
    if (needsRebind_) {
        needsRebind_ = false;
        for (const auto& inputs : inputs_) {
            for (const auto& input : inputs) {
                backend_.free(input);
            }
        }
        inputs_.clear();

        for (const auto& infoSet : inputInfos_) {
            auto& inputs = inputs_.emplace_back();
            for (const auto& info : infoSet) {
                inputs.push_back(backend_.createInputSet(info));
            }
        }
    }
}

TGAGpuPass::~TGAGpuPass()
{
    for (const auto& inputSwap : inputs_) {
        for (const auto& input : inputSwap) {
            backend_.free(input);
        }
    }
}

TGARenderPass::TGARenderPass(tga::Interface& tgai, const tga::RenderPassInfo& info, const std::uint32_t swapCount)
    : TGAGpuPass(tgai, swapCount)
{
    pass_ = backend_.createRenderPass(info);
    inputInfos_.resize(swapCount);
    for (auto& inputInfoSet : inputInfos_) {
        for (std::uint32_t i = 0; i < info.inputLayout.size(); ++i) {
            auto& inputInfo = inputInfoSet.emplace_back(pass_);
            inputInfo.index = i;
        }
    }
}

void TGARenderPass::BindIndexBuffer(tga::Buffer indices) { indexBuffer_ = indices; }

void TGARenderPass::BindVertexBuffer(tga::Buffer vertices) { vertexBuffer_ = vertices; }

void TGARenderPass::BindCommandsBuffer(tga::Buffer commands) { commandsBuffer_ = commands; }

tga::CommandRecorder& TGARenderPass::Execute(tga::CommandRecorder& recorder, const std::uint32_t drawCount,
                                             const std::uint32_t frameIndex)
{
    Init();

    recorder.setRenderPass(pass_, frameIndex);

    recorder.bindIndexBuffer(indexBuffer_).bindVertexBuffer(vertexBuffer_);
    for (const auto& input : inputs_[swapIndex_]) {
        recorder.bindInputSet(input);
    }

    swapIndex_ = (swapIndex_ + 1) % swapCount_;

    return recorder.drawIndexedIndirect(commandsBuffer_, drawCount);
}

TGARenderPass::~TGARenderPass() { backend_.free(pass_); }

TGAQuadPass::TGAQuadPass(tga::Interface& tgai, const tga::RenderPassInfo& info, const std::uint32_t swapCount)
    : TGAGpuPass(tgai, swapCount)
{
    pass_ = backend_.createRenderPass(info);
    inputInfos_.resize(swapCount);
    for (auto& inputInfoSet : inputInfos_) {
        for (std::uint32_t i = 0; i < info.inputLayout.size(); ++i) {
            auto& inputInfo = inputInfoSet.emplace_back(pass_);
            inputInfo.index = i;
        }
    }
}

tga::CommandRecorder& TGAQuadPass::Execute(tga::CommandRecorder& recorder, const std::uint32_t frameIndex)
{
    Init();

    recorder.setRenderPass(pass_, frameIndex);

    for (const auto& input : inputs_[swapIndex_]) {
        recorder.bindInputSet(input);
    }

    swapIndex_ = (swapIndex_ + 1) % swapCount_;

    return recorder.draw(3, 0);
}

TGAQuadPass::~TGAQuadPass() { backend_.free(pass_); }

TGAComputePass::TGAComputePass(tga::Interface& tgai, const tga::ComputePassInfo& info, const std::uint32_t swapCount)
    : TGAGpuPass(tgai, swapCount)
{
    pass_ = backend_.createComputePass(info);
    inputInfos_.resize(swapCount);
    for (auto& inputInfoSet : inputInfos_) {
        for (std::uint32_t i = 0; i < info.inputLayout.size(); ++i) {
            auto& inputInfo = inputInfoSet.emplace_back(pass_);
            inputInfo.index = i;
        }
    }
}

tga::CommandRecorder& TGAComputePass::Execute(tga::CommandRecorder& recorder, std::uint32_t dispatchSizeX,
                                              std::uint32_t dispatchSizeY, std::uint32_t dispatchSizeZ)
{
    Init();

    recorder.setComputePass(pass_);

    for (const auto& input : inputs_[swapIndex_]) {
        recorder.bindInputSet(input);
    }

    swapIndex_ = (swapIndex_ + 1) % swapCount_;

    return recorder.dispatch(dispatchSizeX, dispatchSizeY, dispatchSizeZ);
}

TGAComputePass::~TGAComputePass() { backend_.free(pass_); }