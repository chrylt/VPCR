#pragma once

#include <string_view>
#include <tga/tga.hpp>
#include <tga/tga_utils.hpp>
#include <tga/tga_vulkan/tga_vulkan.hpp>

struct BindingInfo {
    std::variant<tga::Buffer, tga::Texture, tga::ext::TopLevelAccelerationStructure> resource;
    std::uint32_t set;
    std::uint32_t slot = 0;
    std::uint32_t arrayIndex = 0;
    std::uint32_t swapIndex = 0;
};

class TGAGpuPass {
public:
    void BindInput(const BindingInfo& binding);

    virtual ~TGAGpuPass();

protected:
    TGAGpuPass(tga::Interface& tgai, std::uint32_t swapCount);
    void Init();

    tga::Interface& backend_;

    std::vector<std::vector<tga::InputSetInfo>> inputInfos_;
    std::vector<std::vector<tga::InputSet>> inputs_;
    std::uint32_t swapCount_;
    std::uint32_t swapIndex_;
    mutable bool needsRebind_;
};

class TGARenderPass final : public TGAGpuPass {
public:
    TGARenderPass(tga::Interface& tgai, const tga::RenderPassInfo& info, std::uint32_t swapCount = 1);

    void BindIndexBuffer(tga::Buffer indices);
    void BindVertexBuffer(tga::Buffer vertices);
    void BindCommandsBuffer(tga::Buffer commands);

    tga::CommandRecorder& Execute(tga::CommandRecorder& recorder, std::uint32_t drawCount,
                                  std::uint32_t frameIndex = 0);

    ~TGARenderPass();

private:
    tga::Buffer indexBuffer_;
    tga::Buffer vertexBuffer_;
    tga::Buffer commandsBuffer_;

    tga::RenderPass pass_;
};

class TGAQuadPass final : public TGAGpuPass {
public:
    TGAQuadPass(tga::Interface& tgai, const tga::RenderPassInfo& info, std::uint32_t swapCount = 1);

    tga::CommandRecorder& Execute(tga::CommandRecorder& recorder, std::uint32_t frameIndex = 0);

    ~TGAQuadPass();

private:
    tga::RenderPass pass_;
};

class TGAComputePass final : public TGAGpuPass {
public:
    TGAComputePass(tga::Interface& tgai, const tga::ComputePassInfo& info, std::uint32_t swapCount = 1);

    tga::CommandRecorder& Execute(tga::CommandRecorder& recorder, std::uint32_t dispatchSizeX,
                                  std::uint32_t dispatchSizeY = 1, std::uint32_t dispatchSizeZ = 1);

    ~TGAComputePass();

private:
    tga::ComputePass pass_;
};