#pragma once

#include <tga/tga.hpp>
#include <tga/tga_utils.hpp>
#include <tga/tga_vulkan/tga_vulkan.hpp>

class IPipeline {
public:
    class UploadData {
    public:
        virtual tga::CommandRecorder& Upload(tga::CommandRecorder& recorder) const = 0;

        virtual ~UploadData() = default;
    };

    class DownloadData {
    public:
        virtual tga::CommandRecorder& Download(tga::CommandRecorder& recorder) = 0;

        virtual ~DownloadData() = default;
    };

    virtual void Execute(std::uint32_t frameIndex, std::span<const UploadData *const> uploadData = {},
                         std::span<DownloadData *const> downloadData = {}) = 0;

    virtual ~IPipeline() = default;
};