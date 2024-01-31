#pragma once

#include <tga/tga.hpp>
#include <tga/tga_utils.hpp>
#include <tga/tga_vulkan/tga_vulkan.hpp>

class TGACamera {
public:
    struct Camera {
        alignas(16) glm::mat4 view = glm::mat4(1);
        alignas(16) glm::mat4 projection = glm::mat4(1);
        alignas(16) glm::vec3 direcction = glm::vec3(0, 0, 1);
        alignas(16) glm::vec3 position = glm::vec3(0, 0, 1);
        alignas(16) glm::uvec2 resolution = glm::uvec2(1, 1);
        alignas(16) float nearFarDistance;
    };

    TGACamera(tga::Interface& tgai, std::uint32_t resX, std::uint32_t resY, glm::vec3 position = glm::vec3(0),
              float jaw = 0, float pitch = 0, glm::vec3 up = glm::vec3(0.f, 1.f, 0.f));

    void Update(glm::vec3 position, float jaw, float pitch);

    tga::CommandRecorder& Upload(tga::CommandRecorder& recorder) const;

    tga::Buffer GetBuffer();

    glm::vec3 GetPosition() const;
    glm::vec3 GetDirection() const;
    float GetJaw() const;
    float GetPitch() const;
    glm::vec2 GetResolution() const;

    ~TGACamera();

private:
    tga::Interface& backend_;

    float jaw_;
    float pitch_;
    glm::vec3 position_;
    glm::vec3 direction_;
    glm::vec3 up_;
    Camera cam_;

    tga::StagingBuffer staging_;
    tga::Buffer buffer_;
};