#include "TGACamera.h"

TGACamera::TGACamera(tga::Interface& tgai, const std::uint32_t resX, const std::uint32_t resY, const glm::vec3 position,
                     const float jaw, const float pitch, const glm::vec3 up)
    : backend_(tgai), jaw_(jaw), pitch_(pitch), position_(position), up_(up)
{
    cam_.resolution = glm::uvec2(resX, resY);
    cam_.projection =
        glm::perspective_vk(glm::radians(70.f), static_cast<float>(resX) / static_cast<float>(resY), 0.0001f, 50000.f);

    const tga::StagingBufferInfo stagingCameraInfo(sizeof(Camera));
    staging_ = backend_.createStagingBuffer(stagingCameraInfo);

    Update(position, jaw, pitch);

    const tga::BufferInfo cameraInfo(tga::BufferUsage::uniform, sizeof(Camera), staging_);
    buffer_ = backend_.createBuffer(cameraInfo);
    const tga::Binding cameraBinding(buffer_);
}

void TGACamera::Update(const glm::vec3 position, const float jaw, const float pitch)
{
    position_ = position;
    jaw_ = jaw;

    // Sanitize pitch, compute direction vector
    pitch_ = std::min(89.f, std::max(-89.f, pitch));

    direction_.x = cos(glm::radians(jaw_)) * cos(glm::radians(pitch_));
    direction_.y = sin(glm::radians(pitch_));
    direction_.z = sin(glm::radians(jaw_)) * cos(glm::radians(pitch_));
    direction_ = glm::normalize(direction_);

    // Apply view
    cam_.view = glm::lookAt(position_, position_ + direction_, glm::vec3(0, 1, 0));
    cam_.direcction = direction_;
    cam_.position = position_;

    *reinterpret_cast<Camera *>(backend_.getMapping(staging_)) = cam_;
}

tga::CommandRecorder& TGACamera::Upload(tga::CommandRecorder& commandRecorder) const
{
    return commandRecorder.inlineBufferUpdate(buffer_, &cam_, sizeof(Camera));
}

tga::Buffer TGACamera::GetBuffer() { return buffer_; }

glm::vec3 TGACamera::GetPosition() const { return position_; }
glm::vec3 TGACamera::GetDirection() const { return direction_; }
float TGACamera::GetJaw() const { return jaw_; }
float TGACamera::GetPitch() const { return pitch_; }

glm::vec2 TGACamera::GetResolution() const { return cam_.resolution; }

TGACamera::~TGACamera()
{
    backend_.free(staging_);
    backend_.free(buffer_);
}