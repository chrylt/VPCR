#version 460
#extension GL_ARB_gpu_shader_int64 : enable
#extension GL_KHR_vulkan_glsl : enable
#include "common.glsl"

layout(set = 1, binding = 0) buffer COMBINED_BUFFER{
    uint64_t framebuffer[];
};

layout(location = 0) out vec4 color;

void main()
{   
    const uint pixelID = getPixelID(camera.resolution, uvec2(gl_FragCoord.xy));
    const uint storedColor = uint(framebuffer[pixelID] & 0xFFFFFFFF);
    color = unpackUnorm4x8(storedColor);
}