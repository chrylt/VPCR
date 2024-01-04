#version 460
#extension GL_KHR_vulkan_glsl : enable

layout(location = 0) in FragData{   // TODO: take this out when it is decided to not use a sampler
    vec2 texCoords;
}fragData;

layout(set = 0, binding = 0, r32ui) uniform uimage2D renderTarget;

layout(location = 0) out vec4 color;

void main()
{
    const uint storedValue = imageLoad(renderTarget, ivec2(gl_FragCoord.xy)).r;
    color = unpackUnorm4x8(storedValue);
}