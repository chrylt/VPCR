#version 460
#extension GL_KHR_vulkan_glsl : enable
#extension GL_ARB_gpu_shader_int64 : enable
#include "utility_common.glsl"

layout(set = 0, binding = 0) uniform Camera{
    mat4 view;
    mat4 projection;
    vec3 direction;
    vec3 position;
    uvec2 resolution;
}camera;

layout(set = 0, binding = 1) uniform DynamicConst{
    uint workerCount;
    uint depthDiscSteps;
};

layout(set = 1, binding = 0) buffer RENDER_TARGET{
    uint64_t renderTarget[];
};

layout(set = 1, binding = 1) buffer DEPTH_BUFFER{ 
    Histogram depthBuffer[];
};

layout(location = 0) out vec4 color;

void main()
{
    const uint64_t storedValue = renderTarget[getPixelID(camera.resolution, uvec2(gl_FragCoord.xy))];
    float counter = float(storedValue & 0xFFFF);
    float r = float((storedValue >> 48) & 0xFFFF) / counter;
    float g = float((storedValue >> 32) & 0xFFFF) / counter;
    float b = float((storedValue >> 16) & 0xFFFF) / counter;
    color = vec4(vec3(r, g, b) / 255, 1);

    if(counter >= 255)
    {
        // overflow detected
        color = vec4(1, 0, 0, 1);
    }

    uint pixelID = getPixelID(camera.resolution, uvec2(gl_FragCoord.xy));

    int prevIdx = -1;
    int currIdx = depthBuffer[pixelID].startIdx;

    bool foundMax = false;
    while(!foundMax && currIdx != -1){

        // add contribution
        const uint64_t storedValue = renderTarget[pixelID];
        float counter = float(storedValue & 0xFFFF);
        float r = float((storedValue >> 48) & 0xFFFF) / counter;
        float g = float((storedValue >> 32) & 0xFFFF) / counter;
        float b = float((storedValue >> 16) & 0xFFFF) / counter;

        // TODO: check for possible overflow
        // TODO: accumulate the thingies

        // end of loop operations
        prevIdx = currIdx;
        currIdx = depthBuffer[pixelID].buckets[currIdx].nextIdx;

        if(currIdx < prevIdx){
            foundMax = true;
        }
    }

}