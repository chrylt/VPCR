#version 460
#extension GL_KHR_vulkan_glsl : enable
#extension GL_ARB_gpu_shader_int64 : enable
#include "common.glsl"

layout(set = 1, binding = 0) buffer HistogramPerPixel{
    HistogramTP histogram[];
};

layout(set = 1, binding = 1, r32ui) uniform uimage2D depthBuffer;

layout(location = 0) out vec4 color;

void main()
{
    //@Atzubi toggle idea: render one bucket at a time
    uint pixelID = getPixelID(camera.resolution, uvec2(gl_FragCoord.xy));

    // accumulate values
    float counter = 0;
    float r = 0;
    float g = 0;
    float b = 0;

    float prevCounter = 0;
    bool overflowDetected = false;

    for(uint i = 0; i < BUCKET_COUNT_TPDAA; ++i){
        // unpack bucket values
        const uint64_t storedValue = histogram[pixelID].buckets[i].acc; 
        float bucketCounter = float(storedValue & 0xFFFF);
        float bucketR = float((storedValue >> 48) & 0xFFFF);
        float bucketG = float((storedValue >> 32) & 0xFFFF);
        float bucketB = float((storedValue >> 16) & 0xFFFF);

        if(prevCounter > bucketCounter)
            break;  // local maximum found
            
        if(counter + bucketCounter >= 255){
            // potential overflow detected; stop iteration
            overflowDetected = true;
            break;
        }else{
            // add contribution
            counter += bucketCounter;
            r += bucketR;
            g += bucketG;
            b += bucketB;
        }

        // save for comparison with next
        prevCounter = bucketCounter;
    }

    color = vec4(vec3(r, g, b) / (counter + 0.00001) / 255, 1);

    if(counter >= 255){// || overflowDetected){
        color = vec4(1, 0, 1, 1);   //print magenta at overflow
    }
}