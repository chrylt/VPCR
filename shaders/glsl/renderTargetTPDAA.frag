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
        const float bucketCounter = float(storedValue & 0xFFFF);
        const float bucketR = float((storedValue >> 48) & 0xFFFF);
        const float bucketG = float((storedValue >> 32) & 0xFFFF);
        const float bucketB = float((storedValue >> 16) & 0xFFFF);

        if(prevCounter > bucketCounter)
            break;  // local maximum found
            
        if(counter + bucketCounter >= 255 && (toggleFlags & 64) != 0){
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

    // set final rendered color
    color = vec4(vec3(r, g, b) / (counter + 0.00001) / 255, 1);

    // debug visualizations

    if(counter >= 255 && (toggleFlags & 32) != 0){
        color = vec4(1, 0, 1, 1);   //print magenta at overflow
    } else if(overflowDetected && (toggleFlags & 128) != 0){
        color = vec4(0, 1, 1, 1);   //print cyan at prevented overflow
    }else if((toggleFlags & 256) != 0){
        // visualize only the points in certain bucket
        const uint64_t storedValue = histogram[pixelID].buckets[twoPassDensityBucketVis].acc; 
        const float bucketCounter = float(storedValue & 0xFFFF);
        const float bucketR = float((storedValue >> 48) & 0xFFFF);
        const float bucketG = float((storedValue >> 32) & 0xFFFF);
        const float bucketB = float((storedValue >> 16) & 0xFFFF);
        color = vec4(vec3(bucketR, bucketG, bucketB) / (bucketCounter + 0.00001) / 255, 1);
    }
}