#version 460
#extension GL_ARB_gpu_shader_int64 : enable
#extension GL_KHR_vulkan_glsl : enable
#include "common.glsl"

layout(set = 1, binding = 0) buffer DEPTH_BUFFER{ 
    Histogram depthBuffer[];
};

layout(location = 0) out vec4 color;

void main()
{
    uint pixelID = getPixelID(camera.resolution, uvec2(gl_FragCoord.xy));

    float counter = 0;
    float r = 0;
    float g = 0;
    float b = 0;

    int prevIdx = -1;
    int currIdx = depthBuffer[pixelID].startIdx;

    float prevBucketCounter = -1;

    bool foundMax = false;
    bool overflowDetected = false;
    uint TIMEOUT = 1;
    uint timeoutCounter = 0;
    while(!foundMax && currIdx != -1 && !overflowDetected && timeoutCounter < TIMEOUT){
        timeoutCounter += 1;

        // unpack bucket values
        const uint64_t storedValue = depthBuffer[pixelID].buckets[currIdx].acc;
        float bucketCounter = float(storedValue & 0xFFFF);
        float bucketR = float((storedValue >> 48) & 0xFFFF);
        float bucketG = float((storedValue >> 32) & 0xFFFF);
        float bucketB = float((storedValue >> 16) & 0xFFFF);

        if(prevIdx != -1 && (depthBuffer[pixelID].buckets[currIdx].bucketID < depthBuffer[pixelID].buckets[prevIdx].bucketID - 1 
            || bucketCounter < prevBucketCounter)){
            foundMax = true;
            continue;
        }

        if(counter + bucketCounter >= 255){
            // potential overflow detected; stop iteration
            overflowDetected = true;
            continue;
        }else{
            // add contribution
            counter += bucketCounter;
            r += bucketR;
            g += bucketG;
            b += bucketB;
        }

        // end of loop operations
        prevIdx = currIdx;
        prevBucketCounter = bucketCounter;
        currIdx = depthBuffer[pixelID].buckets[currIdx].nextIdx;
    }

    color = vec4(vec3(r, g, b) / (counter + 0.00001) / 255, 1);

    if(counter >= 255){// || overflowDetected){
        color = vec4(1, 0, 1, 1);   //print magenta at overflow
    }

    if(depthBuffer[pixelID].startIdx == -2)
        color = vec4(0, 1, 1, 1);   //print cyan at insert timeout (outer loop)

    if(depthBuffer[pixelID].startIdx == -3)
        color = vec4(1, 0, 0, 1);   //print red at and of histogram reached and insert failed

    if(depthBuffer[pixelID].startIdx == -3)
        color = vec4(0, 0, 1, 1);   //print blue when bucketCount is at limit

    // visualize bucketIDs @Atzubi TODO: toggle
    /*const int startIdx = depthBuffer[pixelID].startIdx;
    if(startIdx != -1){
        const uint foremostBucketID = depthBuffer[pixelID].buckets[startIdx].bucketID; 
        color = vec4(unpackUnorm4x8(wang_hash(foremostBucketID)).rgb, 1);
    }*/
}