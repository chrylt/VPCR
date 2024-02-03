#include "utility_common.glsl"

layout(set = 0, binding = 0) uniform Camera{
    mat4 view;
    mat4 projection;
    vec3 direction;
    vec3 position;
    uvec2 resolution;
    float nearFarDistance;
    float fovY;
}camera;

layout(set = 0, binding = 1) uniform DynamicConst{
    uint totalBatchCount;
    float depthStepSize;
    float lodExtend;
    uint warpWideDeduplication;
};

layout(set = 0, binding = 2) writeonly buffer Statistics{
    uint drawnBatches;
};

layout(set = 0, binding = 3) readonly buffer Points_Position_low_Precision{
    uint pointsPosLow[];
};

layout(set = 0, binding = 4) readonly buffer Points_Position_medium_Precision{
    uint pointsPosMedium[];
};

layout(set = 0, binding = 5) readonly buffer Points_Position_high_Precision{
    uint pointsPosHigh[];
};

layout(set = 0, binding = 6) buffer Points_Color{
    uint pointsRgba[];
};

layout(set = 0, binding = 7) readonly buffer BatchNodes{
    Node nodes[];  
};

uint getBatchPixelExtend(const Node currBatch, const uvec2 resolution, const mat4 projection, const mat4 view){

    // determine precision by projecting batch aabb to screen
    /// create aabb corner points
    const vec3[8] aabbCorners = vec3[](
        vec3(currBatch.box.minX, currBatch.box.minY, currBatch.box.minZ), // 0 - minimum
        vec3(currBatch.box.minX, currBatch.box.minY, currBatch.box.maxZ), // 0 0 1 -> 1
        vec3(currBatch.box.minX, currBatch.box.maxY, currBatch.box.minZ), // 0 1 0 -> 2
        vec3(currBatch.box.minX, currBatch.box.maxY, currBatch.box.maxZ), // 0 1 1 -> 3
        vec3(currBatch.box.maxX, currBatch.box.minY, currBatch.box.minZ), // 1 0 0 -> 4
        vec3(currBatch.box.maxX, currBatch.box.minY, currBatch.box.maxZ), // 1 0 1 -> 5
        vec3(currBatch.box.maxX, currBatch.box.maxY, currBatch.box.minZ), // 1 1 0 -> 6
        vec3(currBatch.box.maxX, currBatch.box.maxY, currBatch.box.maxZ)  // 7 - maximum
    );

    /// find aabb extend on screen
    uvec2 screenMin = resolution;
    uvec2 screenMax = uvec2(0);

    for(uint i = 0; i < 8; ++i){
        /// transform aabb & perspective divide
        const vec4 divReady = projection * view * vec4(aabbCorners[i], 1);
        const vec2 divided = (divReady / abs(divReady.w)).xy;
        const vec2 pixelSpace = ((divided + 1) / 2) * resolution;

        screenMin = ivec2(min(screenMin.x, pixelSpace.x), min(screenMin.y, pixelSpace.y));
        screenMax = ivec2(max(screenMax.x, pixelSpace.x), max(screenMax.y, pixelSpace.y));
    }

    const uint pixelExtend = max(screenMax.x - screenMin.x, screenMax.y - screenMin.y);
    return pixelExtend;
}

vec3 getAdaptivePointPosition(const vec3 batchBoxSize, const vec3 batchBoxMin, const uint batchPixelExtend, const uint pointIdx){

    vec3 resultPosition = batchBoxMin;

    /// low precision
    const uint encodedLow = pointsPosLow[pointIdx];
    const vec3 offsetLow = vec3(encodedLow & 0x3FF, (encodedLow >> 10) & 0x3FF, (encodedLow >> 20) & 0x3FF) / (1 << 10) * batchBoxSize;
    resultPosition += offsetLow;

    if(batchPixelExtend > 512) { // 2^(10 bits) = 1024 different values; can accurately display BB across 512 pixels
        /// add medium precision
        const uint encodedMedium = pointsPosMedium[pointIdx];
        const vec3 offsetMedium = vec3(encodedMedium & 0x3FF, (encodedMedium >> 10) & 0x3FF, (encodedMedium >> 20) & 0x3FF) / (1 << 20) * batchBoxSize;
        resultPosition += offsetMedium;
    }

    if(batchPixelExtend > 524288) {  // 2^(20 bits) = 1'048'576 different values; can accurately display BB across 524'288 pixels
        /// add high precision
        const uint encodedHigh = pointsPosHigh[pointIdx];
        const vec3 offsetHigh = vec3(encodedHigh & 0x3FF, (encodedHigh >> 10) & 0x3FF, (encodedHigh >> 20) & 0x3FF) / (1 << 30) * batchBoxSize;
        resultPosition += offsetHigh;
    }

    return resultPosition;
}