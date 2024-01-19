struct Point {
    vec3 position;
    uint rgba;
};

struct AABB {
    vec3 minV;
    vec3 maxV;
};

struct Batch {
    AABB box;
    uint pointOffset;
    uint pointCount;
};

uint getPixelID(uvec2 resolution, uvec2 pixelCoord){
    return pixelCoord.x * resolution.x + pixelCoord.y;
}