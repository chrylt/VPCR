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

struct Bucket{
    uint bucketID;
    int nextIdx;
    uint64_t acc;
};

struct Histogram{
    int startIdx;
    uint bucketCount;
    Bucket buckets[100];   // maximal 100 filled buckets per pixel
};

uint getPixelID(const uvec2 resolution, const uvec2 pixelCoord){
    return pixelCoord.x * resolution.x + pixelCoord.y;
}