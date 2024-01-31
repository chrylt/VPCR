struct Point{
    vec3 position;
    uint rgba;
};

struct AABB{
    float minX;
    float minY;
    float minZ;
    float maxX;
    float maxY;
    float maxZ;
};

struct Node{
    AABB box;
    uint childrenPointer;
    uint pointOffset;
    uint maskDepthCount;  // higher 19 bits are the count, lower 8 the child mask, middle 5 the tree depth
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

uint wang_hash(uint seed)
{
    seed = (seed ^ 61) ^ (seed >> 16);
    seed *= 9;
    seed = seed ^ (seed >> 4);
    seed *= 0x27d4eb2d;
    seed = seed ^ (seed >> 15);
    return seed;
}