layout(set = 0, binding = 0) uniform Camera
{
    mat4 view;
    mat4 projection;
    vec3 direction;
    vec3 position;
    uvec2 resolution;
}
camera;

layout(set = 0, binding = 1) uniform DynamicConst { uint totalBatchCount; };