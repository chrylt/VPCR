#version 460
#extension GL_KHR_vulkan_glsl: enable

const vec2 vertices[3] = vec2[3](
    vec2(-1,-1), 
    vec2(3,-1), 
    vec2(-1, 3)
);

void main()
{
    gl_Position = vec4(vertices[gl_VertexIndex],0,1);
}
