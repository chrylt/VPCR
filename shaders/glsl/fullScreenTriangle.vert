#version 460

layout(location = 0) out FragData{
    vec2 texCoords;
}fragData;

const vec2 vertices[3] = vec2[3](
    vec2(-1,-1), 
    vec2(3,-1), 
    vec2(-1, 3)
);

void main()
{
    gl_Position = vec4(vertices[gl_VertexIndex],0,1);
    fragData.texCoords = 0.5 * gl_Position.xy + vec2(0.5);
}
