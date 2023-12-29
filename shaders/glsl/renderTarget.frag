#version 460

layout(location = 0) in FragData{
    vec2 texCoords;
}fragData;

layout(set = 0, binding = 0) uniform sampler2D renderTarget;

layout(location = 0) out vec4 color;

void main()
{
    color = texture(renderTarget, fragData.texCoords);
}