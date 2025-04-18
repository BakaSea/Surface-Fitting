#version 460 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

out vec3 worldPos;
out vec3 worldNormal;
out vec2 TexCoords;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
    TexCoords = aTexCoords;    
    gl_Position = projection*view*model*vec4(aPos, 1.f);
    vec4 wPos = model*vec4(aPos, 1.f);
    worldPos = wPos.xyz/wPos.w;
    worldNormal = mat3(transpose(inverse(model)))*aNormal;
}