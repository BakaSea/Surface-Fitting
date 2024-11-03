#version 330 core
out vec4 FragColor;

in vec3 worldPos;
in vec3 worldNormal;
in vec2 TexCoords;

uniform sampler2D texture_diffuse1;
uniform vec3 cameraPos;

void main() {
    vec3 N = normalize(worldNormal);
    vec3 V = normalize(cameraPos-worldPos);
    if (dot(N, V) < 0) N = -N;
    vec3 color = .5f*N+.5f;
    FragColor = vec4(color, 1);
}