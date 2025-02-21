#version 460 core
in vec3 worldPos;
in vec3 worldNormal;
in vec2 TexCoords;

out vec4 FragColor;

uniform vec3 cameraPos;

void main() {
    //vec3 N = normalize(worldNormal);
    vec3 N = worldNormal;
    vec3 V = normalize(cameraPos-worldPos);
    //if (dot(N, V) < 0) N = -N;
    vec3 color = .5f*N+.5f;
    FragColor = vec4(color, 1);
}