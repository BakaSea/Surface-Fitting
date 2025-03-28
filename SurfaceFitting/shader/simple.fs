#version 460 core
#define M_PI 3.14159265358979323846
in vec3 worldPos;
in vec3 worldNormal;
in vec2 TexCoords;

out vec4 FragColor;

uniform vec3 cameraPos;

vec3 lightPos = vec3(1.f, 1.f, 1.f);
vec3 lightI = vec3(10.f);

vec3 F() {
    return vec3(0.f);
}

float D(vec3 wm) {
    return 0.f;
}

float G() {
    return 0.f;
}

void main() {
    //vec3 N = normalize(worldNormal);
    vec3 N = worldNormal;
    vec3 V = normalize(cameraPos-worldPos);
    // vec3 L = lightPos-worldPos;
    // float d2 = dot(L, L);
    // L = normalize(L);
    // vec3 H = (L+V)/2.f;
    // float NdotH = clamp(dot(N, H), 0.f, 1.f);
    // float NdotL = clamp(dot(N, L), 0.f, 1.f);
    // vec3 color = lightI/d2*vec3(.7f, .6f, .5f)*(1.f*1.f/M_PI*NdotL+1.f*pow(NdotH, .5f));
    vec3 color = (N+1.f)/2.f;
    FragColor = vec4(color, 1);
}