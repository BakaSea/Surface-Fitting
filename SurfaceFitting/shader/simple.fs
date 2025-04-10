#version 460 core
#define M_PI 3.14159265358979323846
in vec3 worldPos;
in vec3 worldNormal;
in vec2 TexCoords;

out vec4 FragColor;

uniform vec3 cameraPos;

vec3 lightPos = vec3(1.f, 1.f, 1.f);
vec3 lightI = vec3(10.f);
vec3 albedo = vec3(.7f, .6f, .5f);
//float roughness = .31623f;
float roughness = 0.05f;

vec3 F(vec3 F0, float VdotH) {
    return mix(F0, vec3(1.f), pow(1.f-VdotH, 5.f));
}

float D(float NdotH, float roughness) {
    float a2 = roughness*roughness;
    float cos2Theta = NdotH*NdotH;
    float tan2Theta = (1.f-cos2Theta)/cos2Theta;
    float d = 1.f+tan2Theta/a2;
    return 1.f/(M_PI*a2*cos2Theta*cos2Theta*d*d);
}

float lambda(float cosTheta, float roughness) {
    float cos2Theta = cosTheta*cosTheta;
    float tan2Theta = (1.f-cos2Theta)/cos2Theta;
    float a2 = roughness*roughness;
    return (sqrt(1.f+a2*tan2Theta)-1.f)/2.f;
}

float G(float NdotL, float NdotV, float roughness) {
    return 1.f/(1.f+lambda(NdotL, roughness)+lambda(NdotV, roughness));
}

vec3 tonemapping(vec3 color) {
    return pow(color, vec3(1.f/2.2f));
}

void main() {
    vec3 N = normalize(worldNormal);
    vec3 V = normalize(cameraPos-worldPos);
    vec3 L = lightPos-worldPos;
    float d2 = dot(L, L);
    L = normalize(L);
    vec3 H = normalize(L+V);
    float NdotH = clamp(dot(N, H), 0.f, 1.f);
    float NdotL = clamp(dot(N, L), 0.f, 1.f);
    float NdotV = clamp(dot(N, V), 0.f, 1.f);
    float VdotH = clamp(dot(V, H), 0.f, 1.f);
    //vec3 color = (N+1.f)/2.f;
    vec3 fresnel = F(albedo, VdotH);
    vec3 specular = fresnel*D(NdotH, roughness)*G(NdotL, NdotV, roughness)/(4.f*NdotV*NdotL);
    vec3 diffuse = albedo/M_PI;
    //vec3 color = diffuse*NdotL*lightI/d2;
    vec3 color = specular*NdotL*lightI/d2;
    color = tonemapping(color);
    FragColor = vec4(color, 1);
}