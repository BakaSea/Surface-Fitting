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
float roughness = .3f;
float metallic = .5f;

vec3 F(vec3 F0, float VdotH) {
    return F0 + (vec3(1.0f, 1.0f, 1.0f) - F0) * pow(1.0f - VdotH, 5.0f);
}

float D(float NdotH, float roughness) {
    float a2 = roughness*roughness;
	float d = ((NdotH*a2-NdotH)*NdotH+1);
	return a2/(d*d*M_PI);
}

float G(float NdotL, float NdotV, float roughness) {
	float k = roughness*roughness/2;
	float g_v = NdotV/(NdotV*(1-k)+k);
	float g_l = NdotL/(NdotL*(1-k)+k);
	return g_v*g_l;
}

vec3 tonemapping(vec3 color) {
    return pow(color, vec3(1.0/2.2));
}

void main() {
    vec3 N = normalize(worldNormal);
    vec3 V = normalize(cameraPos-worldPos);
    vec3 L = lightPos-worldPos;
    float d = length(L);
    float d2 = d*d;
    L = normalize(L);
    vec3 H = normalize(L+V);
    float NdotH = max(dot(N, H), 0.f);
    float NdotL = max(dot(N, L), 0.f);
    float NdotV = max(dot(N, V), 0.f);
    float VdotH = max(dot(V, H), 0.f);
    float LdotH = max(dot(L, H), 0.f);
    // float NdotH = abs(dot(N, H));
    // float NdotL = abs(dot(N, L));
    // float NdotV = abs(dot(N, V));
    // float VdotH = abs(dot(V, H));
    // float LdotH = abs(dot(L, H));

    vec3 dielectricDiff = albedo/M_PI;
    vec3 dielectricSpec = vec3(D(NdotH, roughness)*G(NdotL, NdotV, roughness)/(4.f*NdotV*NdotL));
    vec3 dielectric = mix(dielectricDiff, dielectricSpec, 0.04f+(1.f-0.04f)*pow(1.f-VdotH, 5.f));
    vec3 metallicSpec = F(albedo, VdotH)*D(NdotH, roughness)*G(NdotL, NdotV, roughness)/(4.f*NdotV*NdotL);
    vec3 brdf = mix(dielectric, metallicSpec, metallic);
    //vec3 color = (N+1.f)/2.f;
    vec3 color = brdf*NdotL*lightI/d2;
    color = tonemapping(color);
    FragColor = vec4(color, 1);
}