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
float metallic = 0.5f;

struct SGGX {
    float S_xx, S_xy, S_xz, S_yy, S_yz, S_zz;
};

struct Frame {
    vec3 s, t, n;
};

Frame coordinateSystem(vec3 n) {
    vec3 s, t;
    float si = 1.f;
    if (n.z < 0.f)
        si = -1.f;
    float a = -1.f/(si+n.z);
    float b = n.x*n.y*a;
    s = vec3(1.f+si*n.x*n.x*a, si*b, -si*n.x);
    t = vec3(b, si+n.y*n.y*a, -n.y);
    return Frame(s, t, n);
}

vec3 F(vec3 F0, float VdotH) {
    return F0 + (vec3(1.0f, 1.0f, 1.0f) - F0) * pow(1.0f - VdotH, 5.0f);
}

float D(float NdotH, float roughness) {
    float a2 = roughness*roughness;
	float d = ((NdotH*a2-NdotH)*NdotH+1);
	return a2/(d*d*M_PI);
}

float D(vec3 wm, SGGX sggx) {
    float detS = sggx.S_xx*sggx.S_yy*sggx.S_zz-sggx.S_xx*sggx.S_yz*sggx.S_yz-sggx.S_yy*sggx.S_xz*sggx.S_xz-sggx.S_zz*sggx.S_xy*sggx.S_xy+2.0f*sggx.S_xy*sggx.S_xz*sggx.S_yz;
    float den = wm.x*wm.x*(sggx.S_yy*sggx.S_zz-sggx.S_yz*sggx.S_yz)+wm.y*wm.y*(sggx.S_xx*sggx.S_zz-sggx.S_xz*sggx.S_xz)+wm.z*wm.z*(sggx.S_xx*sggx.S_yy-sggx.S_xy*sggx.S_xy)
        +2.0f*(wm.x*wm.y*(sggx.S_xz*sggx.S_yz-sggx.S_zz*sggx.S_xy)+wm.x*wm.z*(sggx.S_xy*sggx.S_yz-sggx.S_yy*sggx.S_xz)+wm.y*wm.z*(sggx.S_xy*sggx.S_xz-sggx.S_xx*sggx.S_yz));
    float D = pow(abs(detS), 1.5f)/(M_PI*den*den);
    return D;
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

    SGGX sggx = SGGX(0.09f, 0, 0, 0.09f, 0, 1);
    Frame f = coordinateSystem(N);
    vec3 wo = vec3(dot(f.s, V), dot(f.t, V), dot(f.n, V));
    vec3 wi = vec3(dot(f.s, L), dot(f.s, L), dot(f.s, L));
    vec3 wm = vec3(dot(f.s, H), dot(f.t, H), dot(f.n, H));

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