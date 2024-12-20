#version 460 core
#define MAX_DIST 100
#define M_PI 3.1415926535897932384626433832795
#define SAMPLES 4

struct VoxelData{
    float q[10];
    float hullQ[10];
    float bmin[3];
    float bmax[3];
    float sigma;
};

layout (binding = 0, std430) readonly buffer ssbo {
    VoxelData voxels[];
};

uniform mat4 viewInv;
uniform mat4 projectionInv;
uniform vec3 cameraPos;
uniform vec2 resolution;
uniform int voxelSize;

out vec4 fragColor;

struct Ray {
    vec3 o;
    vec3 d;
};

struct IntersectResult {
    float q[10];
    vec3 intersection;
    float t;
};

float random(vec2 uv) {
    return fract(sin(dot(uv.xy, vec2(12.9898,78.233))) * 43758.5453123);
}

float f(float q[10], vec3 p) {
    return q[0]*p.x*p.x+q[1]*p.y*p.y+q[2]*p.z*p.z+q[3]*p.x*p.y+q[4]*p.x*p.z+q[5]*p.y*p.z+q[6]*p.x+q[7]*p.y+q[8]*p.z+q[9];
}

vec3 df(float q[10], vec3 p) {
    return vec3(
        2*p.x*q[0]+p.y*q[3]+p.z*q[4]+q[6],
        2*p.y*q[1]+p.x*q[3]+p.z*q[5]+q[7],
        2*p.z*q[2]+p.x*q[4]+p.y*q[5]+q[8]
        );
}

IntersectResult intersectVoxel(Ray r, int idx, float tMin, float tMax, float rnd) {
    VoxelData voxel = voxels[idx];
    vec3 bmin = {voxel.bmin[0], voxel.bmin[1], voxel.bmin[2]};
    vec3 bmax = {voxel.bmax[0], voxel.bmax[1], voxel.bmax[2]};
    float d = clamp(voxel.sigma*sqrt(-2*log(rnd))*cos(2*M_PI*rnd), -3.f*voxel.sigma, 3.f*voxel.sigma);
    float a = voxel.q[0]*r.d.x*r.d.x+voxel.q[1]*r.d.y*r.d.y+voxel.q[2]*r.d.z*r.d.z
        +voxel.q[3]*r.d.x*r.d.y+voxel.q[4]*r.d.x*r.d.z+voxel.q[5]*r.d.y*r.d.z;
    float b = 2*voxel.q[0]*r.o.x*r.d.x+2*voxel.q[1]*r.o.y*r.d.y+2*voxel.q[2]*r.o.z*r.d.z
        +voxel.q[3]*(r.o.x*r.d.y+r.o.y*r.d.x)+voxel.q[4]*(r.o.x*r.d.z+r.o.z*r.d.x)+voxel.q[5]*(r.o.y*r.d.z+r.o.z*r.d.y)
        +voxel.q[6]*r.d.x+voxel.q[7]*r.d.y+voxel.q[8]*r.d.z;
    float c = f(voxel.q, r.o)-d;
    //float c = f(voxel.q, r.o);
    // float a = voxel.hullQ[0]*r.d.x*r.d.x+voxel.hullQ[1]*r.d.y*r.d.y+voxel.hullQ[2]*r.d.z*r.d.z
    //     +voxel.hullQ[3]*r.d.x*r.d.y+voxel.hullQ[4]*r.d.x*r.d.z+voxel.hullQ[5]*r.d.y*r.d.z;
    // float b = 2*voxel.hullQ[0]*r.o.x*r.d.x+2*voxel.hullQ[1]*r.o.y*r.d.y+2*voxel.hullQ[2]*r.o.z*r.d.z
    //     +voxel.hullQ[3]*(r.o.x*r.d.y+r.o.y*r.d.x)+voxel.hullQ[4]*(r.o.x*r.d.z+r.o.z*r.d.x)+voxel.hullQ[5]*(r.o.y*r.d.z+r.o.z*r.d.y)
    //     +voxel.hullQ[6]*r.d.x+voxel.hullQ[7]*r.d.y+voxel.hullQ[8]*r.d.z;
    // float c = f(voxel.hullQ, r.o);
    float delta = b*b-4*a*c;
    IntersectResult res;
    res.t = MAX_DIST;
    if (delta >= 0) {
        float sqrtDelta = sqrt(delta);
        float t = (-b-sqrtDelta)/(2*a);
        if (tMin < t && t < tMax) {
            vec3 p = r.o+t*r.d;
            if (min(p, bmin) == bmin && max(p, bmax) == bmax && f(voxel.hullQ, p) <= 0.1f) {
                res.t = min(res.t, t);
            }
        }
        t = (-b+sqrtDelta)/(2*a);
        if (tMin < t && t < tMax) {
            vec3 p = r.o+t*r.d;
            if (min(p, bmin) == bmin && max(p, bmax) == bmax && f(voxel.hullQ, p) <= 0.1f) {
                res.t = min(res.t, t);
            }
        }
        if (res.t < MAX_DIST) {
            res.intersection = r.o+res.t*r.d;
            res.q = voxel.q;
            res.q[9] -= d;
            // res.q = voxel.hullQ;
        }
    }
    return res;
}

IntersectResult intersect(Ray r, vec2 seed) {
    IntersectResult res;
    res.t = MAX_DIST;
    int idx = -1;
    for (int i = 0; i < voxelSize; i++) {
        IntersectResult intr = intersectVoxel(r, i, 0, res.t, random(seed*vec2(i, 1)/vec2(voxelSize, 1)));
        if (intr.t < res.t) {
            res = intr;
        }
    }
    return res;
}

vec3 df(VoxelData voxel, vec3 p) {
    return vec3(0, 0, 0);
}

vec3 shadeVoxel(IntersectResult intr) {
    vec3 N = normalize(df(intr.q, intr.intersection));
    vec3 V = cameraPos-intr.intersection;
    if (dot(N, V) < 0) {
        N = -N;
    }
    //return N/2.f+.5f;
    vec3 lightPos = vec3(1.f, 1.f, 1.f);
    vec3 L = lightPos-intr.intersection;
    float r = length(L);
    L = normalize(L);
    vec3 H = normalize(L+V);
    float NdotL = dot(N, L);
    float NdotH = dot(N, H);
    vec3 Ld = vec3(1.f)*vec3(1.f)/(r*r)*max(NdotL, 0.f);
    vec3 Ls = vec3(1.f)*vec3(1.f)/(r*r)*pow(max(NdotH, 0.f), 16.f);
    return Ld+Ls;
}

vec3 tonemapping(vec3 color) {
    return pow(color, vec3(1.f/2.2f));
}

void main() {
    vec2 uv = gl_FragCoord.xy/resolution*2.f-1.f;
    vec4 ndcRayDir = vec4(uv, 1.f, 1.f);
    vec4 worldRayDir = viewInv*projectionInv*ndcRayDir;
    vec2 seed = gl_FragCoord.xy/resolution;
    vec3 color = vec3(0, 0, 0);
    bool hit = false;
    for (int s = 0; s < SAMPLES; s++) {
        Ray r = {cameraPos, worldRayDir.xyz/worldRayDir.w};
        r.d = normalize(r.d);
        IntersectResult intr = intersect(r, seed*vec2(1, s)/vec2(1, SAMPLES));
        if (intr.t < MAX_DIST) {
            hit = true;
            color += shadeVoxel(intr)/SAMPLES;
        }
    }
    if (!hit) discard;
    fragColor = vec4(tonemapping(color), 1.f);
}