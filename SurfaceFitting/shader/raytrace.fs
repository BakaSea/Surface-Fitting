#version 460 core
#define MAX_DIST 100

struct VoxelData{
    float q[10];
    float bmin[3];
    float bmax[3];
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
    vec3 intersection;
    float t;
    int idx;
};

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

float intersectVoxel(Ray r, int idx, float tMin, float tMax) {
    VoxelData voxel = voxels[idx];
    vec3 bmin = {voxel.bmin[0], voxel.bmin[1], voxel.bmin[2]};
    vec3 bmax = {voxel.bmax[0], voxel.bmax[1], voxel.bmax[2]};
    float a = voxel.q[0]*r.d.x*r.d.x+voxel.q[1]*r.d.y*r.d.y+voxel.q[2]*r.d.z*r.d.z+voxel.q[3]*r.d.x*r.d.y+voxel.q[4]*r.d.x*r.d.z+voxel.q[5]*r.d.y*r.d.z;
    float b = 2*voxel.q[0]*r.o.x*r.d.x+2*voxel.q[1]*r.o.y*r.d.y+2*voxel.q[2]*r.o.z*r.d.z+voxel.q[3]*(r.o.x*r.d.y+r.o.y*r.d.x)+voxel.q[4]*(r.o.x*r.d.z+r.o.z*r.d.x)+voxel.q[5]*(r.o.y*r.d.z+r.o.z*r.d.y)+voxel.q[6]*r.d.x+voxel.q[7]*r.d.y+voxel.q[8]*r.d.z;
    float c = f(voxel.q, r.o);
    float delta = b*b-4*a*c;
    if (delta >= 0) {
        float sqrtDelta = sqrt(delta);
        float t = (-b-sqrtDelta)/(2*a);
        float res = MAX_DIST;
        if (tMin < t && t < tMax) {
            vec3 p = r.o+t*r.d;
            if (min(p, bmin) == bmin && max(p, bmax) == bmax) {
                res = min(res, t);
            }
        }
        t = (-b+sqrtDelta)/(2*a);
        if (tMin < t && t < tMax) {
            vec3 p = r.o+t*r.d;
            if (min(p, bmin) == bmin && max(p, bmax) == bmax) {
                res = min(res, t);
            }
        }
        return res;
    }
    return MAX_DIST;
}

IntersectResult intersect(Ray r) {
    IntersectResult res;
    res.t = MAX_DIST;
    for (int i = 0; i < voxelSize; i++) {
        float t = intersectVoxel(r, i, 0, res.t);
        if (t < res.t) {
            res.intersection = r.o+t*r.d;
            res.t = t;
            res.idx = i;
        }
    }
    return res;
}

vec3 df(VoxelData voxel, vec3 p) {
    return vec3(0, 0, 0);
}

vec4 shadeVoxel(int idx, vec3 p) {
    // float s = float(idx+1)/float(voxelSize);
    // return vec4(s, s, s, 1);
    VoxelData voxel = voxels[idx];
    vec3 N = normalize(df(voxel.q, p));
    vec3 V = cameraPos-p;
    if (dot(N, V) < 0) {
        N = -N;
    }
    return vec4(N/2.f+.5f, 1);
}

void main() {
    vec2 uv = gl_FragCoord.xy/resolution*2.f-1.f;
    vec4 ndcRayDir = vec4(uv, 1.f, 1.f);
    vec4 worldRayDir = viewInv*projectionInv*ndcRayDir;
    Ray r = {cameraPos, worldRayDir.xyz/worldRayDir.w};
    r.d = normalize(r.d);
    IntersectResult intr = intersect(r);
    if (intr.t < MAX_DIST) {
        fragColor = shadeVoxel(intr.idx, intr.intersection);
    } else {
        discard;
    }
}