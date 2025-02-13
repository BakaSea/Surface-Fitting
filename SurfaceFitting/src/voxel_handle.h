#pragma once
#include <vector>
#include "surface_fit.h"
using namespace std;

struct Voxel {
    QuadricFit fit;
    Quadric quadric;
    SGGX sggx;
    vec3 bmin, bmax;
    float density;
};

struct VoxelLayer {

    ivec3 slice;
    vector<vector<vector<Voxel>>> voxels;

    VoxelLayer(string meshFile, ivec3 slice);

    void loadFromObj(string meshFile);

    void loadFromGltf(string meshFile);

};
