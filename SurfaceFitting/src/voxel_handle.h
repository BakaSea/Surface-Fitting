#pragma once
#include <vector>
#include "surface_fit.h"
using namespace std;

struct Voxel {
    QuadricFit fit;
    Quadric quadric;
    vec3 bmin, bmax;
};

struct VoxelLayer {

    ivec3 slice;
    vector<vector<vector<Voxel>>> voxels;

    VoxelLayer(string meshFile, ivec3 slice);

};
