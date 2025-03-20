#pragma once
#include <vector>
#include "surface_fit.h"
#include "mesh.h"
#include "tiny_gltf.h"
using namespace std;

struct Voxel {
    QuadricFit fit;
    Quadric quadric;
    SGGX sggx;
    vec3 bmin, bmax;
    float alpha;
};

struct VoxelLayer {

    ivec3 slice;
    vector<vector<vector<Voxel>>> voxels;
    vector<Mesh> meshes;

    VoxelLayer(string meshFile, int s);

    void loadFromObj(string meshFile, int s);

    void loadFromGltf(string meshFile, int s);

    void processMesh(tinygltf::Model& model, tinygltf::Mesh& mesh, mat4 M);

    void processNode(tinygltf::Model& model, tinygltf::Node& node, mat4 M);

    void handleMeshes(int s);

};
