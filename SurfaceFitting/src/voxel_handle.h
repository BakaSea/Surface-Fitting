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
    float density;
};

struct VoxelLayer {

    ivec3 slice;
    vector<vector<vector<Voxel>>> voxels;
    vector<Mesh> meshes;

    VoxelLayer(string meshFile, ivec3 slice);

    void loadFromObj(string meshFile);

    void loadFromGltf(string meshFile);

    void processMesh(tinygltf::Model& model, tinygltf::Mesh& mesh, mat4 M);

    void processNode(tinygltf::Model& model, tinygltf::Node& node, mat4 M);

    void handleMeshes();

};
