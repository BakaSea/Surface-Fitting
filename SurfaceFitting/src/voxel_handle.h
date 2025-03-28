#pragma once
#include <vector>
#include "mesh.h"
#include "tiny_gltf.h"
#include "octree.h"
using namespace std;

struct VoxelLayer {

    ivec3 slice;
    //vector<vector<vector<Voxel>>> voxels;
    Octree octree;

    vector<Mesh> meshes;

    VoxelLayer(string meshFile, int s);

    void loadFromObj(string meshFile, int s);

    void loadFromGltf(string meshFile, int s);

    void processMesh(tinygltf::Model& model, tinygltf::Mesh& mesh, mat4 M);

    void processNode(tinygltf::Model& model, tinygltf::Node& node, mat4 M);

    void handleMeshes(int s);

};
