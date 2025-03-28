#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "surface_fit.h"
using namespace std;
using namespace glm;

struct Voxel {
	QuadricFit fit;
	Quadric quadric;
	SGGX sggx;
	vec3 bmin, bmax;
	float alpha = 0.f;
};

struct OctreeNode {
	vec3 bmin;
	float padding;
	vec3 bmax;
	int children[2][2][2];
	int voxelId = -1;

	OctreeNode() : bmin(0.f), bmax(0.f) {
		for (int i = 0; i < 2; ++i) {
			for (int j = 0; j < 2; ++j) {
				for (int k = 0; k < 2; ++k) {
					children[i][j][k] = -1;
				}
			}
		}
	}
};

struct Octree {
	vector<OctreeNode> nodes;
	vector<Voxel> voxels;
	int layers;

	Voxel& getVoxel(int layer, vec3 center);
};

Octree buildOctree(vec3 bmin, vec3 bmax, int layers);
