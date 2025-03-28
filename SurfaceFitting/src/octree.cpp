#include "octree.h"

Voxel& Octree::getVoxel(int layer, vec3 center) {
	int u = 0;
	for (int i = 0; i < layer; ++i) {
		vec3 cap = (nodes[u].bmax - nodes[u].bmin) / 2.f;
		vec3 offset = center - nodes[u].bmin;
		ivec3 idx = offset / cap;
		if (nodes[u].children[idx.x][idx.y][idx.z] == -1) {
			OctreeNode v;
			v.bmin = nodes[u].bmin + vec3(idx) * cap;
			v.bmax = v.bmin + cap;
			nodes.push_back(v);
			nodes[u].children[idx.x][idx.y][idx.z] = nodes.size() - 1;
		}
		u = nodes[u].children[idx.x][idx.y][idx.z];
	}
	if (nodes[u].voxelId == -1) {
		Voxel voxel;
		voxel.bmin = nodes[u].bmin;
		voxel.bmax = nodes[u].bmax;
		voxels.push_back(voxel);
		nodes[u].voxelId = voxels.size() - 1;
	}
	return voxels[nodes[u].voxelId];
}

Octree buildOctree(vec3 bmin, vec3 bmax, int layers) {
	Octree octree;
	OctreeNode root;
	root.bmin = bmin;
	root.bmax = bmax;
	octree.nodes.push_back(root);
	octree.layers = layers;
	return octree;
}
