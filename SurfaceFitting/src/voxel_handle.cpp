#define TINYOBJLOADER_IMPLEMENTATION
#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "voxel_handle.h"
#include <iostream>
#include <GLFW/glfw3.h>
#include "tiny_obj_loader.h"
#include "tiny_gltf.h"
#include "triangle_clip.h"

VoxelLayer::VoxelLayer(string meshFile, ivec3 slice) : slice(slice) {
    voxels = vector<vector<vector<Voxel>>>(slice[0], vector<vector<Voxel>>(slice[1], vector<Voxel>(slice[2])));
    string objSuffix = ".obj";
    if (!meshFile.compare(meshFile.size() - objSuffix.size(), objSuffix.size(), objSuffix)) {
        loadFromObj(meshFile);
    } else {
        loadFromGltf(meshFile);
    }
}

void VoxelLayer::loadFromObj(string meshFile) {
    tinyobj::ObjReaderConfig readerConfig;
    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(meshFile, readerConfig)) {
        if (!reader.Error().empty()) {
            cerr << "TinyObjReader: " << reader.Error();
        }
        exit(1);
    }
    if (!reader.Warning().empty()) {
        cout << "TinyObjReader: " << reader.Warning();
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    float scale = 2.0f;
    vec3 bbmin(INFINITY, INFINITY, INFINITY), bbmax(-INFINITY, -INFINITY, -INFINITY);

    for (size_t s = 0; s < shapes.size(); ++s) {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); ++f) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
            assert(fv == 3);
            for (size_t v = 0; v < fv; ++v) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
                tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
                tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];
                vec3 vertex = vec3(vx, vy, vz) * scale;
                bbmax = max(bbmax, vertex);
                bbmin = min(bbmin, vertex);
            }
            index_offset += fv;
        }
    }
    vec3 cap = (bbmax - bbmin) / vec3(slice);
    double startTime = glfwGetTime();
    for (int i = 0; i < slice[0]; ++i) {
        for (int j = 0; j < slice[1]; ++j) {
            for (int k = 0; k < slice[2]; ++k) {
                voxels[i][j][k].bmin = bbmin + vec3(i, j, k) * cap;
                voxels[i][j][k].bmax = bbmin + vec3(i + 1, j + 1, k + 1) * cap - vec3(1e-4f);
            }
        }
    }

    for (size_t s = 0; s < shapes.size(); ++s) {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); ++f) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
            assert(fv == 3);
            vec3 tri[3];
            for (size_t v = 0; v < fv; ++v) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
                tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
                tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];
                tri[v] = vec3(vx, vy, vz) * scale;
            }
            index_offset += fv;

            vec3 tmin = min(tri[0], min(tri[1], tri[2])), tmax = max(tri[0], max(tri[1], tri[2]));
            int xstart = std::max(int((tmin.x - bbmin.x) / cap.x), 0);
            int ystart = std::max(int((tmin.y - bbmin.y) / cap.y), 0);
            int zstart = std::max(int((tmin.z - bbmin.z) / cap.z), 0);
            int xend = std::min(int((tmax.x - bbmin.x) / cap.x), slice[0] - 1);
            int yend = std::min(int((tmax.y - bbmin.y) / cap.y), slice[1] - 1);
            int zend = std::min(int((tmax.z - bbmin.z) / cap.z), slice[2] - 1);
            for (int i = xstart; i <= xend; ++i) {
                for (int j = ystart; j <= yend; ++j) {
                    for (int k = zstart; k <= zend; ++k) {
                        vector<vec3> points = clipTriangle(tri, voxels[i][j][k].bmin, voxels[i][j][k].bmax);
                        if (points.size() >= 3) {
                            for (auto p : points) {
                                assert(inBox(p, voxels[i][j][k].bmin, voxels[i][j][k].bmax));
                            }
                            for (int p = 1; p <= points.size() - 2; ++p) {
                                vec3 clipTri[3] = { points[0], points[p], points[p + 1] };
                                voxels[i][j][k].fit.addTriangle(clipTri);
                            }
                        }
                        //voxels[i][j][k].fit.addTriangle(tri);
                    }
                }
            }
        }
    }

    ofstream out("param.txt");
    for (int i = 0; i < slice[0]; ++i) {
        for (int j = 0; j < slice[1]; ++j) {
            for (int k = 0; k < slice[2]; ++k) {
                auto& voxel = voxels[i][j][k];
                if (voxel.fit.vertices > 0) {
                    //cout << "Fit " << i << ' ' << j << ' ' << k << endl;
                    voxel.quadric = voxel.fit.fitQuadric();
                    out << voxel.bmin[0] << ' ' << voxel.bmin[1] << ' ' << voxel.bmin[2] << ' ' << voxel.bmax[0] << ' ' << voxel.bmax[1] << ' ' << voxel.bmax[2];
                    for (int q = 0; q < 10; ++q) {
                        out << ' ' << voxel.quadric.c[q];
                    }
                    out << voxel.quadric.sigma << endl;
                    voxel.sggx = voxel.fit.fitSGGX();
                }
            }
        }
    }
    double endTime = glfwGetTime();
    cout << (endTime - startTime) * 1000 << "ms" << endl;
}

void VoxelLayer::loadFromGltf(string meshFile) {
    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    string err, warn;

    string ext = tinygltf::GetFilePathExtension(meshFile);
    bool ret = false;
    if (ext.compare("glb") == 0)
        ret = loader.LoadBinaryFromFile(&model, &err, &warn, meshFile);
    else
        ret = loader.LoadASCIIFromFile(&model, &err, &warn, meshFile);
    if (!warn.empty()) {
        cout << "TinyGLTF: " << warn << endl;
    }
    if (!err.empty()) {
        cerr << "TinyGLTF: " << err << endl;
        exit(1);
    }
    if (!ret) {
        cout << "Failed to parse glTF" << endl;
        exit(1);
    }

}
