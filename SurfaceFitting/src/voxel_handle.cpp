#define TINYOBJLOADER_IMPLEMENTATION
#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "voxel_handle.h"
#include <iostream>
#include <GLFW/glfw3.h>
#include <glm/ext/matrix_transform.hpp>
#include "tiny_obj_loader.h"
#include "triangle_clip.h"

VoxelLayer::VoxelLayer(string meshFile, int s) {
    //voxels = vector<vector<vector<Voxel>>>(slice[0], vector<vector<Voxel>>(slice[1], vector<Voxel>(slice[2])));
    string objSuffix = ".obj";
    if (!meshFile.compare(meshFile.size() - objSuffix.size(), objSuffix.size(), objSuffix)) {
        loadFromObj(meshFile, s);
    } else {
        loadFromGltf(meshFile, s);
    }
}

void VoxelLayer::loadFromObj(string meshFile, int s) {
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
    float scale = 1.0f;

    for (size_t s = 0; s < shapes.size(); ++s) {
        vector<unsigned int> indices(shapes[s].mesh.indices.size());
        vector<Vertex> vertices(attrib.vertices.size() / 3);
        vector<float> weight(vertices.size());
        for (int i = 0; i < shapes[s].mesh.indices.size(); ++i) {
            indices[i] = shapes[s].mesh.indices[i].vertex_index;
        }
        for (int i = 0; i < vertices.size(); ++i) {
            vertices[i].Position = scale * vec3(attrib.vertices[3 * i], attrib.vertices[3 * i + 1], attrib.vertices[3 * i + 2]);
        }
        for (int i = 0; i < indices.size(); i += 3) {
            auto& v0 = vertices[indices[i]], & v1 = vertices[indices[i + 1]], & v2 = vertices[indices[i + 2]];
            vec3 e1 = v1.Position - v0.Position, e2 = v2.Position - v0.Position;
            vec3 n = cross(e1, e2);
            float area = n.length() / 2.f;
            n = normalize(n);
            v0.Normal += area * n;
            v1.Normal += area * n;
            v2.Normal += area * n;
            weight[indices[i]] += area;
            weight[indices[i + 1]] += area;
            weight[indices[i + 2]] += area;
        }
        for (int i = 0; i < vertices.size(); ++i) {
            vertices[i].Normal /= weight[i];
        }
        meshes.push_back(Mesh(vertices, indices, {}));
    }

    handleMeshes(s);
}

void VoxelLayer::processMesh(tinygltf::Model& model, tinygltf::Mesh& mesh, mat4 M) {
    for (auto& primitive : mesh.primitives) {
        assert(primitive.mode == TINYGLTF_MODE_TRIANGLES);
        auto& idxAccessor = model.accessors[primitive.indices];
        assert(idxAccessor.type == TINYGLTF_TYPE_SCALAR);
        assert(idxAccessor.count % 3 == 0);
        auto& idxBufferView = model.bufferViews[idxAccessor.bufferView];
        auto& idxBuffer = model.buffers[idxBufferView.buffer];
        tinygltf::Accessor* posAccessor = nullptr, * normalAccessor = nullptr;
        tinygltf::BufferView* posBufferView = nullptr, * normalBufferView = nullptr;
        tinygltf::Buffer* posBuffer = nullptr, * normalBuffer = nullptr;
        for (auto& attrib : primitive.attributes) {
            auto& accessor = model.accessors[attrib.second];
            if (!attrib.first.compare("POSITION")) {
                posAccessor = &accessor;
                assert(posAccessor->type == TINYGLTF_TYPE_VEC3);
                assert(posAccessor->componentType == TINYGLTF_COMPONENT_TYPE_FLOAT);
                posBufferView = &model.bufferViews[accessor.bufferView];
                posBuffer = &model.buffers[posBufferView->buffer];
            }
            else if (!attrib.first.compare("NORMAL")) {
                normalAccessor = &accessor;
                assert(normalAccessor->type == TINYGLTF_TYPE_VEC3);
                assert(normalAccessor->componentType == TINYGLTF_COMPONENT_TYPE_FLOAT);
                normalBufferView = &model.bufferViews[accessor.bufferView];
                normalBuffer = &model.buffers[normalBufferView->buffer];
            }
        }
        assert(posAccessor != nullptr);
        vector<unsigned int> indices(idxAccessor.count);
        int byteStride = idxAccessor.ByteStride(idxBufferView);
        for (int i = 0; i < idxAccessor.count; ++i) {
            if (idxAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) {
                unsigned short temp = *((unsigned short*)(&idxBuffer.data.at(0) + idxBufferView.byteOffset + idxAccessor.byteOffset + i * byteStride));
                indices[i] = temp;
            } else {
                unsigned int temp = *((unsigned int*)(&idxBuffer.data.at(0) + idxBufferView.byteOffset + idxAccessor.byteOffset + i * byteStride));
                indices[i] = temp;
            }
        }
        vector<Vertex> vertices(posAccessor->count);
        byteStride = posAccessor->ByteStride(*posBufferView);
        for (int i = 0; i < posAccessor->count; ++i) {
            vec3 temp = *((vec3*)(&posBuffer->data.at(0) + posBufferView->byteOffset + posAccessor->byteOffset + i * byteStride));
            vec4 pos = M * vec4(temp, 1.f);
            pos /= pos.w;
            vertices[i].Position = pos;
        }
        if (normalAccessor != nullptr) {
            assert(normalAccessor->count == posAccessor->count);
            byteStride = normalAccessor->ByteStride(*normalBufferView);
            mat4 tiM = transpose(inverse(M));
            for (int i = 0; i < normalAccessor->count; ++i) {
                vec3 temp = *((vec3*)(&normalBuffer->data.at(0) + normalBufferView->byteOffset + normalAccessor->byteOffset + i * byteStride));
                vec4 normal = tiM * vec4(temp, 1.f);
                normal /= normal.w;
                vertices[i].Normal = normal;
            }
        }
        meshes.push_back(Mesh(vertices, indices, {}));
    }
}

void VoxelLayer::processNode(tinygltf::Model& model, tinygltf::Node& node, mat4 M) {
    if (!node.matrix.empty()) {
        dmat4 N;
        memcpy(&N, node.matrix.data(), sizeof(N));
        M *= mat4(N);
    }
    if (!node.translation.empty()) {
        M = translate(M, vec3(node.translation[0], node.translation[1], node.translation[2]));
    }
    if (!node.rotation.empty()) {
        float angle;
        vec3 axis;

        float angleRadians = 2.f * acos(node.rotation[3]);
        if (angleRadians == 0.f) {
            axis = vec3(0, 0, 1);
        } else {
            float denom = sqrt(1.f - node.rotation[3] * node.rotation[3]);
            axis = vec3(node.rotation[0], node.rotation[1], node.rotation[2]) / denom;
        }

        M = rotate(M, angleRadians, axis);
    }
    if (!node.scale.empty()) {
        M = scale(M, vec3(node.scale[0], node.scale[1], node.scale[2]));
    }
    if (0 <= node.mesh && node.mesh < model.meshes.size()) {
        processMesh(model, model.meshes[node.mesh], M);
    }
    for (int child : node.children) {
        processNode(model, model.nodes[child], M);
    }
}

void VoxelLayer::loadFromGltf(string meshFile, int s) {
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

    auto& scene = model.scenes[model.defaultScene];
    for (int node : scene.nodes) {
        processNode(model, model.nodes[node], mat4(1.f));
    }

    handleMeshes(s);
}

void VoxelLayer::handleMeshes(int s) {
    vec3 bbmin(INFINITY, INFINITY, INFINITY), bbmax(-INFINITY, -INFINITY, -INFINITY);
    for (auto& mesh : meshes) {
        for (int i = 0; i < mesh.vertices.size(); ++i) {
            bbmax = max(bbmax, mesh.vertices[i].Position);
            bbmin = min(bbmin, mesh.vertices[i].Position);
        }
    }

    vec3 cap = (bbmax - bbmin) / float(s);
    float minCap = std::min(cap.x, std::min(cap.y, cap.z));
    cap = vec3(minCap);
    for (int i = 0; i < 3; ++i) {
        slice[i] = ceil((bbmax[i] - bbmin[i]) / cap[i]);
    }
    cout << slice.x << ' ' << slice.y << ' ' << slice.z << endl;
    int maxSlice = std::max(slice.x, std::max(slice.y, slice.z));
    int layers = ceil(std::log2(maxSlice));
    slice = ivec3(1 << layers);
    bbmax = bbmin + vec3(slice) * cap;
    octree = buildOctree(bbmin, bbmax, layers);
    cout << slice.x << ' ' << slice.y << ' ' << slice.z << endl;
    //voxels = vector<vector<vector<Voxel>>>(slice.x, vector<vector<Voxel>>(slice.y, vector<Voxel>(slice.z)));

    double startTime = glfwGetTime();
    //for (int i = 0; i < slice[0]; ++i) {
    //    for (int j = 0; j < slice[1]; ++j) {
    //        for (int k = 0; k < slice[2]; ++k) {
    //            /*voxels[i][j][k].fit.bmin = */voxels[i][j][k].bmin = bbmin + vec3(i, j, k) * cap;
    //            /*voxels[i][j][k].fit.bmax = */voxels[i][j][k].bmax = bbmin + vec3(i + 1, j + 1, k + 1) * cap - vec3(1e-4f);
    //        }
    //    }
    //}

    for (auto& mesh : meshes) {
        for (int t = 0; t < mesh.indices.size(); t += 3) {
            vec3 tri[3] = { mesh.vertices[mesh.indices[t]].Position, mesh.vertices[mesh.indices[t + 1]].Position, mesh.vertices[mesh.indices[t + 2]].Position };
            vec3 tmin = min(tri[0], min(tri[1], tri[2])), tmax = max(tri[0], max(tri[1], tri[2]));
            vec3 e1 = tri[1] - tri[0], e2 = tri[2] - tri[0];
            vec3 n = normalize(cross(e1, e2));
            int xstart = std::max(int((tmin.x - bbmin.x) / cap.x), 0);
            int ystart = std::max(int((tmin.y - bbmin.y) / cap.y), 0);
            int zstart = std::max(int((tmin.z - bbmin.z) / cap.z), 0);
            int xend = std::min(int((tmax.x - bbmin.x) / cap.x), slice[0] - 1);
            int yend = std::min(int((tmax.y - bbmin.y) / cap.y), slice[1] - 1);
            int zend = std::min(int((tmax.z - bbmin.z) / cap.z), slice[2] - 1);
            for (int i = xstart; i <= xend; ++i) {
                for (int j = ystart; j <= yend; ++j) {
                    for (int k = zstart; k <= zend; ++k) {
                        vec3 bmin = bbmin + vec3(i, j, k) * cap;
                        vec3 bmax = bmin + cap;
                        //vector<vec3> points = clipTriangle(tri, voxels[i][j][k].bmin, voxels[i][j][k].bmax);
                        vector<vec3> points = clipTriangle(tri, bmin, bmax);
                        if (points.size() >= 3) {
                            for (auto p : points) {
                                //assert(inBox(p, voxels[i][j][k].bmin, voxels[i][j][k].bmax));
                                assert(inBox(p, bmin, bmax));
                            }
                            for (int p = 1; p <= points.size() - 2; ++p) {
                                vec3 clipTri[3] = { points[0], points[p], points[p + 1] };
                                octree.getVoxel(layers, (bmin + bmax) / 2.f).fit.addTriangle(clipTri, vec3(.7f, .6f, .5f), .5f, .04f, .3f);
                                //voxels[i][j][k].fit.addTriangle(clipTri);
                            }
                        }
                        //voxels[i][j][k].fit.addTriangle(tri);
                    }
                }
            }
        }
    }

    ofstream out("param.txt");
    //for (int i = 0; i < slice[0]; ++i) {
    //    for (int j = 0; j < slice[1]; ++j) {
    //        for (int k = 0; k < slice[2]; ++k) {
    //            auto& voxel = voxels[i][j][k];
    //            if (voxel.fit.vertices > 0) {
    //                //cout << "Fit " << i << ' ' << j << ' ' << k << endl;
    //                voxel.quadric = voxel.fit.fitQuadric();
    //                out << voxel.bmin[0] << ' ' << voxel.bmin[1] << ' ' << voxel.bmin[2] << ' ' << voxel.bmax[0] << ' ' << voxel.bmax[1] << ' ' << voxel.bmax[2];
    //                for (int q = 0; q < 10; ++q) {
    //                    out << ' ' << voxel.quadric.c[q];
    //                }
    //                out << ' ' << voxel.quadric.sigma;
    //                //voxel.quadric.sigma = 1e-5f;
    //                //voxel.sggx = voxel.fit.fitSGGX();
    //                voxel.alpha = voxel.fit.fitAlpha(voxel.quadric, voxel.bmin, voxel.bmax);
    //                //voxel.alpha = 1.f;
    //                out << ' ' << voxel.alpha << endl;
    //                voxel.sggx = voxel.fit.fitSGGX(voxel.quadric);
    //                //cout << voxel.sggx.S_xx << ' ' << voxel.sggx.S_yy << ' ' << voxel.sggx.S_zz << endl;
    //            }
    //        }
    //    }
    //}

    for (Voxel& voxel : octree.voxels) {
        if (voxel.fit.vertices > 0) {
            //cout << "Fit " << i << ' ' << j << ' ' << k << endl;
            voxel.quadric = voxel.fit.fitQuadric();
            out << voxel.bmin[0] << ' ' << voxel.bmin[1] << ' ' << voxel.bmin[2] << ' ' << voxel.bmax[0] << ' ' << voxel.bmax[1] << ' ' << voxel.bmax[2];
            for (int q = 0; q < 10; ++q) {
                out << ' ' << voxel.quadric.c[q];
            }
            out << ' ' << voxel.quadric.sigma;
            //voxel.quadric.sigma = 1e-5f;
            //voxel.sggx = voxel.fit.fitSGGX();
            voxel.alpha = voxel.fit.fitAlpha(voxel.quadric, voxel.bmin, voxel.bmax);
            //voxel.alpha = 1.f;
            out << ' ' << voxel.alpha << endl;
            voxel.sggx = voxel.fit.fitSGGX(voxel.quadric);
            voxel.Ed = voxel.fit.fitEd();
            voxel.Es = voxel.fit.fitEs();
            //voxel.sggx.S_xx = voxel.sggx.S_yy = 0.3f*0.3f;
            //voxel.sggx.S_zz = 1.f;
            //cout << voxel.sggxD.S_xx << ' ' << voxel.sggxD.S_yy << ' ' << voxel.sggxD.S_zz << endl;
        }
    }

    double endTime = glfwGetTime();
    cout << (endTime - startTime) * 1000 << "ms" << endl;
}