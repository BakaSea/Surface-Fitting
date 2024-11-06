#include <fstream>
#include <map>
#include <glm/glm.hpp>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "camera.h"
#include "shader_m.h"
#include "mesh.h"
#include "quadricfitting.h"
using namespace std;

struct Voxel{
    Mesh quadric;
    allquadrics::TriangleMesh mesh;
    float q[10];
    vec3 bmin, bmax;
};

vector<vector<vector<Voxel>>> voxels;

struct VoxelData {
    float q[10];
    float bmin[3];
    float bmax[3];
};

vector<VoxelData> voxelDatas;

inline bool inBox(vec3 point, vec3 bmin, vec3 bmax) {
    return min(point, bmin) == bmin && max(point, bmax) == bmax;
}

Mesh cvtMesh(allquadrics::TriangleMesh& mesh) {
    vector<Vertex> vertices;
    vector<unsigned int> indices;
    for (int i = 0; i < mesh.vertices.size(); ++i) {
        Vertex v;
        v.Position = mesh.vertices[i];
        v.Normal = mesh.normals[i];
        v.TexCoords = vec2(0, 0);
        vertices.emplace_back(v);
    }
    for (int i = 0; i < mesh.triangles.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            indices.emplace_back(mesh.triangles[i].ind[j]);
        }
    }
    return Mesh(vertices, indices, vector<Texture>());
}

void buildMesh(allquadrics::Quadric qfit, allquadrics::TriangleMesh& mesh, vec3 bmin, vec3 bmax, int samples) {
    mesh.clear();
    vector<vector<double>> upSurface(samples, vector<double>(samples, INFINITY)), downSurface(samples, vector<double>(samples, INFINITY));
    double gapx = (bmax[0] - bmin[0]) / (samples - 1.f), gapy = (bmax[1] - bmin[1]) / (samples - 1.f);
    double x = bmin[0], y = bmin[1];
    for (int i = 0; i < samples; i++, x += gapx) {
        y = bmin[1];
        for (int j = 0; j < samples; j++, y += gapy) {
            double a = qfit.q[9];
            double b = qfit.q[6] * x + qfit.q[8] * y + qfit.q[3];
            double c = qfit.q[0] + qfit.q[1] * x + qfit.q[2] * y + qfit.q[4] * x * x + qfit.q[5] * x * y + qfit.q[7] * y * y;
            double delta = b * b - 4 * a * c;
            if (delta >= 0) {
                double z = (-b + sqrt(delta)) / (2 * a);
                upSurface[i][j] = z;
                z = (-b - sqrt(delta)) / (2 * a);
                downSurface[i][j] = z;
            }
        }
    }
    x = bmin[0];
    for (int i = 0; i < samples - 1; ++i, x += gapx) {
        y = bmin[1];
        for (int j = 0; j < samples - 1; ++j, y += gapy) {
            {
                vec3 p0(x, y, upSurface[i][j]), p1(x + gapx, y, upSurface[i + 1][j]), p2(x, y + gapy, upSurface[i][j + 1]), p3(x + gapx, y + gapy, upSurface[i + 1][j + 1]);
                if (inBox(p0, bmin, bmax) && inBox(p1, bmin, bmax) && inBox(p2, bmin, bmax) && inBox(p3, bmin, bmax)) {
                    mesh.vertices.emplace_back(p0);
                    mesh.vertices.emplace_back(p1);
                    mesh.vertices.emplace_back(p2);
                    mesh.vertices.emplace_back(p3);
                    mesh.normals.emplace_back(normalize(qfit.df(p0)));
                    mesh.normals.emplace_back(normalize(qfit.df(p1)));
                    mesh.normals.emplace_back(normalize(qfit.df(p2)));
                    mesh.normals.emplace_back(normalize(qfit.df(p3)));
                    int last = mesh.vertices.size() - 1;
                    mesh.triangles.emplace_back(last - 3, last - 2, last - 1);
                    mesh.triangles.emplace_back(last - 2, last - 1, last);
                }
                else if (inBox(p0, bmin, bmax) && inBox(p1, bmin, bmax) && inBox(p2, bmin, bmax)) {
                    mesh.vertices.emplace_back(p0);
                    mesh.vertices.emplace_back(p1);
                    mesh.vertices.emplace_back(p2);
                    mesh.normals.emplace_back(normalize(qfit.df(p0)));
                    mesh.normals.emplace_back(normalize(qfit.df(p1)));
                    mesh.normals.emplace_back(normalize(qfit.df(p2)));
                    int last = mesh.vertices.size() - 1;
                    mesh.triangles.emplace_back(last - 2, last - 1, last);
                }
                else if (inBox(p0, bmin, bmax) && inBox(p1, bmin, bmax) && inBox(p3, bmin, bmax)) {
                    mesh.vertices.emplace_back(p0);
                    mesh.vertices.emplace_back(p1);
                    mesh.vertices.emplace_back(p3);
                    mesh.normals.emplace_back(normalize(qfit.df(p0)));
                    mesh.normals.emplace_back(normalize(qfit.df(p1)));
                    mesh.normals.emplace_back(normalize(qfit.df(p3)));
                    int last = mesh.vertices.size() - 1;
                    mesh.triangles.emplace_back(last - 2, last - 1, last);
                }
                else if (inBox(p0, bmin, bmax) && inBox(p2, bmin, bmax) && inBox(p3, bmin, bmax)) {
                    mesh.vertices.emplace_back(p0);
                    mesh.vertices.emplace_back(p2);
                    mesh.vertices.emplace_back(p3);
                    mesh.normals.emplace_back(normalize(qfit.df(p0)));
                    mesh.normals.emplace_back(normalize(qfit.df(p2)));
                    mesh.normals.emplace_back(normalize(qfit.df(p3)));
                    int last = mesh.vertices.size() - 1;
                    mesh.triangles.emplace_back(last - 2, last - 1, last);
                }
                else if (inBox(p1, bmin, bmax) && inBox(p2, bmin, bmax) && inBox(p3, bmin, bmax)) {
                    mesh.vertices.emplace_back(p1);
                    mesh.vertices.emplace_back(p2);
                    mesh.vertices.emplace_back(p3);
                    mesh.normals.emplace_back(normalize(qfit.df(p1)));
                    mesh.normals.emplace_back(normalize(qfit.df(p2)));
                    mesh.normals.emplace_back(normalize(qfit.df(p3)));
                    int last = mesh.vertices.size() - 1;
                    mesh.triangles.emplace_back(last - 2, last - 1, last);
                }
            }
            {
                vec3 p0(x, y, downSurface[i][j]), p1(x + gapx, y, downSurface[i + 1][j]), p2(x, y + gapy, downSurface[i][j + 1]), p3(x + gapx, y + gapy, downSurface[i + 1][j + 1]);
                if (inBox(p0, bmin, bmax) && inBox(p1, bmin, bmax) && inBox(p2, bmin, bmax) && inBox(p3, bmin, bmax)) {
                    mesh.vertices.emplace_back(p0);
                    mesh.vertices.emplace_back(p1);
                    mesh.vertices.emplace_back(p2);
                    mesh.vertices.emplace_back(p3);
                    mesh.normals.emplace_back(normalize(qfit.df(p0)));
                    mesh.normals.emplace_back(normalize(qfit.df(p1)));
                    mesh.normals.emplace_back(normalize(qfit.df(p2)));
                    mesh.normals.emplace_back(normalize(qfit.df(p3)));
                    int last = mesh.vertices.size() - 1;
                    mesh.triangles.emplace_back(last - 3, last - 2, last - 1);
                    mesh.triangles.emplace_back(last - 2, last - 1, last);
                }
                else if (inBox(p0, bmin, bmax) && inBox(p1, bmin, bmax) && inBox(p2, bmin, bmax)) {
                    mesh.vertices.emplace_back(p0);
                    mesh.vertices.emplace_back(p1);
                    mesh.vertices.emplace_back(p2);
                    mesh.normals.emplace_back(normalize(qfit.df(p0)));
                    mesh.normals.emplace_back(normalize(qfit.df(p1)));
                    mesh.normals.emplace_back(normalize(qfit.df(p2)));
                    int last = mesh.vertices.size() - 1;
                    mesh.triangles.emplace_back(last - 2, last - 1, last);
                }
                else if (inBox(p0, bmin, bmax) && inBox(p1, bmin, bmax) && inBox(p3, bmin, bmax)) {
                    mesh.vertices.emplace_back(p0);
                    mesh.vertices.emplace_back(p1);
                    mesh.vertices.emplace_back(p3);
                    mesh.normals.emplace_back(normalize(qfit.df(p0)));
                    mesh.normals.emplace_back(normalize(qfit.df(p1)));
                    mesh.normals.emplace_back(normalize(qfit.df(p3)));
                    int last = mesh.vertices.size() - 1;
                    mesh.triangles.emplace_back(last - 2, last - 1, last);
                }
                else if (inBox(p0, bmin, bmax) && inBox(p2, bmin, bmax) && inBox(p3, bmin, bmax)) {
                    mesh.vertices.emplace_back(p0);
                    mesh.vertices.emplace_back(p2);
                    mesh.vertices.emplace_back(p3);
                    mesh.normals.emplace_back(normalize(qfit.df(p0)));
                    mesh.normals.emplace_back(normalize(qfit.df(p2)));
                    mesh.normals.emplace_back(normalize(qfit.df(p3)));
                    int last = mesh.vertices.size() - 1;
                    mesh.triangles.emplace_back(last - 2, last - 1, last);
                }
                else if (inBox(p1, bmin, bmax) && inBox(p2, bmin, bmax) && inBox(p3, bmin, bmax)) {
                    mesh.vertices.emplace_back(p1);
                    mesh.vertices.emplace_back(p2);
                    mesh.vertices.emplace_back(p3);
                    mesh.normals.emplace_back(normalize(qfit.df(p1)));
                    mesh.normals.emplace_back(normalize(qfit.df(p2)));
                    mesh.normals.emplace_back(normalize(qfit.df(p3)));
                    int last = mesh.vertices.size() - 1;
                    mesh.triangles.emplace_back(last - 2, last - 1, last);
                }
            }
        }
    }
}

vector<vec3> clipTriangle(const vector<vec3>& triangle, vec3 bmin, vec3 bmax) {
    vector<vec3> res(triangle), p;
    for (int k = 0; k < 3; ++k) {
        p = res;
        res.clear();
        for (int i = 0; i < p.size(); ++i) {
            vec3 e = p[(i + 1) % p.size()] - p[i];
            if (e[k] != 0) {
                float t = (bmin[k] - p[i][k]) / e[k];
                if (0 < t && t < 1) {
                    res.emplace_back(p[i] + t * e);
                }
            }
            if (p[(i + 1) % p.size()][k] >= bmin[k]) {
                res.emplace_back(p[(i + 1) % p.size()]);
            }
        }
        p = res;
        res.clear();
        for (int i = 0; i < p.size(); ++i) {
            vec3 e = p[(i + 1) % p.size()] - p[i];
            if (e[k] != 0) {
                float t = (bmax[k] - p[i][k]) / e[k];
                if (0 < t && t < 1) {
                    res.emplace_back(p[i] + t * e);
                }
            }
            if (p[(i + 1) % p.size()][k] <= bmax[k]) {
                res.emplace_back(p[(i + 1) % p.size()]);
            }
        }
    }
    return res;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;

int main(int argc, char **argv) {
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Test", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // build and compile our shader zprogram
    // ------------------------------------
    Shader raytraceShader("shader/raytrace.vs", "shader/raytrace.fs");

    allquadrics::TriangleMesh inputMesh, quadricMesh;
    
    const char *defaultInput = "../example_data/default.obj";
    const char *meshfile = argc > 1 ? argv[1] : 0;
    if (!meshfile) {
    	cerr << "No mesh file specified?  Loading default mesh: " << defaultInput << endl;
    	meshfile = defaultInput;
    }
    if (meshfile[strlen(meshfile) - 1] == 't') {
        ifstream in(meshfile);
        vector<allquadrics::data_pnw> data;
        while (!in.eof()) {
            allquadrics::data_pnw pnw;
            in >> pnw.p[0] >> pnw.p[1] >> pnw.p[2];
            pnw.n = vec3(0, 0, 0);
            pnw.w = 1;
            data.push_back(pnw);
        }
        allquadrics::Quadric qfit;
        allquadrics::fitEllipsoid(data, qfit);
        ofstream out("ez_param.txt");
        for (int i = 0; i < 10; ++i) {
            out << qfit.q[i] << ' ';
        }
    }
    if (!inputMesh.loadObj(meshfile)) {
        cerr << "Couldn't load file " << meshfile << endl;
        return 1;
    }
    int xslice = 1, yslice = 1, zslice = 1;
    if (argc > 2) xslice = atoi(argv[2]);
    if (argc > 3) yslice = atoi(argv[3]);
    if (argc > 4) zslice = atoi(argv[4]);
    voxels = vector<vector<vector<Voxel>>>(xslice, vector<vector<Voxel>>(yslice, vector<Voxel>(zslice)));
    	
    // Always recenter and scale your data before fitting!
    //inputMesh.centerAndScale(1);
    inputMesh.triangleTags.resize(inputMesh.triangles.size(), 0);
    inputMesh.activeTag = 1;
    
    ofstream out("param.txt");
    vec3 slide(xslice, yslice, zslice);
    vec3 bbmin(INFINITY, INFINITY, INFINITY), bbmax(-INFINITY, -INFINITY, -INFINITY);
    for (int i = 0; i < inputMesh.vertices.size(); ++i) {
        bbmin = min(bbmin, vec3(inputMesh.vertices[i]));
        bbmax = max(bbmax, vec3(inputMesh.vertices[i]));
    }
    vec3 cap = bbmax - bbmin;
    cap[0] /= (float)xslice; cap[1] /= (float)yslice; cap[2] /= (float)zslice;
    double startTime = glfwGetTime();
    for (int i = 0; i < xslice; ++i) {
        for (int j = 0; j < yslice; ++j) {
            for (int k = 0; k < zslice; ++k) {
                voxels[i][j][k].bmin = bbmin + vec3(i, j, k) * cap;
                voxels[i][j][k].bmax = bbmin + vec3(i + 1, j + 1, k + 1) * cap;
                voxels[i][j][k].mesh.clear();
            }
        }
    }
    for (const auto& triangle : inputMesh.triangles) {
        vec3 v0 = inputMesh.vertices[triangle.ind[0]];
        vec3 v1 = inputMesh.vertices[triangle.ind[1]];
        vec3 v2 = inputMesh.vertices[triangle.ind[2]];
        vector<vec3> tri = { v0, v1, v2 };
        vec3 tmin = min(v0, min(v1, v2)), tmax = max(v0, max(v1, v2));
        int xstart = std::max(int((tmin.x - bbmin.x) / cap.x), 0);
        int ystart = std::max(int((tmin.y - bbmin.y) / cap.y), 0);
        int zstart = std::max(int((tmin.z - bbmin.z) / cap.z), 0);
        int xend = std::min(int((tmax.x - bbmin.x) / cap.x), xslice - 1);
        int yend = std::min(int((tmax.y - bbmin.y) / cap.y), yslice - 1);
        int zend = std::min(int((tmax.z - bbmin.z) / cap.z), zslice - 1);
        for (int i = xstart; i <= xend; ++i) {
            for (int j = ystart; j <= yend; ++j) {
                for (int k = zstart; k <= zend; ++k) {
                    vector<vec3> points = clipTriangle(tri, voxels[i][j][k].bmin, voxels[i][j][k].bmax);
                    if (points.size() >= 3) {
                        int vs = voxels[i][j][k].mesh.vertices.size();
                        for (int p = 0; p < points.size(); ++p) {
                            voxels[i][j][k].mesh.vertices.emplace_back(points[p]);
                        }
                        for (int p = 1; p <= points.size() - 2; ++p) {
                            voxels[i][j][k].mesh.triangles.emplace_back(allquadrics::Tri(vs + 0, vs + p, vs + p + 1));
                        }
                    }
                }
            }
        }
    }
    for (int i = 0; i < xslice; ++i) {
        for (int j = 0; j < yslice; ++j) {
            for (int k = 0; k < zslice; ++k) {
                auto& voxel = voxels[i][j][k];
                if (!voxel.mesh.triangles.empty()) {
                    voxel.mesh.computeNormals();
                    allquadrics::Quadric qfit;
                    allquadrics::fitEllipsoid(voxel.mesh, qfit);
                    out << voxel.bmin[0] << ' ' << voxel.bmin[1] << ' ' << voxel.bmin[2] << ' ' << voxel.bmax[0] << ' ' << voxel.bmax[1] << ' ' << voxel.bmax[2];
                    voxel.q[0] = qfit.q[4];
                    voxel.q[1] = qfit.q[7];
                    voxel.q[2] = qfit.q[9];
                    voxel.q[3] = qfit.q[5];
                    voxel.q[4] = qfit.q[6];
                    voxel.q[5] = qfit.q[8];
                    voxel.q[6] = qfit.q[1];
                    voxel.q[7] = qfit.q[2];
                    voxel.q[8] = qfit.q[3];
                    voxel.q[9] = qfit.q[0];
                    for (int q = 0; q < 10; ++q) {
                        out << ' ' << voxel.q[q];
                    }
                    out << endl;
                }
            }
        }
    }
    double endTime = glfwGetTime();
    cout << (endTime - startTime) * 1000 << "ms" << endl;

    for (int i = 0; i < xslice; ++i) {
        for (int j = 0; j < yslice; ++j) {
            for (int k = 0; k < zslice; ++k) {
                auto& voxel = voxels[i][j][k];
                if (!voxel.mesh.triangles.empty()) {
                    VoxelData vd;
                    memcpy(vd.q, voxel.q, sizeof(voxel.q));
                    for (int d = 0; d < 3; ++d) {
                        vd.bmin[d] = voxel.bmin[d];
                        vd.bmax[d] = voxel.bmax[d];
                    }
                    voxelDatas.emplace_back(vd);
                }
            }
        }
    }
    GLuint voxelDatasBuffer;
    glCreateBuffers(1, &voxelDatasBuffer);
    glNamedBufferStorage(voxelDatasBuffer, sizeof(VoxelData)* voxelDatas.size(), (const void*)voxelDatas.data(), GL_DYNAMIC_STORAGE_BIT);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, voxelDatasBuffer);
    GLuint vao;
    glGenVertexArrays(1, &vao);
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    const static GLfloat vertices[] = {
        -1.0f,  1.0f,  1.0f,  1.0f,  1.0f, -1.0f,
        1.0f,  -1.0f, -1.0f, -1.0f, -1.0f,  1.0f };
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glBindVertexArray(0);

    // render loop
    // -----------
    while (!glfwWindowShouldClose(window)) {
        // per-frame time logic
        // --------------------
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);

        // render
        // ------
        glClearColor(.5f, .7f, 1, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // don't forget to enable shader before setting uniforms
        raytraceShader.use();

        //// view/projection transformations
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();

        raytraceShader.setMat4("viewInv", inverse(view));
        raytraceShader.setMat4("projectionInv", inverse(projection));
        raytraceShader.setVec3("cameraPos", camera.Position);
        raytraceShader.setVec2("resolution", vec2(SCR_WIDTH, SCR_HEIGHT));
        raytraceShader.setInt("voxelSize", voxelDatas.size());
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindVertexArray(0);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}


// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}
