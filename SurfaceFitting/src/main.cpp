#define TINYOBJLOADER_IMPLEMENTATION
#include <fstream>
#include <map>
#include <vector>
#include <span>
#include <glm/glm.hpp>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "tiny_obj_loader.h"

#include "camera.h"
#include "shader_m.h"
#include "surface_fit.h"
#include "triangle_clip.h"
//#include "mesh.h"
//#include "quadricfitting.h"
//#include "ellipsoidhull.h"
using namespace std;

struct Voxel{
    QuadricFit fit;
    Quadric quadric;
    vec3 bmin, bmax;
    int vertices;
};

vector<vector<vector<Voxel>>> voxels;

struct VoxelData {
    float q[10];
    //float hullQ[10];
    float bmin[3];
    float bmax[3];
    float sigma;
};

vector<VoxelData> voxelDatas;

inline bool inBox(vec3 point, vec3 bmin, vec3 bmax) {
    return min(point, bmin) == bmin && max(point, bmax) == bmax;
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
    
    const char *defaultInput = "../example_data/default.obj";
    const char *meshfile = argc > 1 ? argv[1] : 0;
    if (!meshfile) {
    	cerr << "No mesh file specified?  Loading default mesh: " << defaultInput << endl;
    	meshfile = defaultInput;
    }
    tinyobj::ObjReaderConfig readerConfig;
    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(meshfile, readerConfig)) {
        if (!reader.Error().empty()) {
            cerr << "TinyObjReader: " << reader.Error();
        }
        exit(1);
    }
    if (!reader.Warning().empty()) {
        cout << "TinyObjReader: " << reader.Warning();
    }

    int xslice = 1, yslice = 1, zslice = 1;
    if (argc > 2) xslice = atoi(argv[2]);
    if (argc > 3) yslice = atoi(argv[3]);
    if (argc > 4) zslice = atoi(argv[4]);
    voxels = vector<vector<vector<Voxel>>>(xslice, vector<vector<Voxel>>(yslice, vector<Voxel>(zslice)));
    
    ofstream out("param.txt");
    //ofstream outHull("hull.txt");
    vec3 slide(xslice, yslice, zslice);
    vec3 bbmin(INFINITY, INFINITY, INFINITY), bbmax(-INFINITY, -INFINITY, -INFINITY);
    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    for (size_t s = 0; s < shapes.size(); ++s) {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); ++f) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
            for (size_t v = 0; v < fv; ++v) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
                tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
                tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];

            }
            index_offset += fv;

        }
    }
    vec3 cap = bbmax - bbmin;
    cap[0] /= (float)xslice; cap[1] /= (float)yslice; cap[2] /= (float)zslice;
    double startTime = glfwGetTime();
    for (int i = 0; i < xslice; ++i) {
        for (int j = 0; j < yslice; ++j) {
            for (int k = 0; k < zslice; ++k) {
                voxels[i][j][k].bmin = bbmin + vec3(i, j, k) * cap;
                voxels[i][j][k].bmax = bbmin + vec3(i + 1, j + 1, k + 1) * cap;
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
                if (voxel.vertices > 0) {
                    voxel.quadric = voxel.fit.fitQuadric();
                    out << voxel.bmin[0] << ' ' << voxel.bmin[1] << ' ' << voxel.bmin[2] << ' ' << voxel.bmax[0] << ' ' << voxel.bmax[1] << ' ' << voxel.bmax[2];
                    for (int q = 0; q < 10; ++q) {
                        out << ' ' << voxel.quadric.c[q];
                    }
                    out << voxel.quadric.sigma << endl;
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
                if (voxel.vertices > 0) {
                    VoxelData vd;
                    memcpy(vd.q, voxel.quadric.c, sizeof(voxel.quadric.c));
                    //memcpy(vd.hullQ, voxel.hullQ, sizeof(voxel.hullQ));
                    for (int d = 0; d < 3; ++d) {
                        vd.bmin[d] = voxel.bmin[d];
                        vd.bmax[d] = voxel.bmax[d];
                    }
                    vd.sigma = voxel.quadric.sigma;
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
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
        float yoffset = camera.MovementSpeed * deltaTime / camera.MouseSensitivity * 10.f;
        camera.ProcessMouseMovement(0, yoffset);
    }
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        float yoffset = -camera.MovementSpeed * deltaTime / camera.MouseSensitivity * 10.f;
        camera.ProcessMouseMovement(0, yoffset);
    }
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        float xoffset = camera.MovementSpeed * deltaTime / camera.MouseSensitivity * 10.f;
        camera.ProcessMouseMovement(xoffset, 0);
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        float xoffset = -camera.MovementSpeed * deltaTime / camera.MouseSensitivity * 10.f;
        camera.ProcessMouseMovement(xoffset, 0);
    }
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
