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
#include "shader_c.h"
#include "voxel_handle.h"
#include "mesh.h"
//#include "quadricfitting.h"
//#include "ellipsoidhull.h"
using namespace std;

struct VoxelData {
    float q[10];
    //float hullQ[10];
    float bmin[3];
    float bmax[3];
    float sggx[6];
    float albedo[3];
    float sigma;
    float alpha;
};

vector<VoxelData> voxelDatas;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void processInput(GLFWwindow *window);
void renderQuad();

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
int renderMode = 0;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;
bool moved = false;

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;

float imgColor[SCR_WIDTH * SCR_HEIGHT * 4];

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
    glfwSetKeyCallback(window, key_callback);

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
    ComputeShader rtCompShader("shader/raytrace.comp");
    raytraceShader.use();
    raytraceShader.setInt("tex", 0);
    Shader simpleShader("shader/simple.vs", "shader/simple.fs");
    
    unsigned int texture;
    glGenTextures(1, &texture);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, SCR_WIDTH, SCR_HEIGHT, 0, GL_RGBA, GL_FLOAT, NULL);
    glBindImageTexture(0, texture, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);

    const char *defaultInput = "../example_data/default.obj";
    const char *meshfile = argc > 1 ? argv[1] : 0;
    if (!meshfile) {
    	cerr << "No mesh file specified?  Loading default mesh: " << defaultInput << endl;
    	meshfile = defaultInput;
    }

    int slice = 1;
    if (argc > 2) slice = atoi(argv[2]);

    //ofstream outHull("hull.txt");
    //auto& attrib = reader.GetAttrib();
    //auto& shapes = reader.GetShapes();
    //float scale = 10.0f;

    //vector<Vertex> vtx;
    //vector<unsigned int> ind;
    //for (int i = 0; i < attrib.vertices.size(); i += 3) {
    //    vec3 position = vec3(attrib.vertices[i], attrib.vertices[i + 1], attrib.vertices[i + 2]) * scale;
    //    vec3 normal = vec3(0);
    //    vtx.push_back(Vertex(position, normal, vec2(0)));
    //}
    //for (size_t s = 0; s < shapes.size(); ++s) {
    //    for (int i = 0; i < shapes[s].mesh.indices.size(); ++i) {
    //        ind.push_back(shapes[s].mesh.indices[i].vertex_index);
    //    }
    //}
    //Mesh mesh(vtx, ind, {});

    VoxelLayer layer(meshfile, slice);

    for (int i = 0; i < layer.slice.x; ++i) {
        for (int j = 0; j < layer.slice.y; ++j) {
            for (int k = 0; k < layer.slice.z; ++k) {
                auto& voxel = layer.voxels[i][j][k];
                if (voxel.fit.vertices > 0) {
                    VoxelData vd;
                    memcpy(vd.q, voxel.quadric.c, sizeof(voxel.quadric.c));
                    //memcpy(vd.hullQ, voxel.hullQ, sizeof(voxel.hullQ));
                    vd.sggx[0] = voxel.sggx.S_xx;
                    vd.sggx[1] = voxel.sggx.S_xy;
                    vd.sggx[2] = voxel.sggx.S_xz;
                    vd.sggx[3] = voxel.sggx.S_yy;
                    vd.sggx[4] = voxel.sggx.S_yz;
                    vd.sggx[5] = voxel.sggx.S_zz;
                    for (int d = 0; d < 3; ++d) {
                        vd.bmin[d] = voxel.bmin[d];
                        vd.bmax[d] = voxel.bmax[d];
                    }
                    vd.albedo[0] = .7f; vd.albedo[1] = .6f; vd.albedo[2] = .5f;
                    vd.sigma = voxel.quadric.sigma;
                    //vd.sigma = 0.001f;
                    vd.alpha = voxel.alpha;
                    voxelDatas.emplace_back(vd);
                }
            }
        }
    }
    GLuint voxelDatasBuffer;
    glCreateBuffers(1, &voxelDatasBuffer);
    glNamedBufferStorage(voxelDatasBuffer, sizeof(VoxelData)* voxelDatas.size(), (const void*)voxelDatas.data(), GL_DYNAMIC_STORAGE_BIT);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, voxelDatasBuffer);

    int currentSample = 1;
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
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (renderMode == 0) {
            rtCompShader.use();
            rtCompShader.setMat4("viewInv", inverse(view));
            rtCompShader.setMat4("projectionInv", inverse(projection));
            rtCompShader.setVec3("cameraPos", camera.Position);
            rtCompShader.setVec2("resolution", vec2(SCR_WIDTH, SCR_HEIGHT));
            rtCompShader.setInt("voxelSize", voxelDatas.size());
            if (moved) {
                currentSample = 1;
                glClearTexImage(texture, 0, GL_RGBA, GL_FLOAT, NULL);
                moved = false;
            } else {
                currentSample++;
            }
            rtCompShader.setInt("currentSample", currentSample);
            glDispatchCompute(SCR_WIDTH, SCR_HEIGHT, 1);
            glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
            raytraceShader.use();
            renderQuad();
        } else if (renderMode == 1) {
            simpleShader.use();
            simpleShader.setMat4("model", mat4(1.0f));
            simpleShader.setMat4("projection", projection);
            simpleShader.setMat4("view", view);
            for (auto& mesh : layer.meshes) {
                mesh.Draw(simpleShader);
            }
        }

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

// renderQuad() renders a 1x1 XY quad in NDC
// -----------------------------------------
unsigned int quadVAO = 0;
unsigned int quadVBO;
void renderQuad() {
    if (quadVAO == 0) {
        float quadVertices[] = {
            // positions        // texture Coords
            -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
            -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
            1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
            1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
        };
        // setup plane VAO
        glGenVertexArrays(1, &quadVAO);
        glGenBuffers(1, &quadVBO);
        glBindVertexArray(quadVAO);
        glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    }
    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_R && action == GLFW_PRESS) {
        renderMode = (renderMode + 1) % 2;
    }
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        camera.ProcessKeyboard(FORWARD, deltaTime);
        moved = true;
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        camera.ProcessKeyboard(BACKWARD, deltaTime);
        moved = true;
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        camera.ProcessKeyboard(LEFT, deltaTime);
        moved = true;
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        camera.ProcessKeyboard(RIGHT, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
        float yoffset = camera.MovementSpeed * deltaTime / camera.MouseSensitivity * 10.f;
        camera.ProcessMouseMovement(0, yoffset);
        moved = true;
    }
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        float yoffset = -camera.MovementSpeed * deltaTime / camera.MouseSensitivity * 10.f;
        camera.ProcessMouseMovement(0, yoffset);
        moved = true;
    }
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        float xoffset = camera.MovementSpeed * deltaTime / camera.MouseSensitivity * 10.f;
        camera.ProcessMouseMovement(xoffset, 0);
        moved = true;
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        float xoffset = -camera.MovementSpeed * deltaTime / camera.MouseSensitivity * 10.f;
        camera.ProcessMouseMovement(xoffset, 0);
        moved = true;
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

    if (xoffset > epsilon<float>() || yoffset > epsilon<float>()) {
        moved = true;
    }

    camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    if (yoffset > epsilon<float>()) {
        moved = true;
    }
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}
