// demonstrates usage of the quadric fitting library
// pass a mesh to fit on the command line (./aqd_demo meshname)
// and this will fit all quadric types, then render the results
// press tab or left/right arrows to cycle through fitting results

#include "view.h"

#include "quadricfitting.h"

#include <fstream>
#include <map>

FILE _iob[] = { *stdin, *stdout, *stderr };
extern "C" FILE * __cdecl __iob_func(void) { return _iob; }

using namespace std;

struct Voxel{
    allquadrics::TriangleMesh quadric;
    vec3 bmin, bmax;
};

vector<Voxel> voxels;

void drawMesh(allquadrics::TriangleMesh &mesh, vec3 bmin, vec3 bmax) {
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i < mesh.triangles.size(); i++) {
        
        if (i < mesh.triangleTags.size()) {
            if (mesh.triangleTags[i] == mesh.activeTag) {
                glColor3d(1,0,0);
            } else {
                glColor3d(1,1,1);
            }
        }
		glNormal3dv(&mesh.triangleNormals[i][0]);
        vec3 v0 = mesh.vertices[mesh.triangles[i].ind[0]];
        vec3 v1 = mesh.vertices[mesh.triangles[i].ind[1]];
        vec3 v2 = mesh.vertices[mesh.triangles[i].ind[2]];
        vec3 vmin = min(v0, min(v1, v2)), vmax = max(v0, max(v1, v2));
        if (min(vmin, bmin) == bmin && max(vmax, bmax) == bmax) {
            for (int ii = 0; ii < 3; ii++) {
                glVertex3dv(&mesh.vertices[mesh.triangles[i].ind[ii]][0]);
            }
        }
	}
	glEnd();
}

void display(Viewport &viewport, allquadrics::TriangleMesh &reference) {

    // setup gl state
    glClearColor(.5f,.7f,1,1);
    glEnable(GL_NORMALIZE);
    glDisable(GL_CULL_FACE);

    // clear the screen
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    
    // setup the camera
    viewport.loadView();

	glColor3d(1,1,1);
	drawMesh(reference, vec3(-INFINITY, -INFINITY, -INFINITY), vec3(INFINITY, INFINITY, INFINITY));

	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4d(.7,.3,.1,.3);
	glEnable(GL_CULL_FACE);
    for (int i = 0; i < voxels.size(); ++i) {
        glCullFace(GL_FRONT);
        drawMesh(voxels[i].quadric, voxels[i].bmin, voxels[i].bmax);
        glCullFace(GL_BACK);
        drawMesh(voxels[i].quadric, voxels[i].bmin, voxels[i].bmax);
    }

    glfwSwapBuffers();
}



void reshape(int w, int h) {
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90.0, ((double)w / MAX(h, 1)), .0001, 5.0);

    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}




int main(int argc, char **argv)
{
	Viewport viewport;
	allquadrics::TriangleMesh inputMesh, quadricMesh;

	const char *defaultInput = "../example_data/default.obj";
	const char *meshfile = argc > 1 ? argv[1] : 0;
	if (!meshfile) {
		cerr << "No mesh file specified?  Loading default mesh: " << defaultInput << endl;
		meshfile = defaultInput;
	}
	if (!inputMesh.loadObj(meshfile)) {
        cerr << "Couldn't load file " << meshfile << endl;
        return 1;
    }
    int xslice = 1, yslice = 1, zslice = 1;
    if (argc > 2) xslice = atoi(argv[2]);
    if (argc > 3) yslice = atoi(argv[3]);
    if (argc > 4) zslice = atoi(argv[4]);
	
	// Always recenter and scale your data before fitting!
	//inputMesh.centerAndScale(1);
    inputMesh.triangleTags.resize(inputMesh.triangles.size(), 0);
    inputMesh.activeTag = 1;

    ofstream out("param.txt");
    vec3 slide(xslice, yslice, zslice);
    vec3 bbmin(INFINITY, INFINITY, INFINITY), bbmax(-INFINITY, -INFINITY, -INFINITY);
    for (int i = 0; i < inputMesh.vertices.size(); ++i) {
        bbmin = min(bbmin, inputMesh.vertices[i]);
        bbmax = max(bbmax, inputMesh.vertices[i]);
    }
    vec3 cap = bbmax - bbmin;
    cap[0] /= (double)xslice; cap[1] /= (double)yslice; cap[2] /= (double)zslice;
    for (int i = 0; i < xslice; ++i) {
        for (int j = 0; j < yslice; ++j) {
            for (int k = 0; k < zslice; ++k) {
                Voxel voxel;
                voxel.bmin = bbmin + prod(vec3(i, j, k), cap);
                voxel.bmax = bbmin + prod(vec3(i + 1, j + 1, k + 1), cap);
                bool flag = false;
                for (int t = 0; t < inputMesh.triangles.size(); ++t) {
                    // Use bounding box to find intersected triangles.
                    vec3 v0 = inputMesh.vertices[inputMesh.triangles[t].ind[0]];
                    vec3 v1 = inputMesh.vertices[inputMesh.triangles[t].ind[1]];
                    vec3 v2 = inputMesh.vertices[inputMesh.triangles[t].ind[2]];
                    vec3 vmin = min(v0, min(v1, v2)), vmax = max(v0, max(v1, v2));
                    if (!(voxel.bmin[0] > vmax[0] || voxel.bmax[0] < vmin[0]
                        || voxel.bmin[1] > vmax[1] || voxel.bmax[1] < vmin[1]
                        || voxel.bmin[2] > vmax[2] || voxel.bmax[2] < vmin[2])) {
                        inputMesh.triangleTags[t] = 1;
                        flag = true;
                    }
                }
                if (flag) {
                    allquadrics::Quadric qfit;
                    allquadrics::fitEllipsoid(inputMesh, qfit);
                    vec3 r(0, 0, 0);
                    qfit.buildMeshFromQuadric(voxel.quadric, r, r);
                    voxels.push_back(voxel);
                    for (int t = 0; t < inputMesh.triangleTags.size(); ++t) {
                        inputMesh.triangleTags[t] = 0;
                    }
                    out << voxel.bmin << voxel.bmax;
                    for (int q = 0; q < 10; ++q) {
                        out << ' ' << qfit.q[q];
                    }
                    out << endl;
                }
            }
        }
    }

	int showQuadricFit = 0;

    glfwInit();

    // default window size:
    int W = 800, H = 500;
    // Open window
    int ok = glfwOpenWindow(W, H, 8, 8, 8, 8, 24, 8, GLFW_WINDOW);
    if( !ok ) { glfwTerminate(); return 0; }
    // setup gl window/perspective based on window height
    reshape(W,H);

    // Set window title
    glfwSetWindowTitle( "Quadric Fitting Demo" );

    // Enable sticky keys
    glfwEnable( GLFW_STICKY_KEYS );
	
    // set some lights
    {
       float ambient[3] = { .1f, .1f, .1f };
       float diffuse[3] = { .4f, .3f, .3f };
	   float pos[4] = { 0, 2, 0, 0 };
       
       glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
       glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
       glLightfv(GL_LIGHT1, GL_POSITION, pos);
       glEnable(GL_LIGHT1);
    }
    {
       float ambient[3] = { .1f, .1f, .1f };
       float diffuse[3] = { .1f, .2f, .2f};
       float pos[4] = { 0, 0, -2, 0 };
       glLightfv(GL_LIGHT2, GL_AMBIENT, ambient);
       glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse);
       glLightfv(GL_LIGHT2, GL_POSITION, pos);
       glEnable(GL_LIGHT2);
    }
    {
       float ambient[3] = { .1f, .1f, .1f };
       float diffuse[3] = { .1f, .2f, .1f};
       float pos[4] = { -1, 0, 0, 0 };
       glLightfv(GL_LIGHT3, GL_AMBIENT, ambient);
       glLightfv(GL_LIGHT3, GL_DIFFUSE, diffuse);
       glLightfv(GL_LIGHT3, GL_POSITION, pos);
       glEnable(GL_LIGHT3);
    }
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

	srand(95);

    viewport.resetCam();

    int mx, my;
    //glfwDisable( GLFW_MOUSE_CURSOR );
    glfwGetMousePos(&mx, &my);

	vec3 mousePos;
    double lastTime = 0, timePerFrame = 1.0/30.0;

    vec4 lightPos(-5,5,5,0);

    bool mouseWasUp = false;
    
    string windowTitle = "Quadric Fitting Demo";
    glfwSetWindowTitle(windowTitle.c_str());

    // Main rendering loop
    while (true) {

        bool changedLast = false;

		int logics = 0;
		while (glfwGetTime() - lastTime > timePerFrame) {
			{   
				int nmx, nmy;
				glfwGetMousePos(&nmx, &nmy);

                if (!glfwGetMouseButton(GLFW_MOUSE_BUTTON_1) && !glfwGetMouseButton(GLFW_MOUSE_BUTTON_2)) {
                    mouseWasUp = true;
                }

                if (glfwGetMouseButton(GLFW_MOUSE_BUTTON_1)) { // mouse movements update the view
                    // Rotate viewport orientation proportional to mouse motion
                    viewport.mousePos = vec2((double)mx / (double)W,(double)my / (double)H);
                    vec2 newMouse = vec2((double)nmx / (double)W,(double)nmy / (double)H);
                    vec2 diff = (newMouse - viewport.mousePos);
                    double len = diff.length();
					if (!glfwGetKey('Z') && !glfwGetKey(GLFW_KEY_LALT) && len > .001) {
                        vec3 axis = vec3(diff[1]/len, diff[0]/len, 0);
                        viewport.orientation = rotation3D(axis, 180 * len) * viewport.orientation;
                    }
                    if ( (glfwGetKey('Z') || glfwGetKey(GLFW_KEY_LALT)) && fabs(diff[1]) > .001) {
    	                viewport.zoom += diff[1];
    	                if (viewport.zoom < .001) viewport.zoom = .001;
                    }

                    //Record the mouse location for drawing crosshairs
                    viewport.mousePos = newMouse;
                }
                
                mx = nmx; my = nmy;
			}

			lastTime += timePerFrame;
			logics++;

			if (changedLast || logics > 10) // slow down if you really can't keep up
				break;
		}

		if (logics > 0) {

            display(viewport, inputMesh);
		}
		 else {
            glfwSleep( .001 ); // else release control to OS for 5 ms
        }

        // Check if the escape key was pressed, or if the window was closed
        if (glfwGetKey( GLFW_KEY_ESC ) || !glfwGetWindowParam( GLFW_OPENED )) {
            break;
        }
    }

    // cleanup and exit
    glfwTerminate();
    return 0;
}
