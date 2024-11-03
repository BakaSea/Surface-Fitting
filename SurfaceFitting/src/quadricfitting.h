// type-specific quadric fitting library
// by James Andrews (zaphos@gmail.com)

// this is 'research code': it's not thoroughly tested and it's not optimized for performance
// report bugs and feature requests to: zaphos@gmail.com

// overview: call one of the fit*() functions on your data to get direct fits of all quadric types
// currently your data must be a vector of data_pnw objects, or a TriangleMesh

// for best results, please center and scale your data before calling the fitting functions

#ifndef QUADRICFITTING_H
#define QUADRICFITTING_H

#include <vector>
#include <glm/glm.hpp>
//#include "algebra3.h"

using namespace glm;
using namespace std;

namespace allquadrics {

// THE DATA STRUCTURES (pre-declarations)

// structure containing data points (with point, normal, and a 'weight' indicating how much it contributes to the fit (e.g. the dunavant weights go here)
struct data_pnw;
// structure for representing a triangle mesh; used to fit quadric surfaces to a mesh and to export quadric surfaces as meshes
// the fitting functions use dunavant quadrature on the triangles of the mesh to integrate error over the surface
// note: you can optionally use triangleTags to filter which triangles are used for the fit (e.g. if you want to fit selections on a given mesh, use this to indicate the selection.)
struct TriangleMesh;

// structure defining a quadric surface, with helpful member functions for evaluating, transforming and meshing that surface
struct Quadric;


// THE FITTING FUNCTIONS!

//Fits an ellipsoid
void fitEllipsoid(std::vector<data_pnw> &data, Quadric &quadric);
void fitEllipsoid(TriangleMesh &mesh, Quadric &quadric);


// IDs for all quadric types we fit; the fitAllQuadrics functions return the quadrics in this order
enum { TYPE_GENERAL_QUADRIC = 0, TYPE_ROTSYM_QUADRIC, TYPE_PLANE, TYPE_SPHERE,                  // 0-3
    TYPE_GEN_CYL, TYPE_CIRC_CYL, TYPE_CONE, TYPE_CIRC_CONE,                                     // 4-7
    TYPE_ELLIPSOID_BIASED, TYPE_HYPERBOLOID_BIASED, TYPE_ELLIPSOID_OPT, TYPE_HYPERBOLOID_OPT,   // 8-11
    TYPE_HYPERBOLOID_1SHEET, TYPE_HYPERBOLOID_2SHEET, TYPE_PARABOLOID,                          // 12-14
    TYPE_PARABOLOID_ELLIPTICAL, TYPE_PARABOLOID_HYPERBOLIC,                                     // 15-16
    TYPE_ELL_CYL, TYPE_HYPER_CYL, TYPE_PARA_CYL,                                                // 17-19
    NUM_QUADRIC_TYPES };

// a data point with position, normal and weight
struct data_pnw {
	dvec3 p, n;
	double w;

	data_pnw(dvec3 &p, dvec3 &n, double w) : p(p), n(n), w(w) {}
	data_pnw() {}
};

// stores triangles
struct Tri {
	int ind[3];
	Tri(int a, int b, int c) { ind[0] = a; ind[1] = b; ind[2] = c; }
	Tri() { ind[0] = ind[1] = ind[2] = 0; }
};

// stores triangle meshes
struct TriangleMesh {
	
	std::vector<Tri> triangles;
	std::vector<dvec3> normals;
	std::vector<dvec3> vertices;
    
    // use the tags to filter what gets fit
    // if triangleTag.size()!=triangles.size(), we fit everything
    // otherwise, we only fit the triangles that have a tag matching activeTag
    std::vector<int> triangleTags;
    int activeTag; 

	void computeNormals() {
        normals.resize(vertices.size());
		//triangleNormals.resize(triangles.size());
		//for (size_t i = 0; i < triangles.size(); i++) {
		//	dvec3 edge1 = vertices[triangles[i].ind[1]] - vertices[triangles[i].ind[0]];
		//	dvec3 edge2 = vertices[triangles[i].ind[2]] - vertices[triangles[i].ind[1]];
  //          triangleNormals[i] = cross(edge1, edge2);
  //          triangleNormals[i] = normalize(triangleNormals[i]);
		//}
	}

	void clear() {
		triangles.clear();
        vertices.clear();
        normals.clear();
	}

	bool loadObj(const char *file);

	void centerAndScale(double scale) {
		if (vertices.empty())
			return;

		dvec3 maxp = vertices[0], minp = vertices[0];
		for (vector<dvec3>::iterator it = vertices.begin(); it != vertices.end(); ++it) {
			maxp = max(*it, maxp); // max and min def'd in algebra3.h take the max or min componentwise from each vector
			minp = min(*it, minp);
		}
		dvec3 center = (maxp+minp)*.5;
		dvec3 size = maxp-minp;
        double maxSizeInv = glm::max(size[0], glm::max(size[1], size[2]));
		if (maxSizeInv == 0) // mesh is just one point
			return;
		maxSizeInv = 1.0f/maxSizeInv;
		for (vector<dvec3>::iterator it = vertices.begin(); it != vertices.end(); ++it) {
			*it = (*it-center)*maxSizeInv*scale;
		}
	}

};

// Class to represent a quadric in general form, using 10 parameters
// includes helpful functions to transform the quadric shape, and to produce a mesh of the quadric surface
struct Quadric {
    double q[10];

	Quadric()  { clear(); }
	void clear() { for (int i = 0; i < 10; i++) { q[i] = 0; } }

	// rescale the quadric parameters
	void normalizeField() {
		double maxq = 0;
        for (int i = 0; i < 10; i++) {
            maxq = glm::max(fabs(q[i]), maxq);
        }
        for (int i = 0; i < 10; i++) {
            q[i] /= maxq;
        }
    }

    void init(const double *params) {
        for (int i = 0; i < 10; i++) {
            q[i] = params[i];
        }
    }
	void init(const std::vector<double> &params) {
        for (int i = 0; i < 10; i++) {
            q[i] = params[i];
        }
    }
    void fill(std::vector<double> &params) {
        params.resize(10);
        for (int i = 0; i < 10; i++) {
            params[i] = q[i];
        }
    }
    double f(const dvec3 &p) const { // value of quadric function
        double x = p[0], y = p[1], z = p[2];
        return q[0] + q[1]*x + q[2]*y + q[3]*z 
            + q[4]*x*x + q[5]*x*y + q[6]*x*z
            + q[7]*y*y + q[8]*y*z + q[9]*z*z;
    }
    dvec3 df(const dvec3 &p) const { // gradient of quadric function
        double x = p[0], y = p[1], z = p[2];
        return dvec3(q[1] + 2 * x * q[4] + y * q[5] + z * q[6],
            q[2] + x * q[5] + 2 * y * q[7] + z * q[8],
            q[3] + x * q[6] + y * q[8] + 2 * z * q[9]);
    }

    // approx distance due to [taubin, 93]
    double approxDist(const dvec3 &p) {
        dvec3 dfp = df(p);
        // quadric coefficients, following quadric surface extraction by ... paper's text (b, a reversed as in errata)
        double b = -sqrt(dot(dfp, dfp));
        double a = -sqrt((q[5] * q[5] + q[6] * q[6] + q[8] * q[8]) / 2
            + q[4] * q[4] + q[7] * q[7] + q[9] * q[9]);
        double c = fabs(f(p));
        double num = (-b - sqrt(b * b - 4 * a * c));
        double denom = 2*a;
		if (fabs(a) < .0000001) { // special case: it's a plane ...
            if (b > -.0000001) { // b == -|grad|
                return c / -b;
            } else { // it's a totally degen quadric
                return 0;
            }
        } else {
            double d = num/denom;
            return fabs(d);
        }
    }

};

} // namespace allquadrics


#endif




