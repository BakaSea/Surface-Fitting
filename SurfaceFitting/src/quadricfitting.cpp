#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>
#include "quadricfitting.h"
#include "roots3and4.h"
#include "float.h"

using namespace std;
using namespace Eigen;

namespace allquadrics {

struct sortBySecond {
    bool operator()(const pair<int,double> &a, const pair<int,double> &b)
    { return a.second < b.second; }
};

dvec3 getdmat3Eigenvalues(const dmat3 &m) {
    Matrix3d M;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            M(i, j) = m[i][j];
        }
    }
    Vector3cd eig = M.eigenvalues();
    return dvec3(eig[0].real(), eig[1].real(), eig[2].real());
}



// scalar triple product
inline double stp(dvec3 &a0, dvec3 &a1, dvec3 &a2) {
    return dot(a0, cross(a1, a2));
}

inline double det(vec2 &a0, vec2 &a1) {
    return a0[0]*a1[1]-a0[1]*a1[0];
}
void buildDetPoly3(dmat3 &A, dmat3 &B, double *a) {
    a[0] = stp(A[0], A[1], A[2]);
    a[1] = stp(B[0], A[1], A[2]) + stp(A[0], B[1], A[2]) + stp(A[0], A[1], B[2]);
    a[2] = stp(A[0], B[1], B[2]) + stp(B[0], A[1], B[2]) + stp(B[0], B[1], A[2]);
    a[3] = stp(B[0], B[1], B[2]);
}

struct sortByFirst {
    bool operator()(const pair<double,int> &a, const pair<double,int> &b)
    { return a.second < b.second; }
};

int isHyperbolic(const dmat3 &q, double epsilon = 2e-4) {
    dvec3 eigs = getdmat3Eigenvalues(q);
    eigs = normalize(eigs);
    
    double e01 = eigs[0]*eigs[1], e02 = eigs[0]*eigs[2], e12 = eigs[1]*eigs[2];
    if (eigs[0]*eigs[1] < -epsilon || eigs[0]*eigs[2] < -epsilon || eigs[1]*eigs[2] < -epsilon) return 1;
    int numZ = (fabs(eigs[0]) < epsilon) + (fabs(eigs[1]) < epsilon) + (fabs(eigs[2]) < epsilon);
    if (numZ > 1) return -1;
    return 0;
}

template<typename T>
dmat3 getQmat(T *q) {
    dmat3 A(0);
    A[0][0] = q[4];
    A[1][0] = A[0][1] = q[5]*.5f;
    A[2][0] = A[0][2] = q[6]*.5f;
    A[1][1] = q[7];
    A[1][2] = A[2][1] = q[8]*.5f;
    A[2][2] = q[9];
    return A;
}

struct QuadricFieldFitter {
    double M[100], N[100];
    QuadricFieldFitter() { clear(0); }

    void clear(int sizehint) {
        for (int i = 0; i < 100; i++) { M[i]=N[i]=0; } 
    }

    // find the range of parameters for which interpolating the two param. vectors gives ellipsoids and hyperboloids
    // also, the best (by M,N metric) interpolation t's for each type
    void lineSearch(double *p1, double *p2,  
                                     double &bestHyper, double &bestEll, double &bestPara, double &bestParaH, double &bestParaE,
                                      double &t_hyper, double &t_ell, double &t_para, double &t_hpara, double &t_epara);

    template<typename T>
    T getTaubinError(T *params) {
        T msum = 0, nsum = 0;
        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < 10; j++) {
                msum += M[i*10+j]*params[j]*params[i];
                nsum += N[i*10+j]*params[j]*params[i];
            }
        }
        return msum / nsum;
    }
    template<typename T>
    T getTaubinError_interp(T *params1, T *params2, double t) {
        T params[10];
        for (int i = 0; i < 10; i++) {
            params[i] = params1[i] + t*params2[i];
        }
        T msum = 0, nsum = 0;
        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < 10; j++) {
                msum += M[i*10+j]*params[j]*params[i];
                nsum += N[i*10+j]*params[j]*params[i];
            }
        }
        return msum / nsum;
    }

    void addPosition(double scale, const dvec3 &p) {
        double c[10];
        double cx[10],cy[10],cz[10];
        double x = p[0], y = p[1], z = p[2];

        // coefficient vector
        c[0] = 1;
        c[1] = x;
        c[2] = y;
        c[3] = z;
        c[4] = x*x;
        c[5] = x*y;
        c[6] = x*z;
        c[7] = y*y;
        c[8] = y*z;
        c[9] = z*z;

        // partial derivative vector's coefficient vectors
        for (int i = 0; i < 10; i++) { cx[i] = cy[i] = cz[i] = 0; }
        cx[1] = 1; cx[4] = 2*x; cx[5] = y; cx[6] = z;
        cy[2] = 1; cy[5] = x; cy[7] = 2*y; cy[8] = z;
        cz[3] = 1; cz[6] = x; cz[8] = y; cz[9] = 2*z;

        // add cross products to accumulator matrices
        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < 10; j++) {
                M[i*10+j] += c[i]*c[j] * scale;
                N[i*10+j] += (cx[i]*cx[j] + cy[i]*cy[j] + cz[i]*cz[j]) * scale;
            }
        }
    }
    void addNormal(double scale, const dvec3 &p, const dvec3 &n) {
        double cx[10],cy[10],cz[10];
        double x = p[0], y = p[1], z = p[2];

        // partial derivative vector's coefficient vectors
        for (int i = 0; i < 10; i++) { cx[i] = cy[i] = cz[i] = 0; }
        cx[1] = 1; cx[4] = 2*x; cx[5] = y; cx[6] = z;
        cy[2] = 1; cy[5] = x; cy[7] = 2*y; cy[8] = z;
        cz[3] = 1; cz[6] = x; cz[8] = y; cz[9] = 2*z;

        // add cross products to accumulator matrices
        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < 10; j++) {
                N[i*10+j] += (cx[i]*cx[j] + cy[i]*cy[j] + cz[i]*cz[j]) * scale;
            }
        }

		/*if (g_optk.gradientErrorWt > 0) {
            dvec3 t = findOrtho(n);
            dvec3 b = t%n;
            b.normalize();

            // square vectors:
            double ct[10], cb[10];
            for (int i = 0; i < 10; i++) {
                ct[i] = cx[i]*t[0]+cy[i]*t[1]+cz[i]*t[2];
                cb[i] = cx[i]*b[0]+cy[i]*b[1]+cz[i]*b[2];
            }

            double gradwt = g_optk.gradientErrorWt;
            for (int i = 0; i < 10; i++) {
                for (int j = 0; j < 10; j++) {
                    M[i*10+j] += ct[i]*ct[j] * scale * gradwt;
                    M[i*10+j] += cb[i]*cb[j] * scale * gradwt;
                }
            }
        }*/
    }

    void addEl(double scale, const dvec3 &p, const dvec3 &n) {
        double c[10];
        double cx[10],cy[10],cz[10];
        double x = p[0], y = p[1], z = p[2];

        // coefficient vector
        c[0] = 1;
        c[1] = x;
        c[2] = y;
        c[3] = z;
        c[4] = x*x;
        c[5] = x*y;
        c[6] = x*z;
        c[7] = y*y;
        c[8] = y*z;
        c[9] = z*z;

        // partial derivative vector's coefficient vectors
        for (int i = 0; i < 10; i++) { cx[i] = cy[i] = cz[i] = 0; }
        cx[1] = 1; cx[4] = 2*x; cx[5] = y; cx[6] = z;
        cy[2] = 1; cy[5] = x; cy[7] = 2*y; cy[8] = z;
        cz[3] = 1; cz[6] = x; cz[8] = y; cz[9] = 2*z;

        // add cross products to accumulator matrices
        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < 10; j++) {
                M[i*10+j] += c[i]*c[j] * scale;
                N[i*10+j] += (cx[i]*cx[j] + cy[i]*cy[j] + cz[i]*cz[j]) * scale;
            }
        }

		// code to account for error in the normals as well; commented out for release (b/c it required a new parameter and didn't seem to have a huge effect in practice)
/*        if (g_optk.gradientErrorWt > 0) {
            dvec3 t = findOrtho(n);
            dvec3 b = t%n;
            b.normalize();

            // square vectors:
            double ct[10], cb[10];
            for (int i = 0; i < 10; i++) {
                ct[i] = cx[i]*t[0]+cy[i]*t[1]+cz[i]*t[2];
                cb[i] = cx[i]*b[0]+cy[i]*b[1]+cz[i]*b[2];
            }

            double gradwt = g_optk.gradientErrorWt;
            for (int i = 0; i < 10; i++) {
                for (int j = 0; j < 10; j++) {
                    M[i*10+j] += ct[i]*ct[j] * scale * gradwt;
                    M[i*10+j] += cb[i]*cb[j] * scale * gradwt;
                }
            }
        }*/
    }

    void getParams(vector<double> &tofill, int forceType = -1); // uses lapack stuff that is included in the cpp file
    void getParamBasis(vector<double> &sortedFabsEigvals, vector<double> &sortedEigvecs);
};

// given: p1 and p2 are general quadric parameter vectors
// p1 should be the taubin optimal parameter vector
// outputs: errors and interpolation parameters of the best hyperboloid, ellipsoid, and paraboloid
void QuadricFieldFitter::lineSearch(double *p1, double *p2,  
                                     double &bestHyper, double &bestEll, double &bestPara, double &bestParaH, double &bestParaE,
                                      double &t_hyper, double &t_ell, double &t_para, double &t_hpara, double &t_epara)
{
    bestHyper = bestEll = bestPara = bestParaE = bestParaH = -2; // negative errors indicate unassigned / no results found yet

    dmat3 q31 = getQmat(p1);
    dmat3 q32 = getQmat(p2);
    double det3poly[4];
    buildDetPoly3(q31, q32, det3poly);
    double roots3[3];
    int nr3;
    nr3 = SolveCubic(det3poly, roots3);
    vector< pair<int,double> > allRoots;

    // assign the t=0 case as best ell or hyper based on the type
    int hyperT = isHyperbolic(q31);
    if (hyperT != 0) {
        t_hyper = 0;
        bestHyper = getTaubinError_interp(p1, p2, 0);
    }
    if (hyperT < 1) {
        t_ell = 0;
        bestEll = getTaubinError_interp(p1, p2, 0);
    }

    // evaluate the error at the roots of the determinant 
    for (int ri = 0; ri < nr3; ri++) {
        double t = roots3[ri];
        double err = getTaubinError_interp(p1, p2, t);

        if (bestPara < 0 || err < bestPara) {
            t_para = t;
            bestPara = err;
        }
        if (bestHyper < 0 || err < bestHyper) {
            t_hyper = t;
            bestHyper = err;
        }

        int hyperType = isHyperbolic(q31+t*q32);
        if (hyperType != 0) {
            if (bestParaH < 0 || err < bestParaH) {
                t_hpara = t;
                bestParaH = err;
            }
        }
        if (hyperType < 1) {
            if (bestEll < 0 || err < bestEll) {
                t_ell = t;
                bestEll = err;
            }
            if (bestParaE < 0 || err < bestParaE) {
                t_epara = t;
                bestParaE = err;
            }
        }

        
    }
}



void QuadricFieldFitter::getParamBasis(vector<double> &sortedFabsEigvals, vector<double> &sortedEigvecs) {
    sortedFabsEigvals.clear(); sortedEigvecs.clear();
    GeneralizedEigenSolver<MatrixXd> ges;
    MatrixXd Mtemp(10, 10), Ntemp(10, 10);
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            Mtemp(i, j) = M[i * 10 + j];
            Ntemp(i, j) = N[i * 10 + j];
        }
    }
    ges.compute(Mtemp, Ntemp);
    double alphar[10], beta[10];
    double VR[100];
    for (int i = 0; i < 10; ++i) {
        alphar[i] = ges.alphas()[i].real();
        beta[i] = ges.betas()[i];
        for (int j = 0; j < 10; ++j) {
            complex<double> v = ges.eigenvectors().col(i)[j];
            VR[i * 10 + j] = v.real();
        }
    }

    vector<pair<int, double> > fabsEigvalColumns;

    int mincol = -1;
    double mineig = -1;
    for (int i = 0; i < 10; i++) {
        if (fabs(beta[i]) > 0) {
            double eigval = fabs(alphar[i] / beta[i]);
            fabsEigvalColumns.push_back(pair<int,double>(i, eigval));
            if (mincol == -1 || eigval < mineig) {
                mincol = i;
                mineig = eigval;
            }
        }
    }
    sort(fabsEigvalColumns.begin(), fabsEigvalColumns.end(), sortBySecond());

    for (size_t i = 0; i < fabsEigvalColumns.size(); i++) {
        int col = fabsEigvalColumns[i].first;
        sortedFabsEigvals.push_back(fabsEigvalColumns[i].second);
        for (int j = 0; j < 10; j++) {
            sortedEigvecs.push_back(VR[j+10*col]);
        }
    }
}

void QuadricFieldFitter::getParams(std::vector<double> &tofill, int forceType) {
    //double work[QUADFITWORKSIZE];
    double Mtemp[100], Ntemp[100];
    for (int i = 0; i < 100; i++) { Mtemp[i] = M[i]; Ntemp[i] = N[i]; }

    enum {TYPE_ELLIPSOID = 0, TYPE_HYPERBOLOID, TYPE_MAX};

    if (forceType > -1) {
        for (int i = 0; i < 100; i++) { Ntemp[i] = 0; }
        
        if (forceType == TYPE_ELLIPSOID) {
            Ntemp[44] = Ntemp[77] = Ntemp[99] = -1;
            Ntemp[47] = Ntemp[74] = Ntemp[49] = Ntemp[94] = Ntemp[79] = Ntemp[97] = 1;
            Ntemp[55] = Ntemp[66] = Ntemp[88] = -1;
        } else if (forceType == TYPE_HYPERBOLOID) {
            Ntemp[44] = Ntemp[77] = Ntemp[99] = 0;
            Ntemp[47] = Ntemp[74] = Ntemp[49] = Ntemp[94] = Ntemp[79] = Ntemp[97] = 1;
            Ntemp[55] = Ntemp[66] = Ntemp[88] = -.5;
        }
    }
    GeneralizedEigenSolver<MatrixXd> ges;
    MatrixXd MM(10, 10), NN(10, 10);
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            MM(i, j) = Mtemp[i * 10 + j];
            NN(i, j) = Ntemp[i * 10 + j];
        }
    }
    ges.compute(MM, NN);
    double alphar[10], beta[10];
    double VR[100];
    for (int i = 0; i < 10; ++i) {
        alphar[i] = ges.alphas()[i].real();
        beta[i] = ges.betas()[i];
        for (int j = 0; j < 10; ++j) {
            VR[i * 10 + j] = ges.eigenvectors().col(i)[j].real();
        }
    }

    tofill.resize(10);
    int mincol = -1;
    double mineig = -1;
    for (int i = 0; i < 10; i++) {
        //if (fabs(alphai[i]) < 0.00001 && beta[i] > 0) {
        if (fabs(beta[i]) > 0.000001) {
            double eigvalErr = fabs(alphar[i] / beta[i]);
            if (forceType > -1) { // also check that the parameters form the target type
                dmat3 Qmat = getQmat(VR+10*i);
                double det2 = Qmat[0][0]*Qmat[1][1] - Qmat[1][0]*Qmat[0][1];
                double det3 = determinant(Qmat);
                double det1 = Qmat[0][0];

                bool ellipsoid = det2 > 0 && ( (det1 > 0 && det3 > 0) || (det1 < 0 && det3 < 0) );
                //if (g_optk.useEigenvalueSignForQuadricTypeCheck) ellipsoid = alphar[i]/beta[i] > 0; 
                if (forceType == TYPE_ELLIPSOID ) {
                    if (!ellipsoid)
                        continue; // skip it; it's not an ellipsoid
                    eigvalErr = getTaubinError(VR+10*i);
                }
                if (forceType == TYPE_HYPERBOLOID && ellipsoid) {
                    continue; // skip it; not a hyperboloid
                }
                if (forceType == TYPE_HYPERBOLOID) {
                    eigvalErr = getTaubinError(VR+10*i);
                    dvec3 eigs = getdmat3Eigenvalues(Qmat);
                    for (int i = 0; i < 3; i++) {
                        if (fabs(eigs[i]) < .0001) eigs[i] = 0; 
                    }
                }
            }
            
            if (mincol == -1 || eigvalErr < mineig) {
                mincol = i;
                mineig = eigvalErr;
            }
        }
    }
    if (mincol == -1) {
        for (int i = 0; i < 10; i++) {
            tofill[i] = 9999; // no valid solutions; make a degenerate failquadric
        }
    } else {
        for (int i = 0; i < 10; i++) {
            tofill[i] = VR[i+10*mincol]; // grab the best column
        }
    }

}



template <class DataIterator>
void fitEllipsoidHelper(DataIterator first, DataIterator last, Quadric &field) {
    QuadricFieldFitter qff;
    
	for (DataIterator i = first; i != last; i++) {
		qff.addEl(i->w, i->p, i->n);
	}
    
    vector<double> sortedFabsEigvals, sortedEigvecs;
    qff.getParamBasis(sortedFabsEigvals, sortedEigvecs);
    
    double th, te, tp, tpe, tph;
    double eh, ee, ep, epe, eph;
    double q0[10], q1[10];
    for (int i = 0; i < 10; ++i) {
        q0[i] = sortedEigvecs[i];
        q1[i] = sortedEigvecs[i + 10];
    }
    
    dmat3 qmat = getQmat(q0);
    int hyper = isHyperbolic(qmat);

    if (hyper < 1) {
        field.init(q0);
    } else {
        qff.lineSearch(q0, q1, eh, ee, ep, eph, epe, th, te, tp, tph, tpe);

        vector<double> biasedEllipsoidParams;
        if (ee < 0) { // no ellipsoid fit
            qff.getParams(biasedEllipsoidParams, 0); // (biased) fit that is forced to be an ellipsoid

            for (int i = 0; i < 10; ++i) {
                q1[i] = biasedEllipsoidParams[i];
            }
            qff.lineSearch(q0, q1, eh, ee, ep, eph, epe, th, te, tp, tph, tpe);
        }
        
        vector<double> optEllParams(10, 0);

        for (int i = 0; i < 10; i++) {
            //optHyperParams[i] = q0[i] + th*q1[i];
            optEllParams[i] = q0[i] + te*q1[i];
        }

        field.init(optEllParams);
    }
}

// helper function: computes the six order-4 dunavant points and weights
// a, b, c are the triangle vertex positions
// i is the index of the dunavant point (must be in range [0,5])
// out is the i'th dunavant point, wt is the corresponding dunavant weight
inline void computeIthDunavant(dvec3 &a, dvec3 &b, dvec3 &c, int i, dvec3 &out, double &wt) {
	// dunavant weights and coordinates (for polynomials of order 4)
	double w[6] = {0.223381589678011, 0.223381589678011, 0.223381589678011, 0.109951743655322, 0.109951743655322, 0.109951743655322};
	double x[6] = {0.10810301816807, 0.445948490915965, 0.445948490915965, 0.81684757298045896, 0.091576213509771007, 0.091576213509771007};
	double y[6] = {0.445948490915965, 0.445948490915965, 0.10810301816807, 0.091576213509771007, 0.091576213509771007, 0.81684757298045896};
	out = a*(1.0-x[i]-y[i]) + b*(x[i]) + c*(y[i]);
	wt = w[i];
}

// iterator to traverse dunavant points of a triangle mesh
class dunavantIterator
{
    public:
        dunavantIterator(TriangleMesh &m) : mesh(&m), triInd(0), subInd(0) { findStartTri(); computeData(); }
        dunavantIterator operator++() { increment(); return *this; }
        dunavantIterator operator++(int junk) { increment(); return *this; }
        data_pnw& operator*() { return data; }
        data_pnw* operator->() { return &data; }
		bool operator==(const dunavantIterator& rhs) { 
			return mesh == rhs.mesh && triInd == rhs.triInd && subInd == rhs.subInd; 
		}
        bool operator!=(const dunavantIterator& rhs) { 
			return mesh != rhs.mesh || triInd != rhs.triInd || subInd != rhs.subInd;  
		}
		static dunavantIterator end(TriangleMesh &m) {
			dunavantIterator it(m);
			it.triInd = (int)it.mesh->triangles.size();
			return it;
		}
    private:
        TriangleMesh *mesh;
		int triInd, subInd;
		data_pnw data;
    
        void findStartTri() {
            while (!mesh->triangleTags.empty() && mesh->triangleTags.size()==mesh->triangles.size()
                   && triInd < mesh->triangles.size()
                   && mesh->triangleTags[triInd] != mesh->activeTag) {
                triInd++;
            }
        }

		inline void increment() {
			if (done()) return;

			subInd++;
			if (subInd > 5) {
				subInd = 0;
				triInd++;
                
                //if the triangleTags vector is set up, skip over non-active triangles 
                while (!mesh->triangleTags.empty() && mesh->triangleTags.size()==mesh->triangles.size()
                       && triInd < mesh->triangles.size()
                       && mesh->triangleTags[triInd] != mesh->activeTag) {
                    triInd++;
                }
			}
			computeData();
		}
		inline bool done() {
			return triInd >= (int)mesh->triangles.size();
		}
		inline void computeData() {
			if (done()) {
				return;
			}

			computeIthDunavant(
				mesh->vertices[mesh->triangles[triInd].ind[0]], 
				mesh->vertices[mesh->triangles[triInd].ind[1]],
				mesh->vertices[mesh->triangles[triInd].ind[2]],
				subInd, data.p, data.w
				);
			data.n = dvec3(0, 0, 0);
		}
};	

//Fits an ellipsoid
void fitEllipsoid(std::vector<data_pnw> &data, Quadric &quadric) {
    fitEllipsoidHelper(data.begin(), data.end(), quadric);
}
void fitEllipsoid(TriangleMesh &mesh, Quadric &quadric) {
    fitEllipsoidHelper(dunavantIterator(mesh), dunavantIterator::end(mesh), quadric);
}

namespace { 
    int getNValues(stringstream &ss, vector<int> &values, char delim = '/') {
	    values.clear();
	    string sblock;
	    if (ss >> sblock) {
		    stringstream block(sblock);
		    string s;
		    int value;
		    while (getline(block, s, delim)) {
			    stringstream valuestream(s);
			    if (valuestream >> value)
				    values.push_back(value);
                else
                    values.push_back(-1);
		    }
	    }
	    return (int)values.size();
    }
}


bool TriangleMesh::loadObj(const char *file) {
    clear();

	ifstream f(file);
	if (!f) {
		cerr << "Couldn't open file: " << file << endl;
		return false;
	}
	string line;
	vector<int> first, second, third;
	vector<int> orig;
	while (getline(f,line)) {
		if (line.empty())
			continue;
		stringstream ss(line);
		string op;
		ss >> op;
		if (op.empty() || op[0] == '#')
			continue;
		if (op == "v") {
			dvec3 v;
			ss >> v[0] >> v[1] >> v[2];
			vertices.push_back(v);
		}
		if (op == "f")
		{
            if (!getNValues(ss, first))
				continue;
            if (!getNValues(ss, second))
                continue;
			while (getNValues(ss, third)) {
                triangles.push_back(Tri(first[0]-1, second[0]-1, third[0]-1));
                second = third;
			}
		}
	}

	computeNormals();

	return true;
}



} // namespace allquadrics

