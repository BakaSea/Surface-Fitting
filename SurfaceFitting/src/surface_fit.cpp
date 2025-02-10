#include "surface_fit.h"
#include <iostream>
#include "roots3and4.h"
#include "triangle_clip.h"
using namespace std;

void QuadricFit::addPoint(const dvec3& p, double w) {
	vertices++;
	double prevWeightSum = weightSum;
	weightSum += w;
	M *= prevWeightSum / weightSum;
	N *= prevWeightSum / weightSum;
	w /= weightSum;
	double x = p.x, y = p.y, z = p.z;
	double c[10] = {
		x*x, y*y, z*z,
		x*y, x*z, y*z,
		x, y, z, 1
	};
	double cx[10], cy[10], cz[10];
	memset(cx, 0, sizeof(cx));
	memset(cy, 0, sizeof(cy));
	memset(cz, 0, sizeof(cz));
	cx[0] = 2.f * x; cx[3] = y; cx[4] = z; cx[6] = 1.f;
	cy[1] = 2.f * y; cy[3] = x; cy[5] = z; cy[7] = 1.f;
	cz[2] = 2.f * z; cz[4] = x; cz[5] = y; cz[8] = 1.f;
	for (int i = 0; i < 10; ++i) {
		for (int j = 0; j < 10; ++j) {
			M(i, j) += w * c[i] * c[j];
			N(i, j) += w * (cx[i] * cx[j] + cy[i] * cy[j] + cz[i] * cz[j]);
		}
	}
}

const double dunavantW[6] = {0.223381589678011, 0.223381589678011, 0.223381589678011, 0.109951743655322, 0.109951743655322, 0.109951743655322};
const double dunavantX[6] = {0.10810301816807, 0.445948490915965, 0.445948490915965, 0.81684757298045896, 0.091576213509771007, 0.091576213509771007};
const double dunavantY[6] = {0.445948490915965, 0.445948490915965, 0.10810301816807, 0.091576213509771007, 0.091576213509771007, 0.81684757298045896};

void QuadricFit::addTriangle(const vec3 tri[3]) {
	dvec3 dtri[3] = { dvec3(tri[0]), dvec3(tri[1]), dvec3(tri[2]) };
	dvec3 e1 = tri[1] - tri[0], e2 = tri[2] - tri[0];
	dvec3 n = cross(e1, e2);
	double area = length(n) / 2.f;
	n = normalize(n);
	// Fitting Quadric
	for (int i = 0; i < 6; ++i) {
		dvec3 p = dtri[0] * (1.0 - dunavantX[i] - dunavantY[i]) + dtri[1] * dunavantX[i] + dtri[2] * dunavantY[i];
		//addPoint(p, 1);
		addPoint(p, dunavantW[i] * area);
		//addPoint(p, dunavantW[i]);
	}

	// Fitting SGGX
	double prevNormalWeightSum = normalWeightSum;
	normalWeightSum += area;
	SigmaNormal *= prevNormalWeightSum / normalWeightSum;
	normalSum *= prevNormalWeightSum / normalWeightSum;
	double wNormal = area / normalWeightSum;
	if (isnan(area) || isnan(n[0]) || isnan(n[1]) || isnan(n[2])) {
		cout << tri[0][0] << ' ' << tri[0][1] << ' ' << tri[0][2] << endl;
		cout << tri[1][0] << ' ' << tri[1][1] << ' ' << tri[1][2] << endl;
		cout << tri[2][0] << ' ' << tri[2][1] << ' ' << tri[2][2] << endl;
		cout << endl;
	}
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			SigmaNormal(i, j) += wNormal * n[i] * n[j];
		}
	}
	normalSum += wNormal * n;
	normals.push_back({ area, n });

	// Density Calculation
	//sa += area;
	//vec3 volumeBox = (bmax - bmin) / (1.f * DENSITY_SAMPLES);
	//for (int i = 0; i < DENSITY_SAMPLES; ++i) {
	//	for (int j = 0; j < DENSITY_SAMPLES; ++j) {
	//		for (int k = 0; k < DENSITY_SAMPLES; ++k) {
	//			vec3 vbmin = bmin + volumeBox * vec3(i, j, k);
	//			vec3 vbmax = bmin + volumeBox * vec3(i + 1, j + 1, k + 1);
	//			vector<vec3> points = clipTriangle(tri, vbmin, vbmax);
	//			if (points.size() >= 3) {
	//				for (int p = 1; p <= points.size() - 2; ++p) {
	//					vec3 ce1 = points[p] - points[0], ce2 = points[p + 1] - points[0];
	//					vec3 cn = cross(ce1, ce2);
	//					float carea = length(cn) / 2.f;
	//					areas[i][j][k] += carea;
	//				}
	//			}
	//		}
	//	}
	//}
}


bool isEllipsoid(VectorXd c) {
	Matrix3f q = Matrix3f::Zero();
	q(0, 0) = c(0);
	q(1, 1) = c(1);
	q(2, 2) = c(2);
	q(0, 1) = q(1, 0) = 0.5f * c(3);
	q(0, 2) = q(2, 0) = 0.5f * c(4);
	q(1, 2) = q(2, 1) = 0.5f * c(5);
	Vector3f eig = q.eigenvalues().real().normalized();
	if (eig(0) * eig(1) < -2e-4f || eig(0) * eig(2) < -2e-4f || eig(1) * eig(2) < -2e-4f) return false;
	return true;
}

inline dmat3 getQMat(VectorXd c) {
	dmat3 m(0.f);
	m[0][0] = c(0);
	m[1][1] = c(1);
	m[2][2] = c(2);
	m[0][1] = m[1][0] = 0.5f * c(3);
	m[0][2] = m[2][0] = 0.5f * c(4);
	m[1][2] = m[2][1] = 0.5f * c(5);
	return m;
}

bool QuadricFit::lineSearch(VectorXd c1, VectorXd c2, double& t) const {
	double det3poly[4], root[3];
	dmat3 a = getQMat(c1), b = getQMat(c2);
	det3poly[0] = determinant(a);
	det3poly[1] = determinant(dmat3(b[0], a[1], a[2])) + determinant(dmat3(a[0], b[1], a[2])) + determinant(dmat3(a[0], a[1], b[2]));
	det3poly[2] = determinant(dmat3(a[0], b[1], b[2])) + determinant(dmat3(b[0], a[1], b[2])) + determinant(dmat3(b[0], b[1], a[2]));
	det3poly[3] = determinant(b);
	//det3poly[0] = stp(a[0], a[1], a[2]);
	//det3poly[1] = stp(b[0], a[1], a[2]) + stp(a[0], b[1], a[2]) + stp(a[0], a[1], b[2]);
	//det3poly[2] = stp(a[0], b[1], b[2]) + stp(b[0], a[1], b[2]) + stp(b[0], b[1], a[2]);
	//det3poly[3] = stp(b[0], b[1], b[2]);
	int nr3 = SolveCubic(det3poly, root), minRoot = -1;
	double minVal = -1.f;
	for (int i = 0; i < nr3; ++i) {
		VectorXd c = c1 + root[i] * c2;
		dmat3 q = getQMat(c);
		//cout << "Det: " << determinant(q) << endl;
		if (isEllipsoid(c)) {
			double err = getTaubinErr(c);
			if (minRoot == -1 || err < minVal) {
				minRoot = i;
				minVal = err;
			}
		}
	}
	if (minRoot == -1) {
		//for (int i = 0; i < 4; ++i) {
		//	cout << det3poly[i] << ' ';
		//}
		//cout << endl;
		//for (int i = 0; i < nr3; ++i) {
		//	cout << root[i] << ' ';
		//}
		//cout << endl;
		//cout << c1 << endl;
		//cout << endl;
		//cout << c2 << endl;
		//cout << endl;
		//for (int i = 0; i < nr3; ++i) {
		//	VectorXf c = c1 + root[i] * c2;
		//	cout << c(0) << ' ' << c(1) << ' ' << c(2) << endl;
		//}
		//cout << endl;
		return false;
	}
	t = root[minRoot];
	return true;
}

Quadric QuadricFit::fitQuadric() const {
	GeneralizedEigenSolver<MatrixXd> ges;
	ges.compute(M, N);
	auto alphas = ges.alphas();
	auto betas = ges.betas();
	int minCol = -1, secCol = -1;
	double minVal = -1.0, secVal = -1.0;
	for (int i = 0; i < 10; ++i) {
		if (fabs(betas[i]) > 0) {
			double eigenVal = fabs(alphas[i].real() / betas[i]);
			if (eigenVal > 0) {
				if (minCol == -1 || eigenVal <= minVal) {
					secCol = minCol;
					secVal = minVal;
					minCol = i;
					minVal = eigenVal;
				} else if (eigenVal < secVal) {
					secCol = i;
					secVal = eigenVal;
				}
			}
		}
	}

	float c[10] = {0};

	if (minCol == -1) {
		cout << "f" << endl;
		return Quadric(c, -1);
	}
	//VectorXf vc = ges.eigenvectors().col(minCol).real();
	//for (int i = 0; i < 10; ++i)
	//	c[i] = vc[i];
	//float var = vc.transpose() * M * vc;
	//float var = minVal * vc.transpose() * N * vc;
	//return Quadric(c, sqrt(var));

	VectorXd minC = ges.eigenvectors().col(minCol).real();
	VectorXd secC = ges.eigenvectors().col(secCol).real();
	if (isEllipsoid(minC)) {
		for (int i = 0; i < 10; ++i) {
			c[i] = minC(i);
		}
		double var = minC.transpose() * M * minC;
		return Quadric(c, sqrt(var));
	}
	double t;
	if (lineSearch(minC, secC, t)) {
		VectorXd vc = minC + t * secC;
		for (int i = 0; i < 10; ++i) {
			c[i] = vc(i);
		}
		double var = vc.transpose() * M * vc;
		return Quadric(c, sqrt(var));
	}

	MatrixXd NN = MatrixXd::Zero(10, 10);
	NN(0, 0) = NN(1, 1) = NN(2, 2) = -1;
	NN(0, 1) = NN(1, 0) = NN(0, 2) = NN(2, 0) = NN(1, 2) = NN(2, 1) = 1;
	NN(3, 3) = NN(4, 4) = NN(5, 5) = -1;
	ges.compute(M, NN);
	alphas = ges.alphas();
	betas = ges.betas();
	secCol = -1;
	secVal = -1.0;
	for (int i = 0; i < 10; ++i) {
		if (fabs(betas[i]) > 0) {
			VectorXd vc = ges.eigenvectors().col(i).real();
			//mat3 Qmat = getQMat(vc);
			//double det2 = Qmat[0][0] * Qmat[1][1] - Qmat[1][0] * Qmat[0][1];
			//double det3 = determinant(Qmat);
			//double det1 = Qmat[0][0];
			//if (det2 > 0 && ((det1 > 0 && det3 > 0) || (det1 < 0 && det3 < 0))) {
			if (isEllipsoid(vc)) {
				double err = getTaubinErr(vc);
				if (secCol == -1 || err < secVal) {
					secCol = i;
					secVal = err;
				}
			}
		}
	}
	if (secCol == -1) {
		cout << "ff" << endl;
		return Quadric(c, -1.f);
	}
	secC = ges.eigenvectors().col(secCol).real();
	//cout << 1 << endl;
	if (lineSearch(minC, secC, t)) {
		//cout << 2 << endl;
		VectorXd vc = minC + t * secC;
		for (int i = 0; i < 10; ++i) {
			c[i] = vc[i];
		}
		double var = vc.transpose() * M * vc;
		return Quadric(c, sqrt(var));
	}
	for (int i = 0; i < 10; ++i) {
		c[i] = secC(i);
	}
	cout << 3 << endl;
	double var = secC.transpose() * M * secC;
	return Quadric(c, sqrt(var));
}

SGGX QuadricFit::fitSGGX() const {
	SelfAdjointEigenSolver<Matrix3d> saes;
	saes.compute(SigmaNormal);
	mat3 omegas;
	vec3 sigma(0.f);
	float ws = 0;
	for (int i = 0; i < normals.size(); ++i) {
		ws += normals[i].first;
	}
	for (int i = 0; i < 3; ++i) {
		Vector3d omega = saes.eigenvectors().col(i);
		omegas[i] = vec3(omega.x(), omega.y(), omega.z());
		//sigma[i] = dot(normalSum, omegas[i]);
		for (int j = 0; j < normals.size(); ++j) {
			//sigma[i] += normals[j].first * glm::clamp(dot(normals[j].second, omegas[i]), 0.f, 1.f);
			sigma[i] += normals[j].first * fabs(dot(normals[j].second, omegas[i]));
		}
		sigma[i] /= ws;
	}
	mat3 diag(0.f);
	for (int i = 0; i < 3; ++i) {
		//cout << sigma[i] << ' ';
		diag[i][i] = sigma[i] * sigma[i];
	}
	//cout << endl;
	mat3 S = omegas * diag * transpose(omegas);
	return SGGX(S);
}

//float QuadricFit::getDensity() const {
//	//vec3 volumeBox = (bmax - bmin) / (1.f * DENSITY_SAMPLES);
//	//float volumeSize = volumeBox.x * volumeBox.y * volumeBox.z;
//	//float volumeSum = 0.f, areaSum = 0.f;
//	//for (int i = 0; i < DENSITY_SAMPLES; ++i) {
//	//	for (int j = 0; j < DENSITY_SAMPLES; ++j) {
//	//		for (int k = 0; k < DENSITY_SAMPLES; ++k) {
//	//			if (areas[i][j][k] > FLT_EPSILON) {
//	//				areaSum += areas[i][j][k];
//	//				volumeSum += volumeSize;
//	//			}
//	//		}
//	//	}
//	//}
//	//return areaSum / volumeSum;
//	vec3 volumeBox = bmax - bmin;
//	float volumeSize = volumeBox.x * volumeBox.y * volumeBox.z;
//	return sa / volumeSize;
//}
