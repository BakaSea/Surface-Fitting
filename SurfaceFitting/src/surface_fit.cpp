#include "surface_fit.h"
#include <iostream>
#include "roots3and4.h"
#include "triangle_clip.h"
using namespace std;

void QuadricFit::addPoint(const dvec3& p, double w) {
	vertices++;
	//double prevWeightSum = weightSum;
	weightSum += w;
	//M *= prevWeightSum / weightSum;
	//N *= prevWeightSum / weightSum;
	//w /= weightSum;
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
	triangles.push_back({ tri[0], tri[1], tri[2] });
	dvec3 dtri[3] = { dvec3(tri[0]), dvec3(tri[1]), dvec3(tri[2]) };
	dvec3 e1 = tri[1] - tri[0], e2 = tri[2] - tri[0];
	dvec3 n = cross(e1, e2);
	double area = length(n) / 2.f;
	if (area <= 1e-6f)
		return;
	n = normalize(n);
	// Fitting Quadric
	for (int i = 0; i < 6; ++i) {
		dvec3 p = dtri[0] * (1.0 - dunavantX[i] - dunavantY[i]) + dtri[1] * dunavantX[i] + dtri[2] * dunavantY[i];
		//addPoint(p, 1);
		addPoint(p, dunavantW[i] * area);
		//addPoint(p, dunavantW[i]);
	}
	// Fitting SGGX
	//double prevNormalWeightSum = normalWeightSum;
	//normalWeightSum += area;
	//SigmaNormal *= prevNormalWeightSum / normalWeightSum;
	//double wNormal = area / normalWeightSum;
	//if (isnan(area) || isnan(n[0]) || isnan(n[1]) || isnan(n[2])) {
	//	cout << tri[0][0] << ' ' << tri[0][1] << ' ' << tri[0][2] << endl;
	//	cout << tri[1][0] << ' ' << tri[1][1] << ' ' << tri[1][2] << endl;
	//	cout << tri[2][0] << ' ' << tri[2][1] << ' ' << tri[2][2] << endl;
	//	cout << endl;
	//}
	//for (int i = 0; i < 3; ++i) {
	//	for (int j = 0; j < 3; ++j) {
	//		SigmaNormal(i, j) += wNormal * n[i] * n[j];
	//	}
	//}
	//normals.push_back({ area, n });

	areaSum += area;
}

bool isEllipsoid(VectorXd c) {
	Matrix3d q = Matrix3d::Zero();
	q(0, 0) = c(0);
	q(1, 1) = c(1);
	q(2, 2) = c(2);
	q(0, 1) = q(1, 0) = 0.5f * c(3);
	q(0, 2) = q(2, 0) = 0.5f * c(4);
	q(1, 2) = q(2, 1) = 0.5f * c(5);
	Vector3d eig = q.eigenvalues().real().normalized();
	if (eig(0) * eig(1) < -DBL_EPSILON || eig(0) * eig(2) < -DBL_EPSILON || eig(1) * eig(2) < -DBL_EPSILON) return false;
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
	b -= a;
	det3poly[0] = determinant(a);
	det3poly[1] = determinant(dmat3(b[0], a[1], a[2])) + determinant(dmat3(a[0], b[1], a[2])) + determinant(dmat3(a[0], a[1], b[2]));
	det3poly[2] = determinant(dmat3(a[0], b[1], b[2])) + determinant(dmat3(b[0], a[1], b[2])) + determinant(dmat3(b[0], b[1], a[2]));
	det3poly[3] = determinant(b);

	int nr3 = SolveCubic(det3poly, root), minRoot = -1;
	double minVal = -1.f;
	for (int i = 0; i < nr3; ++i) {
		VectorXd c = (1.0 - root[i]) * c1 + root[i] * c2;
		dmat3 q = getQMat(c);
		if (isEllipsoid(c)) {
			double err = getTaubinErr(c);
			if (minRoot == -1 || err < minVal) {
				minRoot = i;
				minVal = err;
			}
		}
	}
	if (minRoot == -1) {
		//if (isEllipsoid(c2)) {
		//	for (int i = 0; i < 4; ++i) {
		//		cout << det3poly[i] << ' ';
		//	}
		//	cout << endl;
		//	for (int i = 0; i < nr3; ++i) {
		//		cout << root[i] << ' ';
		//		VectorXd c = (1.0 - root[i]) * c1 + root[i] * c2;
		//		dmat3 q = getQMat(c);
		//		cout << "Det: " << determinant(q) << endl;
		//	}
		//	cout << endl;
		//	cout << c1 << endl;
		//	cout << endl;
		//	cout << c2 << endl;
		//	cout << endl;
		//}
		return false;
	}
	t = root[minRoot];
	return true;
}

Quadric QuadricFit::fitQuadric() const {
	GeneralizedEigenSolver<MatrixXd> ges;
	MatrixXd wM = M / weightSum, wN = N / weightSum;
	//ges.compute(M, N);
	ges.compute(wM, wN);
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
	if (secCol == -1) {
		secCol = minCol;
	}
	VectorXd minC = ges.eigenvectors().col(minCol).real();
	VectorXd secC = ges.eigenvectors().col(secCol).real();
	if (isEllipsoid(minC)) {
		for (int i = 0; i < 10; ++i) {
			c[i] = minC(i);
		}
		//double var = minC.transpose() * M * minC;
		double var = minC.transpose() * wM * minC;
		if (var < 0) {
			return Quadric(c, FLT_EPSILON);
		}
		return Quadric(c, sqrt(var));
	}
	double t;
	if (lineSearch(minC, secC, t)) {
		//VectorXd vc = minC + t * secC;
		VectorXd vc = (1.0 - t) * minC + t * secC;
		for (int i = 0; i < 10; ++i) {
			c[i] = vc(i);
		}
		//double var = vc.transpose() * M * vc;
		double var = vc.transpose() * wM * vc;
		if (var < 0) {
			return Quadric(c, FLT_EPSILON);
		}
		return Quadric(c, sqrt(var));
	}

	MatrixXd NN = MatrixXd::Zero(10, 10);
	NN(0, 0) = NN(1, 1) = NN(2, 2) = -1;
	NN(0, 1) = NN(1, 0) = NN(0, 2) = NN(2, 0) = NN(1, 2) = NN(2, 1) = 1;
	NN(3, 3) = NN(4, 4) = NN(5, 5) = -1;
	//ges.compute(M, NN);
	ges.compute(wM, NN);
	alphas = ges.alphas();
	betas = ges.betas();
	secCol = -1;
	secVal = -1.0;
	for (int i = 0; i < 10; ++i) {
		if (fabs(betas[i]) > 0) {
			VectorXd vc = ges.eigenvectors().col(i).real();
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
	if (lineSearch(minC, secC, t)) {
		VectorXd vc = (1.0 - t) * minC + t * secC;
		for (int i = 0; i < 10; ++i) {
			c[i] = vc[i];
		}
		//double var = vc.transpose() * M * vc;
		double var = vc.transpose() * wM * vc;
		if (var < 0) {
			return Quadric(c, FLT_EPSILON);
		}
		return Quadric(c, sqrt(var));
	}
	for (int i = 0; i < 10; ++i) {
		c[i] = secC(i);
	}
	//cout << 3 << endl;
	//double var = secC.transpose() * M * secC;
	double var = secC.transpose() * wM * secC;
	if (var < 0) {
		return Quadric(c, FLT_EPSILON);
	}
	return Quadric(c, sqrt(var));
}

void coordinateSystem(const vec3 &n, vec3 &s, vec3 &t) {
	float si = sign(n.z);
	float a = -1.f / (si + n.z);
	float b = n.x * n.y * a;
	s = vec3(1.f + si * n.x * n.x * a, si * b, -si * n.x);
	t = vec3(b, si + n.y * n.y * a, -n.y);
}

SGGX QuadricFit::fitSGGX(const Quadric& q) const {
	mat3 S(0.f);
	float normalWeightSum = 0.f;
	for (auto& tri : triangles) {
		vec3 e1 = tri[1] - tri[0], e2 = tri[2] - tri[0];
		vec3 n = cross(e1, e2);
		float area = length(n) / 2.f;
		n = normalize(n);
		vec3 qn = normalize(q.df((tri[0] + tri[1] + tri[2]) / 3.f)), s, t;
		coordinateSystem(qn, s, t);
		n = vec3(dot(s, n), dot(t, n), dot(qn, n));

		S[0][0] += area * glm::clamp(n.x, 0.f, 1.f);
		S[0][0] += area * glm::clamp(-n.x, 0.f, 1.f);
		S[0][0] += area * glm::clamp(n.y, 0.f, 1.f);
		S[0][0] += area * glm::clamp(-n.y, 0.f, 1.f);

		S[1][1] += area * glm::clamp(n.x, 0.f, 1.f);
		S[1][1] += area * glm::clamp(-n.x, 0.f, 1.f);
		S[1][1] += area * glm::clamp(n.y, 0.f, 1.f);
		S[1][1] += area * glm::clamp(-n.y, 0.f, 1.f);

		//S[2][2] += 2.f * area * glm::clamp(n.z, 0.f, 1.f);
		//S[2][2] += 2.f * area * glm::clamp(-n.z, 0.f, 1.f);
		S[2][2] += 4.f * area;

		normalWeightSum += 4.f * area;
	}
	S /= normalWeightSum;
	for (int i = 0; i < 3; ++i) {
		S[i][i] *= S[i][i];
		if (isnan(S[i][i])) {
			cout << "shit" << endl;
		}
	}
	return SGGX(S);
}

const int AREA_SAMPLES = 256;

float QuadricFit::fitAlpha(const Quadric& q, const vec3& bmin, const vec3& bmax) const {
	float triAreaSum = 0.f, surfArea = 0.f;
	for (auto& tri : triangles) {
		vec3 e1 = tri[1] - tri[0], e2 = tri[2] - tri[0];
		vec3 n = cross(e1, e2);
		float area = length(n) / 2.f;
		n = normalize(n);
		vec3 qn = normalize(q.df((tri[0] + tri[1] + tri[2]) / 3.f));
		area *= abs(dot(qn, n));
		triAreaSum += area;
	}
	vec3 cap = bmax - bmin;
	float pdf = 1.f / (cap.x * cap.y);
	for (int i = 0; i < AREA_SAMPLES; ++i) {
		float x = bmin.x + cap.x * rand() / RAND_MAX;
		float y = bmin.y + cap.y * rand() / RAND_MAX;
		//float x = bmin.x + cap.x * (i / 16) / 16.f;
		//float y = bmin.y + cap.y * (i % 16) / 16.f;
		float a = q.c[2], b = q.c[4] * x + q.c[5] * y + q.c[8], c = q.f(vec3(x, y, 0));
		float delta = b * b - 4 * a * c;
		vec3 p(x, y, 0);
		if (delta > FLT_EPSILON) {
			float upProb = rand() / RAND_MAX;
			if (upProb < .5f) {
				p.z = (-b + sqrt(delta)) / (2 * a);
				if (inBox(p, bmin, bmax)) {
					vec3 dF = q.df(p);
					if (dF.z != 0.f) {
						float fx = -dF.x / dF.z;
						float fy = -dF.y / dF.z;
						surfArea += sqrt(1 + fx * fx + fy * fy) / (pdf * .5f);
					}
				}
			} else {
				p.z = (-b - sqrt(delta)) / (2 * a);
				if (inBox(p, bmin, bmax)) {
					vec3 dF = q.df(p);
					if (dF.z != 0.f) {
						float fx = -dF.x / dF.z;
						float fy = -dF.y / dF.z;
						surfArea += sqrt(1 + fx * fx + fy * fy) / (pdf * .5f);
					}
				}
			}
		} else if (delta >= -FLT_EPSILON) {
			p.z = (-b) / (2 * a);
			if (inBox(p, bmin, bmax)) {
				vec3 dF = q.df(p);
				if (dF.z != 0.f) {
					float fx = -dF.x / dF.z;
					float fy = -dF.y / dF.z;
					surfArea += sqrt(1 + fx * fx + fy * fy) / pdf;
				}
			}
		}
	}
	surfArea /= 1.f * AREA_SAMPLES;
	if (surfArea == 0.f)
		return 0.f;
	return triAreaSum / surfArea;
}
