#include "surface_fit.h"
#include <iostream>

void QuadricFit::addPoint(const vec3& p, float w) {
	vertices++;
	float x = p.x, y = p.y, z = p.z;
	float c[10] = {
		x*x, y*y, z*z,
		x*y, x*z, y*z,
		x, y, z, 1
	};
	float cx[10], cy[10], cz[10];
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

const float dunavantW[6] = {0.223381589678011, 0.223381589678011, 0.223381589678011, 0.109951743655322, 0.109951743655322, 0.109951743655322};
const float dunavantX[6] = {0.10810301816807, 0.445948490915965, 0.445948490915965, 0.81684757298045896, 0.091576213509771007, 0.091576213509771007};
const float dunavantY[6] = {0.445948490915965, 0.445948490915965, 0.10810301816807, 0.091576213509771007, 0.091576213509771007, 0.81684757298045896};

void QuadricFit::addTriangle(const vec3 tri[3]) {
	vec3 bmin = min(tri[0], min(tri[1], tri[2]));
	vec3 bmax = max(tri[0], max(tri[1], tri[2]));
	for (int i = 0; i < 6; ++i) {
		vec3 p = tri[0] * (1.f - dunavantX[i] - dunavantY[i]) + tri[1] * dunavantX[i] + tri[2] * dunavantY[i];
		assert(inBox(p, bmin, bmax));
		addPoint(p, dunavantW[i]);
	}
}

Quadric QuadricFit::fitQuadric() const {
	GeneralizedEigenSolver<MatrixXf> ges;
	ges.compute(M, N);
	auto alphas = ges.alphas();
	auto betas = ges.betas();
	int minCol = -1;
	float minVal = -1.f;
	for (int i = 0; i < 10; ++i) {
		if (fabs(betas[i]) > 0) {
			float eigenVal = fabs(alphas[i].real() / betas[i]);
			if (minCol == -1 || eigenVal < minVal) {
				minCol = i;
				minVal = eigenVal;
			}
		}
	}
	float c[10];
	for (int i = 0; i < 10; ++i)
		c[i] = ges.eigenvectors().col(minCol)[i].real();
	return Quadric(c, sqrt(minVal));
}
