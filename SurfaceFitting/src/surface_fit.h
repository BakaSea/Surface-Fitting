#pragma once
#include <glm/glm.hpp>
#include <Eigen/Dense>
#include <vector>
using namespace glm;
using namespace Eigen;

struct Quadric {
	
	float c[10], sigma;

	Quadric() {
		for (int i = 0; i < 10; ++i)
			c[i] = 0;
		sigma = 0;
	}

	Quadric(float param[10], float s) {
		for (int i = 0; i < 10; ++i)
			c[i] = param[i];
		sigma = s;
	}

	float f(const vec3& p) const {
		float x = p.x, y = p.y, z = p.z;
		return c[0] * x * x + c[1] * y * y + c[2] * z * z + c[3] * x * y + c[4] * x * z + c[5] * y * z + c[6] * x + c[7] * y + c[8] * z + c[9];
	}

	vec3 df(const vec3& p) const {
		float x = p.z, y = p.y, z = p.z;
		return vec3(2.f * c[0] * x + c[3] * y + c[4] * z + c[6],
			2.f * c[1] * y + c[3] * x + c[5] * z + c[7],
			2.f * c[2] * z + c[4] * x + c[5] * y + c[8]);
	}

};

struct SGGX {

	float S_xx, S_xy, S_xz, S_yy, S_yz, S_zz;

	SGGX() {
		S_xx = S_xy = S_xz = S_yy = S_yz = S_zz = 0;
	}

	SGGX(mat3 S) {
		S_xx = S[0][0];
		S_xy = S[0][1];
		S_xz = S[0][2];
		S_yy = S[1][1];
		S_yz = S[1][2];
		S_zz = S[2][2];
	}

};

struct QuadricFit {

	MatrixXd M, N;
	Matrix3d SigmaNormal;
	dvec3 normalSum;
	double weightSum;
	double normalWeightSum;
	int vertices;
	std::vector<std::pair<double, vec3>> normals;

	QuadricFit() {
		M = MatrixXd::Zero(10, 10);
		N = MatrixXd::Zero(10, 10);
		SigmaNormal = Matrix3d::Zero();
		normalSum = dvec3(0.f);
		weightSum = 0.f;
		normalWeightSum = 0;
		vertices = 0;
		normals.clear();
	}

	void addPoint(const dvec3& p, double w);

	void addTriangle(const vec3 tri[3]);

	double getTaubinErr(const VectorXd& c) const {
		double mSum = c.transpose() * M * c;
		double nSum = c.transpose() * N * c;
		return mSum / nSum;
	}

	bool lineSearch(VectorXd c1, VectorXd c2, double& t) const;

	Quadric fitQuadric() const;

	SGGX fitSGGX() const;

};

inline bool inBox(vec3 point, vec3 bmin, vec3 bmax) {
	return bmin.x - std::numeric_limits<float>::epsilon() <= point.x && point.x <= bmax.x + std::numeric_limits<float>::epsilon()
		&& bmin.y - std::numeric_limits<float>::epsilon() <= point.y && point.y <= bmax.y + std::numeric_limits<float>::epsilon()
		&& bmin.z - std::numeric_limits<float>::epsilon() <= point.z && point.z <= bmax.z + std::numeric_limits<float>::epsilon();
}