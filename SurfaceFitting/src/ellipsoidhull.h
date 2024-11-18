#pragma once
#include <glm/glm.hpp>
#include <span>

using namespace glm;

struct Ellipsoid {
    Ellipsoid() = default;
    Ellipsoid(const mat3& Q, const vec3& center) : Q(Q), center(center) {}

    // Q is the "half" of the standard positive definite ellipsoid matrix M: M = Q^t Q
    // Q =
    // diag(1/a, 1/b, 1/c) * (X, Y, Z)^T
    // such that the ellipsoid is defined as
    // (x - center)^T Q^t Q (x - center) <= 1
    mat3 Q;
    vec3 center;

    void getParam(float q[10]) {
        mat3 A = transpose(Q) * Q;
        vec3 b = -2.f * transpose(A) * center;
        float c = dot(center, A * center) - 1.f;
        q[0] = A[0][0];
        q[1] = A[1][1];
        q[2] = A[2][2];
        q[3] = 2.f * A[1][0];
        q[4] = 2.f * A[2][0];
        q[5] = 2.f * A[2][1];
        q[6] = b.x;
        q[7] = b.y;
        q[8] = b.z;
        q[9] = c;
    }
};

Ellipsoid compute_bounding_ellipsoid(std::span<const dvec3> points);