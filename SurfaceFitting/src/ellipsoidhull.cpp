#include "ellipsoidhull.h"
#include <Eigen/Dense>
#include <limits>

using namespace Eigen;

vec4 compute_bounding_sphere(std::span<const vec3> points, const vec3 &V)
{
    int N = (int)points.size();
    float max_dot = -std::numeric_limits<float>::infinity();
    int max_dot_idx = 0;
    float min_dot = std::numeric_limits<float>::infinity();
    int min_dot_idx = 0;

    vec3 center(0);
    for (int i = 0; i < N; ++i) {
        center += points[i];
        //
        float d = dot(points[i], V);
        if (d > max_dot) {
            max_dot = d;
            max_dot_idx = i;
        }
        if (d < min_dot) {
            min_dot = d;
            min_dot_idx = i;
        }
    }
    center /= (float)N;

    vec3 q = 0.5f * (points[max_dot_idx] + points[min_dot_idx]);
    float r = length(q - points[max_dot_idx]);
    float r2 = r * r;

    float rr2 = 0.0f;
    for (int i = 0; i < N; ++i) {
        float l = length(points[i] - center);
        rr2 = std::max(rr2, l*l);
        float ll = length(points[i] - q);
        if (ll * ll > r2) {
            vec3 g = q - r * normalize(points[i] - q);
            q = 0.5f * (g + points[i]);
            r = length(q - points[i]);
            r2 = r * r;
        }
    }
    float rr = std::sqrt(rr2);
    if (rr < r) {
        return vec4(center.x, center.y, center.z, rr);
    } else {
        return vec4(q.x, q.y, q.z, r);
    }
}

void symeig3x3(float cov00, float cov01, float cov02, float cov11, float cov12, float cov22, vec3& eigenvalues, std::array<vec3, 3>& eigenvectors) {
    Matrix3f cov;
    cov(0, 0) = cov00;
    cov(0, 1) = cov(1, 0) = cov01;
    cov(0, 2) = cov(2, 0) = cov02;
    cov(1, 1) = cov11;
    cov(1, 2) = cov(2, 1) = cov12;
    cov(2, 2) = cov22;
    EigenSolver<Matrix3f> solver;
    solver.compute(cov);
    Vector3cf eval = solver.eigenvalues();
    eigenvalues = vec3(eval[0].real(), eval[1].real(), eval[2].real());
    for (int i = 0; i < 3; ++i) {
        Vector3cf evec = solver.eigenvectors().col(i);
        eigenvectors[i] = vec3(evec[0].real(), evec[1].real(), evec[2].real());
    }
}

Ellipsoid compute_bounding_ellipsoid(std::span<const dvec3> points)
{
    int N = (int)points.size();
    vec3 mean(0);
    for (int i = 0; i < N; ++i) {
        mean += points[i];
    }
    mean /= (float)N;
    float cov00 = 0.0f, cov01 = 0.0f, cov02 = 0.0f, cov11 = 0.0f, cov12 = 0.0f, cov22 = 0.0f;
    for (int i = 0; i < N; ++i) {
        float dx = points[i].x - mean.x;
        float dy = points[i].y - mean.y;
        float dz = points[i].z - mean.z;
        cov00 += dx * dx;
        cov01 += dx * dy;
        cov02 += dx * dz;
        cov11 += dy * dy;
        cov12 += dy * dz;
        cov22 += dz * dz;
    }
    cov00 /= (float)N;
    cov01 /= (float)N;
    cov02 /= (float)N;
    cov11 /= (float)N;
    cov12 /= (float)N;
    cov22 /= (float)N;
    vec3 eigenvalues;
    std::array<vec3, 3> eigenvectors;
    symeig3x3(cov00, cov01, cov02, cov11, cov12, cov22, eigenvalues, eigenvectors);

    std::array<vec2, 3> ext;
    for (int j = 0; j < 3; ++j) {
        ext[j][0] = std::numeric_limits<float>::infinity();
        ext[j][1] = -std::numeric_limits<float>::infinity();
    }
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < 3; ++j) {
            float d = dot(vec3(points[i]), eigenvectors[j]);
            ext[j][0] = std::min(ext[j][0], d);
            ext[j][1] = std::max(ext[j][1], d);
        }
    }
    mat3 S = mat3(1.f);
    mat3 Sinv = mat3(1.f);
    for (int j = 0; j < 3; ++j) {
        float s = ext[j][1] - ext[j][0];
        // Avoid degeneracy
        s = std::max(s, 1e-4f);
        S[j][j] = s;
        Sinv[j][j] = 1.0f / s;
    }
    mat3 R;
    for (int j = 0; j < 3; ++j) {
        R[j] = eigenvectors[j];
    }
    mat3 M = R * Sinv * transpose(R);
    mat3 Minv = R * S * transpose(R);

    std::vector<vec3> mp(N);
    for (int i = 0; i < N; ++i) {
        mp[i] = M * points[i];
    }

    int scaled_max_axis = 0;
    float scaled_max_abs_eigenvalue = 0.0f;
    for (int j = 0; j < 3; ++j) {
        float scaled_abs_eigenvalue = std::abs(eigenvalues[j]) / (S[j][j] * S[j][j]);
        if (scaled_max_abs_eigenvalue < scaled_abs_eigenvalue) {
            scaled_max_abs_eigenvalue = scaled_abs_eigenvalue;
            scaled_max_axis = j;
        }
    }
    vec4 sphere = compute_bounding_sphere(mp, eigenvectors[scaled_max_axis]);

    Ellipsoid e;
    // semi-axes
    for (int j = 0; j < 3; ++j) {
        e.Q[j] = (1.0f / sphere[3]) * Sinv[j][j] * eigenvectors[j];
    }
    e.Q = transpose(e.Q);
    // center
    e.center = Minv * vec3(sphere);

    return e;
}
