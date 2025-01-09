#pragma once
#include <vector>
#include <glm/glm.hpp>
using namespace std;
using namespace glm;

vector<vec3> clipTriangle(const vec3 triangle[3], vec3 bmin, vec3 bmax) {
    vector<vec3> res(3), p;
    for (int i = 0; i < 3; ++i)
        res[i] = triangle[i];
    for (int k = 0; k < 3; ++k) {
        p = res;
        res.clear();
        for (int i = 0; i < p.size(); ++i) {
            vec3 e = p[(i + 1) % p.size()] - p[i];
            if (e[k] != 0) {
                float t = (bmin[k] - p[i][k]) / e[k];
                if (0 < t && t < 1) {
                    res.emplace_back(p[i] + t * e);
                }
            }
            if (p[(i + 1) % p.size()][k] >= bmin[k]) {
                res.emplace_back(p[(i + 1) % p.size()]);
            }
        }
        p = res;
        res.clear();
        for (int i = 0; i < p.size(); ++i) {
            vec3 e = p[(i + 1) % p.size()] - p[i];
            if (e[k] != 0) {
                float t = (bmax[k] - p[i][k]) / e[k];
                if (0 < t && t < 1) {
                    res.emplace_back(p[i] + t * e);
                }
            }
            if (p[(i + 1) % p.size()][k] <= bmax[k]) {
                res.emplace_back(p[(i + 1) % p.size()]);
            }
        }
    }
    return res;
}