#pragma once
#include <vector>
#include <glm/glm.hpp>
using namespace glm;
using namespace std;

vector<vec3> clipTriangle(const vec3 triangle[3], vec3 bmin, vec3 bmax);