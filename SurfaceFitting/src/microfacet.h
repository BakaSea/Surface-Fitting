#pragma once
#include <glm/glm.hpp>
using namespace glm;

float GGXNDF(float cosTheta, float alpha);

vec3 sampleGGXNDF(vec3 n, float alpha, vec2 u);