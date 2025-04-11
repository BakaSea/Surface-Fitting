#include "microfacet.h"
#include <glm/gtc/constants.hpp>

float GGXNDF(float cosTheta, float alpha) {
	float cos2Theta = cosTheta * cosTheta;
	float a2 = alpha * alpha;
	float d = cos2Theta*(a2-1.f)+1.f;
	return a2 / (glm::pi<float>() * d * d);
}

vec3 sampleGGXNDF(vec3 n, float alpha, vec2 u) {
	float phi = 2.f * glm::pi<float>() * u.x;
	float cosTheta = sqrt((1.f - u.y) / (1.f + (alpha * alpha - 1.f) * u.y));
	float sinTheta = sqrt(1.f - cosTheta * cosTheta);
	vec3 h;
	h.x = cos(phi) * sinTheta;
	h.y = sin(phi) * sinTheta;
	h.z = cosTheta;
	vec3 up = fabs(n.z) < 0.999 ? vec3(0, 0, 1) : vec3(1, 0, 0);
	vec3 tangent = normalize(cross(up, n));
	vec3 bitangent = cross(n, tangent);
	vec3 sampleVec = tangent * h.x + bitangent * h.y + n * h.z;
	return normalize(sampleVec);
}
