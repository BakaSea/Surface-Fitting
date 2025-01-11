#version 430 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D tex;

vec3 tonemapping(vec3 color) {
    return pow(color, vec3(1.f/2.2f));
}

void main() {
    vec3 texCol = texture(tex, TexCoords).rgb;
    FragColor = vec4(tonemapping(texCol), 1.0);
}