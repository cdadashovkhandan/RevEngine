#version 410
// Fragment shader

layout(location = 0) in vec3 vert_coords_fs;
layout(location = 1) in vec3 vert_color_fs;

out vec4 color_gl;

void main() {
    color_gl = vec4(vert_color_fs, 1.0);
}
