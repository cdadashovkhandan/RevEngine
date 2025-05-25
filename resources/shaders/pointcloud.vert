#version 410
// Vertex shader

layout(location = 0) in vec3 vert_coords_vs;
layout(location = 1) in vec3 vert_normal_vs;

layout(location = 0) out vec3 vert_coords_fs;
layout(location = 1) out vec3 vert_normal_fs;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 proj_matrix;
uniform mat3 normal_matrix;

void main() {
    vec4 vert_coords = vec4(vert_coords_vs, 1.0);
    // mat4 view_model = view_matrix * model_matrix;

    // vert_coords_fs = vec3(view_matrix * model_matrix * vert_coords);
    // vert_normal_fs = normalize(normal_matrix * normalize(vert_normal_vs));

    gl_Position = vert_coords;
    gl_PointSize = 24;
}
