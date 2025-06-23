#version 410
// Vertex shader

layout(location = 0) in vec4 vert_coords_vs;
layout(location = 1) in vec4 vert_normal_vs;

layout(location = 0) out vec3 vert_coords_fs;
layout(location = 1) out vec3 vert_normal_fs;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 proj_matrix;
uniform mat3 normal_matrix;

void main() {
    vec4 vert_coords = vert_coords_vs;
    mat4 view_model = view_matrix * model_matrix;

    vert_coords_fs = vec3(view_matrix * model_matrix * vert_coords);
    vert_normal_fs = normalize(normal_matrix * normalize(vert_normal_vs.xyz));

    gl_Position = proj_matrix * view_model * vert_coords;
    gl_PointSize = 1;
}
