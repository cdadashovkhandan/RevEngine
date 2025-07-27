#version 410
// Vertex shader

layout(location = 0) in vec4 vert_coords_vs;
layout(location = 1) in vec3 vert_color_vs;

layout(location = 0) out vec3 vert_coords_fs;
layout(location = 1) out vec3 vert_color_fs;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 proj_matrix;


void main() {
    // vec4 vert_coords = vec4(vert_coords_vs.x / scale_factor, vert_coords_vs.y / scale_factor, vert_coords_vs.z / scale_factor, 1.0f);
    vec4 vert_coords = vec4(vert_coords_vs.x, vert_coords_vs.y, vert_coords_vs.z, 1.0f);

    mat4 view_model = view_matrix * model_matrix;

    vert_coords_fs = vec3(view_matrix * model_matrix * vert_coords);

    gl_Position = proj_matrix * view_model * vert_coords;
    vert_color_fs = vert_color_vs;

    gl_PointSize = 1;
}
