#version 410
// Vertex shader

layout(location = 0) in vec3 vert_coords_vs;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 proj_matrix;
uniform mat3 normal_matrix;

void main() {
    vec4 vert_coords = vec4(vert_coords_vs.x, vert_coords_vs.y, vert_coords_vs.z, 1.0f);

    mat4 view_model = view_matrix * model_matrix;

    gl_Position = proj_matrix * view_model * vert_coords;
    gl_PointSize = 20;
}
