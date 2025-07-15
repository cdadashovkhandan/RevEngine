#version 410
// Vertex shader

layout(location = 0) in vec4 vert_coords_vs;
layout(location = 1) in vec3 vert_color_vs;
layout(location = 2) in vec4 vert_normal_vs;

out VS_OUT {
    vec3 normal;
} vs_out;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 proj_matrix;
uniform mat3 normal_matrix;

void main() {
    // vec4 vert_coords = vec4(vert_coords_vs.x / scale_factor, vert_coords_vs.y / scale_factor, vert_coords_vs.z / scale_factor, 1.0f);
    vec4 vert_coords = vec4(vert_coords_vs.x, vert_coords_vs.y, vert_coords_vs.z, 1.0f);

    mat4 view_model = view_matrix * model_matrix;

    gl_Position = proj_matrix * view_model * vert_coords;
    vs_out.normal = normalize(normal_matrix * normalize(vert_normal_vs.xyz));

}
