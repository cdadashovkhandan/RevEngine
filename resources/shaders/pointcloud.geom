#version 410
// geometry shader


layout (points) in;
layout (line_strip, max_vertices = 6) out;

in VS_OUT {
    vec3 normal;
} gs_in[];

const float MAGNITUDE = 0.4;

uniform mat4 proj_matrix;
//uniform bool render_normals;

void GenerateLine(int index)
{
    gl_Position = proj_matrix * gl_in[index].gl_Position;
    EmitVertex();
    gl_Position = proj_matrix * (gl_in[index].gl_Position +
                                vec4(gs_in[index].normal, 0.0) * MAGNITUDE);
    EmitVertex();
    EndPrimitive();
}

void main()
{
//    if (render_normals)
//    {
//        GenerateLine(0);
//    }
    gl_Position = gl_in[0].gl_Position;
    EmitVertex();
    EndPrimitive();
}
