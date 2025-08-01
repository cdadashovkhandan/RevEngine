#version 410
// geometry shader


layout (points) in;
layout (line_strip, max_vertices = 6) out;

const float MAGNITUDE = 2;

uniform mat4 proj_matrix;
uniform mat3 normal_matrix;

vec4 process(vec3 target)
{
    return proj_matrix * vec4(normal_matrix * target, 1.0) * 0.0005;
}

void main()
{
    vec4 ogPos = gl_in[0].gl_Position;
    gl_Position = ogPos;
    EmitVertex();

    gl_Position = ogPos + process(vec3(1.0, 0.0, 0.0)) ;
    EmitVertex();

    gl_Position = ogPos;
    EmitVertex();

    gl_Position = ogPos + process(vec3(0.0, 1.0, 0.0));
    EmitVertex();

    gl_Position = ogPos;
    EmitVertex();

    gl_Position = ogPos + process(vec3(0.0, 0.0, 1.0));
    EmitVertex();

    EndPrimitive();
}
