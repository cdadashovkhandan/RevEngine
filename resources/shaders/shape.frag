#version 410
layout(location = 1) in vec3 vert_color_fs;
out vec4 FragColor;

void main()
{
    FragColor = vec4(vert_color_fs, 1.0f);
}
