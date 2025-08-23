#ifndef RENDERSHAPE_H
#define RENDERSHAPE_H

#include <Eigen/Dense>
#include <GL/gl.h>

#include <vector>
struct RenderShape
{
    GLuint vao;

    GLuint vbo;
    GLuint vbo_colors;

    GLuint ibo;

    std::vector<uint32_t> indices;
    std::vector<Eigen::Vector3f> vertices;
};

#endif // RENDERSHAPE_H
