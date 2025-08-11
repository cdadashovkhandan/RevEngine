#ifndef RENDERSHAPE_H
#define RENDERSHAPE_H

#include <Eigen/src/Core/Matrix.h>
#include <GL/gl.h>

#include <vector>
struct RenderShape
{
    GLuint vao;

    GLuint vbo;
    GLuint ibo;

    std::vector<int32_t> indices;
    std::vector<Eigen::Vector3f> vertices;
};

#endif // RENDERSHAPE_H
