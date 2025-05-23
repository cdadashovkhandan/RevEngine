
#ifndef MATERIAL_H
#define MATERIAL_H

#include <QMatrix4x4>
#include <QOpenGLFunctions_4_1_Core>
#include <QOpenGLShaderProgram>
#include <GL/gl.h>

class Material
{
public:
    explicit Material(QOpenGLFunctions_4_1_Core* gl);
    virtual ~Material()
    {
        delete shader;
    }

    virtual void update_uniforms(QMatrix4x4 model_mat, QMatrix4x4 view_mat, QMatrix4x4 proj_mat, QMatrix3x3 normal_mat) = 0;
    virtual void bind();
    virtual void release();

protected:
    QOpenGLFunctions_4_1_Core* gl;
    QOpenGLShaderProgram* shader;
};

#endif //MATERIAL_H
