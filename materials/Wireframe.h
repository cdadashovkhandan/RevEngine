#ifndef WIREFRAME_H
#define WIREFRAME_H

#include <QOpenGLShaderProgram>

#include "Material.h"

class Wireframe : public Material
{
public:
    explicit Wireframe(QOpenGLFunctions_4_1_Core* gl);
    ~Wireframe() override;

    void update_uniforms(QMatrix4x4 model_mat, QMatrix4x4 view_mat, QMatrix4x4 proj_mat, QMatrix3x3 normal_mat) override;
private:
    QOpenGLShaderProgram* shader;
};


#endif // WIREFRAME_H
