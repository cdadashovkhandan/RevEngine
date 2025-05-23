
#ifndef PHONG_H
#define PHONG_H

#include <QOpenGLShaderProgram>

#include "Material.h"

class Phong : public Material
{
public:
    explicit Phong(QOpenGLFunctions_4_1_Core* gl);
    ~Phong() override;

    void update_uniforms(QMatrix4x4 model_mat, QMatrix4x4 view_mat, QMatrix4x4 proj_mat, QMatrix3x3 normal_mat) override;

    QVector3D diffuse_color{ 0.03, 0.80, 0.87 };
    QVector3D specular_color{ 1.0, 1.0, 1.0 };
    QVector3D ambient_color{ 1.0, 1.0, 1.0 };

    float diffuse_coefficient{ 0.6 };
    float specular_coefficient{ 0.5 };
    float ambient_coefficient{ 0.1 };
    float shininess{ 2.0 };

private:
    QOpenGLShaderProgram* shader;
};



#endif //PHONG_H
