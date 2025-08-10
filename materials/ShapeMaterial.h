#ifndef SHAPEMATERIAL_H
#define SHAPEMATERIAL_H

#include "Material.h"

class ShapeMaterial : public Material
{
public:
    explicit ShapeMaterial(QOpenGLFunctions_4_1_Core* gl);

    void update_uniforms(QMatrix4x4 model_mat, QMatrix4x4 view_mat, QMatrix4x4 proj_mat, QMatrix3x3 normal_mat) override;
};

#endif // SHAPEMATERIAL_H
