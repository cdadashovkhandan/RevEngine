#ifndef NORMALSMATERIAL_H
#define NORMALSMATERIAL_H

#include "Material.h"

class NormalsMaterial : public Material
{
public:
    explicit NormalsMaterial(QOpenGLFunctions_4_1_Core* gl);

    void update_uniforms(QMatrix4x4 model_mat, QMatrix4x4 view_mat, QMatrix4x4 proj_mat, QMatrix3x3 normal_mat) override;
};

#endif // NORMALSMATERIAL_H
