#ifndef WORLDMATERIAL_H
#define WORLDMATERIAL_H

#include "Material.h"

class WorldMaterial : public Material
{
public:
    explicit WorldMaterial(QOpenGLFunctions_4_1_Core* gl);

    void update_uniforms(QMatrix4x4 model_mat, QMatrix4x4 view_mat, QMatrix4x4 proj_mat, QMatrix3x3 normal_mat) override;
};

#endif // WORLDMATERIAL_H
