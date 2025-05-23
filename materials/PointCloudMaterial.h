#ifndef POINTCLOUDMATERIAL_H
#define POINTCLOUDMATERIAL_H

#include "Material.h"

class PointCloudMaterial : public Material
{
public:
    explicit PointCloudMaterial(QOpenGLFunctions_4_1_Core* gl);

    void update_uniforms(QMatrix4x4 model_mat, QMatrix4x4 view_mat, QMatrix4x4 proj_mat, QMatrix3x3 normal_mat) override;
};

#endif // POINTCLOUDMATERIAL_H
