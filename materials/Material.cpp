
#include "Material.h"

Material::Material(QOpenGLFunctions_4_1_Core* gl) : gl(gl)
{
}

void Material::bind() {
    shader->bind();
}

void Material::release() {
    shader->release();
}
