#include "ShapeMaterial.h"

ShapeMaterial::ShapeMaterial(QOpenGLFunctions_4_1_Core* gl) : Material(gl)
{
    shader = new QOpenGLShaderProgram();
    shader->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/shape.vert");
    // shader->addShaderFromSourceFile(QOpenGLShader::Geometry, ":/shaders/shape.geom");
    shader->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/shape.frag");
    shader->link();
}

void ShapeMaterial::update_uniforms(QMatrix4x4 model_mat, QMatrix4x4 view_mat, QMatrix4x4 proj_mat, QMatrix3x3 normal_mat) {
    bind();
    const GLint model_uni = shader->uniformLocation("model_matrix");
    const GLint view_uni = shader->uniformLocation("view_matrix");
    const GLint proj_uni = shader->uniformLocation("proj_matrix");
    const GLint norm_uni = shader->uniformLocation("normal_matrix");

    gl->glUniformMatrix4fv(model_uni, 1, false, model_mat.data());
    gl->glUniformMatrix4fv(view_uni, 1, false, view_mat.data());
    gl->glUniformMatrix4fv(proj_uni, 1, false, proj_mat.data());
    gl->glUniformMatrix3fv(norm_uni, 1, false, normal_mat.data());
    release();
}
