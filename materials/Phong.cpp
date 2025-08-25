
#include "Phong.h"

Phong::Phong(QOpenGLFunctions_4_1_Core* gl) : Material(gl)
{
    shader = new QOpenGLShaderProgram();
    shader->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/phong.vert");
    shader->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/phong.frag");
    shader->link();
}

Phong::~Phong()
{
    delete shader;
}

void Phong::update_uniforms(QMatrix4x4 model_mat, QMatrix4x4 view_mat, QMatrix4x4 proj_mat, QMatrix3x3 normal_mat)
{
    bind();
    const GLint model_uni = shader->uniformLocation("model_matrix");
    const GLint view_uni = shader->uniformLocation("view_matrix");
    const GLint proj_uni = shader->uniformLocation("proj_matrix");
    const GLint norm_uni = shader->uniformLocation("normal_matrix");

    const GLint diffuse_col_uni = shader->uniformLocation("mat_diffuse_color");
    const GLint specular_col_uni = shader->uniformLocation("mat_specular_color");
    const GLint ambient_col_uni = shader->uniformLocation("mat_ambient_color");

    const GLint diffuse_coeff_uni = shader->uniformLocation("mat_diffuse_coeff");
    const GLint specular_coeff_uni = shader->uniformLocation("mat_specular_coeff");
    const GLint ambient_coeff_uni = shader->uniformLocation("mat_ambient_coeff");
    const GLint mat_shininess_uni = shader->uniformLocation("mat_shininess");

    gl->glUniformMatrix4fv(model_uni, 1, false, model_mat.data());
    gl->glUniformMatrix4fv(view_uni, 1, false, view_mat.data());
    gl->glUniformMatrix4fv(proj_uni, 1, false, proj_mat.data());
    gl->glUniformMatrix3fv(norm_uni, 1, false, normal_mat.data());

    gl->glUniform3f(diffuse_col_uni, diffuse_color[0], diffuse_color[1], diffuse_color[2]);
    gl->glUniform3f(specular_col_uni, specular_color[0], specular_color[1], specular_color[2]);
    gl->glUniform3f(ambient_col_uni, ambient_color[0], ambient_color[1], ambient_color[2]);

    gl->glUniform1f(diffuse_coeff_uni, diffuse_coefficient);
    gl->glUniform1f(specular_coeff_uni, specular_coefficient);
    gl->glUniform1f(ambient_coeff_uni, ambient_coefficient);
    gl->glUniform1f(mat_shininess_uni, shininess);
    release();
}
