#include "ModelRenderer.h"
#include "Camera.h"
#include "Scene.h"
#include "Settings.h"

#include <QMatrix4x4>

#include <materials/PointCloudMaterial.h>

ModelRenderer::ModelRenderer(QOpenGLFunctions_4_1_Core* gl, Scene* scene)
    : Renderer(gl, scene, nullptr),
    pointCloudMat(new PointCloudMaterial(gl))
{
    gl->glGenVertexArrays(1, &vao);
    gl->glEnable(GL_DEPTH_TEST);
}

ModelRenderer::~ModelRenderer()
{
    delete pointCloudMat;
}

/**
 * @brief ModelRenderer::initBuffers Initialize all the buffers of the renderer.
 */
void ModelRenderer::initBuffers()
{
    gl->glGenVertexArrays(1, &vao);
    gl->glBindVertexArray(vao);

    gl->glGenBuffers(1, &vbo);
    gl->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    gl->glEnableVertexAttribArray(0);
    gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    gl->glGenBuffers(1, &nbo);
    gl->glBindBuffer(GL_ARRAY_BUFFER, nbo);
    gl->glEnableVertexAttribArray(1);
    gl->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    gl->glGenBuffers(1, &ibo);
    gl->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

    gl->glBindVertexArray(0);
}

/**
 * Updates the buffer data with the mesh data provided
 * @param mesh
 */
void ModelRenderer::update_buffers(Model* model)
{
    // QVector<QVector3D>& vertices = mesh->v_coords;
    // QVector<QVector3D>& normals = mesh->v_normals;
    // QVector<uint32_t> indices = mesh->f_verts();



    if (settings->showPointCloud)
    {
        QVector<QVector3D>& points = model->pointCloud->points;
        gl->glBindVertexArray(vao);

        // Coords
        gl->glBindBuffer(GL_ARRAY_BUFFER, vbo);
        gl->glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(QVector3D), points.data(), GL_STATIC_DRAW);

        gl->glBindVertexArray(0);
        render_size = points.size();
    }

    //TODO: I'm not sure if showing both at the same time is a good idea. Think about it further.
    if (settings->showMesh && model->mesh != nullptr)
    {
        // Mesh rendering code
    }

    /*
    gl->glBindVertexArray(vao);

    // Coords
    gl->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    gl->glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(QVector3D), vertices.data(), GL_STATIC_DRAW);

    // Normals
    gl->glBindBuffer(GL_ARRAY_BUFFER, nbo);
    gl->glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(QVector3D), normals.data(), GL_STATIC_DRAW);

    // Indices
    gl->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    gl->glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint32_t), indices.data(), GL_STATIC_DRAW);

    gl->glBindVertexArray(0);

    render_size = indices.size();
    */
}

/**
 * Updates the uniforms for the scene
 */
void ModelRenderer::update_uniforms()
{
    QMatrix4x4 model = QMatrix4x4();
    QMatrix4x4 view = scene->camera->viewMatrix();
    QMatrix4x4 proj = scene->camera->projectionMatrix();
    QMatrix3x3 norm = (view * model).normalMatrix();

    pointCloudMat->update_uniforms(model, view, proj, norm);
    // wireframe_mat->update_uniforms(model, view, proj, norm);
}

/**
 * Renders the mesh to the screen
 */
void ModelRenderer::render()
{
    // gl->glClearColor(0, 0, 0, 1.0f);
    // gl->glClear(GL_COLOR_BUFFER_BIT);
    // gl->glEnable(GL_PROGRAM_POINT_SIZE);

    gl->glDepthFunc(GL_LEQUAL);
    // gl->glPolygonMode(GL_POINT, GL_FILL);
    gl->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    gl->glEnable(GL_POLYGON_OFFSET_FILL);
    gl->glPolygonOffset(1, 1);
    // gl->glPolygonOffset(1, 1);
    drawMaterial(*pointCloudMat);
    // gl->glDisable(GL_POLYGON_OFFSET_FILL);

    // gl->glClearColor(0, 0, 0, 1.0f);
    // gl->glClear(GL_COLOR_BUFFER_BIT);

    // gl->glDepthFunc(GL_LEQUAL);

    // gl->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    // gl->glEnable(GL_POLYGON_OFFSET_FILL);
    // gl->glPolygonOffset(1, 1);
    // drawMaterial(*pointCloudMat);
    // gl->glDisable(GL_POLYGON_OFFSET_FILL);

    // if (settings->showWireframe)
    // {
    //     gl->glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    //     // drawMaterial(*wireframe_mat);
    // }
}

/**
 * Draws the elements to the screen using the provided material
 * @param material
 */
void ModelRenderer::drawMaterial(Material &material)
{
    material.bind();
    {
        gl->glBindVertexArray(vao);
        gl->glDrawArrays(GL_POINTS, render_size, GL_UNSIGNED_INT);
        gl->glBindVertexArray(0);
    }
    material.release();
}

