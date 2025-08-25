#include "ModelRenderer.h"
#include "Camera.h"
#include "Scene.h"
#include "Settings.h"

#include <QMatrix4x4>

#include <materials/PointCloudMaterial.h>
#include <materials/NormalsMaterial.h>
#include <materials/WorldMaterial.h>
#include <materials/ShapeMaterial.h>

ModelRenderer::ModelRenderer(QOpenGLFunctions_4_1_Core* gl, Scene* scene)
    : Renderer(gl, scene, nullptr),
    pointCloudMat(new PointCloudMaterial(gl)),
    normalsMat(new NormalsMaterial(gl)),
    worldMat(new WorldMaterial(gl)),
    shapeMat(new ShapeMaterial(gl))
{
    gl->glEnable(GL_DEPTH_TEST);
    gl->glGenVertexArrays(1, &vao);
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
    gl->glBindVertexArray(vao);

    gl->glGenBuffers(1, &vbo);
    gl->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    gl->glEnableVertexAttribArray(0);
    gl->glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

    gl->glGenBuffers(1, &vbo_colors);
    gl->glBindBuffer(GL_ARRAY_BUFFER, vbo_colors);
    gl->glEnableVertexAttribArray(1);
    gl->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    gl->glGenBuffers(1, &nbo);
    gl->glBindBuffer(GL_ARRAY_BUFFER, nbo);
    gl->glEnableVertexAttribArray(2);
    gl->glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    gl->glBindVertexArray(0);
}

/**
 * Updates the buffer data with the model data provided
 * @param model
 */
void ModelRenderer::update_buffers(Model* model)
{
    // Point Cloud
    gl->glBindVertexArray(vao);

    if (settings->showPointCloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr = settings->useDownsampledVersion && model->pointCloudDownsampled != nullptr
            ? model->pointCloudDownsampled
            : model->pointCloud;

        // Coords
        gl->glBindBuffer(GL_ARRAY_BUFFER, vbo);
        gl->glBufferData(GL_ARRAY_BUFFER, cloudPtr->points.size() * sizeof(pcl::PointXYZ),
                         cloudPtr->points.data(), GL_STATIC_DRAW);

        render_size = cloudPtr->points.size();
    }

    // Normals
    if (model->normals != nullptr)
    {
        gl->glBindBuffer(GL_ARRAY_BUFFER, nbo);
        gl->glBufferData(GL_ARRAY_BUFFER, model->normals->size() * sizeof(Eigen::Vector3f),
                         model->normals->data(), GL_STATIC_DRAW);
    }

    // Cluster Colors
    std::vector<float> clusterColors(render_size * 3);

    if (settings->showClusters && model->clusterIndices != nullptr)
    {
        std::fill(clusterColors.begin(), clusterColors.end(), 0.4f); // initialize to dark grey
        size_t colorIndex = 0;
        for (pcl::PointIndices::Ptr const &cluster : *model->clusterIndices)
        {
            for (auto const index : cluster->indices)
            {
                QColor newColor = colors[colorIndex];
                clusterColors[3 * index] = newColor.red();
                clusterColors[3 * index + 1] = newColor.green();
                clusterColors[3 * index + 2] = newColor.blue();
            }
            colorIndex = (colorIndex + 1) % CLUSTER_COLOR_COUNT;
        }
    }
    else
        std::fill(clusterColors.begin(), clusterColors.end(), 1.0f); // initialize to white

    // Colors
    gl->glBindBuffer(GL_ARRAY_BUFFER, vbo_colors);
    gl->glBufferData(GL_ARRAY_BUFFER, clusterColors.size() * sizeof(float),
                     clusterColors.data(), GL_STATIC_DRAW);

    gl->glBindVertexArray(0);

    clearRenderShapes();

    if (settings->showShapes && model->shapes != nullptr)
    {

        size_t colorIndex = 0;
        for (PrimitiveShape* shape : *model->shapes)
        {
            qDebug() << "Rendering shape of type " << shape->shapeType;
            std::shared_ptr<RenderShape> renderShape = shape->getRenderShape();

            gl->glGenVertexArrays(1, &renderShape->vao);


            gl->glBindVertexArray(renderShape->vao);
            gl->glGenBuffers(1, &renderShape->vbo);
            gl->glGenBuffers(1, &renderShape->vbo_colors);
            gl->glGenBuffers(1, &renderShape->ibo);

            gl->glBindBuffer(GL_ARRAY_BUFFER, renderShape->vbo);
            gl->glBufferData(GL_ARRAY_BUFFER, renderShape->vertices.size() * sizeof(Eigen::Vector3f),
                             renderShape->vertices.data(), GL_STATIC_DRAW);

            gl->glEnableVertexAttribArray(0);
            gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

            // Shape Colors
            std::vector<float> shapeColors(renderShape->vertices.size() * 3);

            for (size_t idx = 0; idx < renderShape->vertices.size(); ++idx)
            {
                QColor newColor = colors[colorIndex];
                shapeColors[3 * idx] = newColor.red();
                shapeColors[3 * idx + 1] = newColor.green();
                shapeColors[3 * idx + 2] = newColor.blue();
            }
            colorIndex = (colorIndex + 1) % CLUSTER_COLOR_COUNT;

            gl->glBindBuffer(GL_ARRAY_BUFFER, renderShape->vbo_colors);
            gl->glBufferData(GL_ARRAY_BUFFER, shapeColors.size() * sizeof(float),
                             shapeColors.data(), GL_STATIC_DRAW);

            gl->glEnableVertexAttribArray(1);
            gl->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

            gl->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, renderShape->ibo);
            gl->glBufferData(GL_ELEMENT_ARRAY_BUFFER, renderShape->indices.size() * sizeof(uint32_t), renderShape->indices.data(), GL_STATIC_DRAW);

            gl->glBindVertexArray(0);

            renderShapes.push_back(renderShape);
        }
    }
}

/**
 * @brief ModelRenderer::clearRenderShapes Delete all current render shapes.
 */
void ModelRenderer::clearRenderShapes()
{
    for (const std::shared_ptr<RenderShape> &renderShape : renderShapes) {
        gl->glDeleteVertexArrays(1, &renderShape->vao);
        gl->glDeleteBuffers(1, &renderShape->vbo);
        gl->glDeleteBuffers(1, &renderShape->vbo_colors);
        gl->glDeleteBuffers(1, &renderShape->ibo);
    }

    renderShapes.clear();
}

/**
 * @brief ModelRenderer::update_uniforms Update all the uniforms in the scene.
 */
void ModelRenderer::update_uniforms()
{
    float scaleFactor = 0.01f;
    Eigen::Transform<float, 3, Eigen::Affine> model_raw =
        Eigen::Transform<float, 3, Eigen::Affine>{Eigen::Transform<float, 3, Eigen::Affine>::Identity()}
            .scale(settings->scaleFactor);

    QMatrix4x4 model(model_raw.data());
    QMatrix4x4 view = scene->camera->viewMatrix();
    QMatrix4x4 proj = scene->camera->projectionMatrix();
    QMatrix3x3 norm = (view * model).normalMatrix();

    pointCloudMat->update_uniforms(model, view, proj, norm);
    normalsMat->update_uniforms(model, view, proj, norm);
    worldMat->update_uniforms(model, view, proj, norm);
    shapeMat->update_uniforms(model, view, proj, norm);
}

/**
 * @brief ModelRenderer::render Render the model to the screen.
 */
void ModelRenderer::render()
{
    gl->glClearColor(0, 0, 0, 1.0f);
    gl->glClear(GL_COLOR_BUFFER_BIT);
    gl->glEnable(GL_PROGRAM_POINT_SIZE);

    gl->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    gl->glPolygonOffset(1, 1);

    if (settings->showPointCloud)
        drawMaterial(*pointCloudMat);

    if (settings->showAxisLines)
        drawMaterial(*worldMat);

    if (settings->showNormals && scene->model->normals != nullptr)
    {
        drawMaterial(*normalsMat);
    }

    if (settings->showShapes && renderShapes.size() > 0)
    {
        for (const std::shared_ptr<RenderShape> &shape : renderShapes)
            drawShape(shape);
    }
}

/**
 * @brief ModelRenderer::drawMaterial Draws the elements to the screen using the provided material.
 * @param material
 */
void ModelRenderer::drawMaterial(Material &material)
{
    material.bind();
    {
        gl->glBindVertexArray(vao);

        gl->glBindBuffer(GL_ARRAY_BUFFER, vbo);
        gl->glEnableVertexAttribArray(0);
        gl->glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

        gl->glBindBuffer(GL_ARRAY_BUFFER, vbo_colors);
        gl->glEnableVertexAttribArray(1);
        gl->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        gl->glDrawArrays(GL_POINTS, 0, render_size);
        gl->glBindVertexArray(0);
    }
    material.release();
}

/**
 * @brief ModelRenderer::drawShape Render a given RenderShape.
 * @param renderShape
 */
void ModelRenderer::drawShape(std::shared_ptr<RenderShape> const renderShape)
{
    shapeMat->bind();
    {
        gl->glBindVertexArray(renderShape->vao);

        gl->glBindBuffer(GL_ARRAY_BUFFER, renderShape->vbo);
        gl->glEnableVertexAttribArray(0);
        gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        gl->glBindBuffer(GL_ARRAY_BUFFER, renderShape->vbo_colors);
        gl->glEnableVertexAttribArray(1);
        gl->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        gl->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, renderShape->ibo);

        gl->glDrawElements(GL_TRIANGLES, renderShape->indices.size(), GL_UNSIGNED_INT, nullptr);

        gl->glBindVertexArray(0);
    }
    shapeMat->release();
}

