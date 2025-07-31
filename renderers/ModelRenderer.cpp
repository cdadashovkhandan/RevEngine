#include "ModelRenderer.h"
#include "Camera.h"
#include "Scene.h"
#include "Settings.h"

#include <QMatrix4x4>

#include <materials/PointCloudMaterial.h>
#include <materials/NormalsMaterial.h>

ModelRenderer::ModelRenderer(QOpenGLFunctions_4_1_Core* gl, Scene* scene)
    : Renderer(gl, scene, nullptr),
    pointCloudMat(new PointCloudMaterial(gl)),
    normalsMat(new NormalsMaterial(gl))

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
    gl->glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

    gl->glGenBuffers(1, &vbo_colors);
    gl->glBindBuffer(GL_ARRAY_BUFFER, vbo_colors);
    gl->glEnableVertexAttribArray(1);
    gl->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    gl->glGenBuffers(1, &nbo);
    gl->glBindBuffer(GL_ARRAY_BUFFER, nbo);
    gl->glEnableVertexAttribArray(2);
    gl->glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

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


    // Point Cloud

    if (settings->showPointCloud)
    {
        std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> points =
            model->pointCloud->points;

        // std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> scaledPoints(points.size());

        // std::transform(points.begin(), points.end(), scaledPoints.begin(), [](pcl::PointXYZ const point)
        //                {return pcl::PointXYZ(point.x / 100.0f, point.y / 100.0f, point.z / 100.0f );});

        gl->glBindVertexArray(vao);

        // Coords
        gl->glBindBuffer(GL_ARRAY_BUFFER, vbo);
        gl->glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(pcl::PointXYZ),
                         points.data(), GL_STATIC_DRAW);

        gl->glBindVertexArray(0);
        render_size = points.size();
    }

    // Normals
    if (model->normals != nullptr)
    {

        // std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> scaledPoints(points.size());

        // std::transform(points.begin(), points.end(), scaledPoints.begin(), [](pcl::PointXYZ const point)
        //                {return pcl::PointXYZ(point.x / 100.0f, point.y / 100.0f, point.z / 100.0f );});

        gl->glBindVertexArray(vao);

        // Normals
        gl->glBindBuffer(GL_ARRAY_BUFFER, nbo);
        gl->glBufferData(GL_ARRAY_BUFFER, model->normals->size() * sizeof(Eigen::Vector3f),
                         model->normals->data(), GL_STATIC_DRAW);

        gl->glBindVertexArray(0);
    }

    // Cluster Colors
    std::vector<float> colors(render_size * 3);

    if (settings->showClusters && model->pointIndices != nullptr)
    {
        std::fill(colors.begin(), colors.end(), 0.2f); // initialize to dark grey
        size_t colorIndex = 0;
        for (pcl::PointIndices const cluster : *model->pointIndices)
        {
            for (auto const index : cluster.indices)
            {
                QColor newColor = clusterColors[colorIndex];
                colors[3 * index] = newColor.red();
                colors[3 * index + 1] = newColor.green();
                colors[3 * index + 2] = newColor.blue();
            }
            colorIndex = (colorIndex + 1) % CLUSTER_COLOR_COUNT;
        }
    }
    else
        std::fill(colors.begin(), colors.end(), 1.0f); // initialize to white

    gl->glBindVertexArray(vao);

    // Colors
    gl->glBindBuffer(GL_ARRAY_BUFFER, vbo_colors);
    gl->glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(float),
                     colors.data(), GL_STATIC_DRAW);

    gl->glBindVertexArray(0);

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
}

/**
 * Renders the mesh to the screen
 */
void ModelRenderer::render()
{
    gl->glClearColor(0, 0, 0, 1.0f);
    gl->glClear(GL_COLOR_BUFFER_BIT);
    gl->glEnable(GL_PROGRAM_POINT_SIZE);

    // gl->glDepthFunc(GL_LEQUAL);
    gl->glPolygonMode(GL_POINT, GL_FILL);
    // gl->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    // gl->glEnable(GL_POLYGON_OFFSET_FILL);
    // gl->glPolygonOffset(1, 1);
    gl->glPolygonOffset(1, 1);
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

    //TODO: this is a bit weird. Scene->model is not used in updateBuffers, so this seems kind of disconnected and flimsy.
    if (settings->showNormals && scene->model->normals != nullptr)
    {
     //   gl->glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        drawMaterial(*normalsMat);
    }
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
        gl->glDrawArrays(GL_POINTS, 0, render_size);
        gl->glBindVertexArray(0);
    }
    material.release();
}

