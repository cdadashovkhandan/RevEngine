// Adapted from the Advanced Computer Graphics Assignment Framework
#include "Viewport.h"

#include <QOpenGLVersionFunctionsFactory>
#include <QMouseEvent>
#include <QQuaternion>
// #include <MeshManager.h>

#include "Camera.h"
#include "Scene.h"
#include "renderers/Renderer.h"

#include <renderers/ModelRenderer.h>

Viewport::Viewport(QWidget* parent) : QOpenGLWidget(parent)
{
    scene = new Scene();
}

Viewport::~Viewport()
{
    delete renderer;
}

/**
 * @brief Viewport::showModel Display a given model in the viewport by feeding it into the buffers.
 * @param mesh The model to be displayed.
 */
void Viewport::showModel(Model *model)
{
    renderer->update_uniforms();
    renderer->update_buffers(model);
    update();
    scene->model = model;
}

/**
 * @brief Viewport::resetCamera Reset camera position and zoom.
 */
void Viewport::resetCamera()
{
    scene->camera->position = {3, 3, 3};
    scene->camera->fov = { 60.0f };
    renderer->update_uniforms();
    update();
}

//----OpenGL methods----

void Viewport::initializeGL()
{
    QOpenGLWidget::initializeGL();
    gl = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_4_1_Core>(this->context());

    renderer = new ModelRenderer(gl, scene);
    renderer->init(gl, settings);
}

void Viewport::resizeGL(const int w, const int h)
{
    QOpenGLWidget::resizeGL(w, h);
    scene->camera->aspect_ratio = (float)w / (float)h;

    renderer->update_uniforms();
    update();
}

void Viewport::paintGL()
{
    if (scene->model != nullptr)
    {
        QOpenGLWidget::paintGL();
        renderer->render();
    }
}

//----Mouse input event handlers----

void Viewport::mouseReleaseEvent(QMouseEvent* event)
{
    mouse_buttons = event->buttons();
    mouse_position = event->pos();
}

void Viewport::mousePressEvent(QMouseEvent* event)
{
    mouse_buttons = event->buttons();
    mouse_position = event->pos();
}

void Viewport::mouseMoveEvent(QMouseEvent* event)
{
    QOpenGLWidget::mouseMoveEvent(event);

    QPoint mouse_delta = event->pos() - mouse_position;

    // LMB: Rotation
    if ((mouse_buttons & Qt::LeftButton) && (event->buttons() & Qt::LeftButton))
    {
        float angle_hor = (float)-mouse_delta.x() / 2;
        float angle_ver = (float)-mouse_delta.y() / 2;

        QQuaternion rotation_hor = QQuaternion::fromAxisAndAngle(QVector3D{ 0, 1, 0 }, angle_hor);
        QVector3D relative_old = scene->camera->position - scene->camera->target;
        QVector3D relative_hor = rotation_hor.rotatedVector(relative_old);

        float angle_to_up = -qRadiansToDegrees(qAcos(QVector3D::dotProduct(relative_hor.normalized(), QVector3D{ 0, 1, 0 })));
        float angle_to_down = qRadiansToDegrees(qAcos(QVector3D::dotProduct(relative_hor.normalized(), QVector3D{ 0, -1, 0 })));
        angle_ver = qBound(angle_to_up + 1, angle_ver, angle_to_down - 1);

        QVector3D axis_ver = QVector3D::crossProduct(-relative_hor, scene->camera->up).normalized();
        QQuaternion rotation_ver = QQuaternion::fromAxisAndAngle(axis_ver, angle_ver);
        QVector3D relative_final = rotation_ver.rotatedVector(relative_hor);

        scene->camera->position = scene->camera->target + relative_final;
        renderer->update_uniforms();
        update();
    }

    mouse_buttons = event->buttons();
    mouse_position = event->pos();
}

void Viewport::wheelEvent(QWheelEvent* event)
{
    // Scroll wheel: Zoom + Dolly
    float pDelta = (float)event->pixelDelta().y();
    float aDelta = (float)event->angleDelta().y();

    float delta = pDelta == 0 ? aDelta : pDelta;
    float factor = qPow(1.1f, -delta / 120.0f);

    QVector3D relative = scene->camera->position - scene->camera->target;

    float length = relative.length();
    float new_length = factor * length;

    QVector3D new_relative = relative.normalized() * new_length;
    scene->camera->position = scene->camera->target + new_relative;
    scene->camera->fov = scene->camera->fov * factor;

    renderer->update_uniforms();
    update();
}
