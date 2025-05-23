// Adapted from the Advanced Computer Graphics Assignment Framework

#ifndef VIEWPORT_H
#define VIEWPORT_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_1_Core>

#include <data/Model.h>

#include "Settings.h"
#include "data/Mesh.h"

class Renderer;
struct Scene;

class Viewport : public QOpenGLWidget
{
    QOpenGLFunctions_4_1_Core* gl { nullptr };
    Renderer* renderer { nullptr };
    Scene* scene { nullptr };

    Qt::MouseButtons mouse_buttons { Qt::NoButton };
    QPoint mouse_position;

public:
    explicit Viewport(QWidget* parent);
    ~Viewport() override;

    void showModel(Model *model);
    void resetCamera();

    Settings* settings{ nullptr };

  protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mouseReleaseEvent(QMouseEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
};

#endif // VIEWPORT_H
