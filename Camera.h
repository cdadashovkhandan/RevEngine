// Adapted from the Advanced Computer Graphics Assignment Framework
#ifndef CAMERA_H
#define CAMERA_H

#include <QVector3D>

enum class Projection { Perspective, Orthographic };

struct Camera
{
    float near{ 0.1f };
    float far{ 100.0f };
    float fov{ 60.0f };
    float aspect_ratio{ 1 };

    QVector3D position{ 0, 0, 3 };
    QVector3D target{ 0, 0, 0 };
    QVector3D up{ 0, 1, 0 };

    Projection projection{ Projection::Orthographic };

    [[nodiscard]] QMatrix4x4 viewMatrix() const;
    [[nodiscard]] QMatrix4x4 projectionMatrix() const;
};

#endif //CAMERA_H
