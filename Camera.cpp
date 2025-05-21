// Adapted from the Advanced Computer Graphics Assignment Framework
#include "Camera.h"

#include <QMatrix4x4>

/**
 * @brief Camera::viewMatrix Calculate the view matrix of the camera.
 * @return
 */
QMatrix4x4 Camera::viewMatrix() const
{
    QVector3D z = (position - target).normalized();
    QVector3D y = up.normalized();
    QVector3D x = QVector3D::crossProduct(y, z).normalized();
    y = QVector3D::crossProduct(z, x).normalized();

    QMatrix4x4 view{
        x.x(), x.y(), x.z(), -QVector3D::dotProduct(x, position),
        y.x(), y.y(), y.z(), -QVector3D::dotProduct(y, position),
        z.x(), z.y(), z.z(), -QVector3D::dotProduct(z, position),
        0, 0, 0, 1,
    };

    return view;
}

/**
 * @brief Camera::projectionMatrix Calculate the projection matrix of the camera.
 * @return
 */
QMatrix4x4 Camera::projectionMatrix() const
{
    const float DEG2RAD = acosf(-1.0f) / 180.0f;

    const float tangent = tanf(fov / 2 * DEG2RAD);  // tangent of half fovX
    const float right = near * tangent;             // half width of near plane
    const float top = right / aspect_ratio;         // half height of near plane

    QMatrix4x4 projectionMat;
    if (projection == Projection::Perspective)
    {
        projectionMat(0, 0) = near / right;
        projectionMat(1, 1) = near / top;
        projectionMat(2, 2) = -(far + near) / (far - near);
        projectionMat(2, 3) = -(2 * far * near) / (far - near);
        projectionMat(3, 2) = -1;
        projectionMat(3, 3) = 0;
    }
    else // Projection::Orthographic
    {
        projectionMat(0, 0) = 1 / right;
        projectionMat(1, 1) = 1 / top;
        projectionMat(2, 2) = -2 / (far - near);
        projectionMat(2, 3) = -(far + near) / (far - near);
    }
    return projectionMat;
}
