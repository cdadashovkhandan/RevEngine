#include "NormalPlane.h"
#include <QtMath>
#include <QDebug>

NormalPlane::NormalPlane() {}

QVector<ParamPair> NormalPlane::buildParameters(QVector<QVector3D> const points) const
{
    // Three params: theta, phi, rho
    // Theta: [0 , 2pi)
    // Phi: [0, pi]
    // Rho: from 0 to the largest magnitude across points increments of rho/50

    float maxMagnitude = 0;
    for(QVector3D const point : points)
    {
        maxMagnitude = qMax(point.length(), maxMagnitude);
    }

    QVector<ParamPair> output;
    for (float theta = 0; theta < 2 * M_PI; ++theta)
    {
        for(float phi = 0; phi <= M_PI; ++phi)
        {
            for(float rho = 0; rho <= maxMagnitude; rho += maxMagnitude / 50.0f)
            {
                QVector<float> params({theta, phi, rho});
                ParamPair entry(0, params);
                output.append(entry);
            }
        }
    }
    return output;
}

bool NormalPlane::isIntersecting(QVector3D const point, QVector<float> const params) const
{
    // Parameters
    float tht = params[0]; //theta
    float phi = params[1];
    float rho = params[2];

    float eqn = point.x() * qCos(tht) * qSin(phi)
                + point.y() * qSin(tht) * qSin(phi)
                + point.z() * qCos(phi);

    // TODO: might need epsilon value
    // qDebug() << eqn;
    return qAbs(eqn)< (rho / 2);
}
