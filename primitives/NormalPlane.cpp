#include "NormalPlane.h"
#include <QtMath>

NormalPlane::NormalPlane() {}

QVector<ParamPair> NormalPlane::buildParameters()
{
    // Three params: theta, phi, rho
    // Theta: [0 , 2pi)
    // Phi: [0, pi]
    // Rho: [0, 1], increments of 0.1 (maybe change?)

    QVector<ParamPair> output;
    for (float theta = 0; theta < 2 * M_PI; ++theta)
    {
        for(float phi = 0; phi <= M_PI; ++phi)
        {
            for(float rho = 0; rho <= 1.0f; rho += 0.1f)
            {
                QVector<float> params({theta, phi, rho});
                ParamPair entry(0, params);
                output.append(entry);
            }
        }
    }
    return output;
}

bool NormalPlane::isIntersecting(QVector3D point, QVector<float> params) const
{
    // Parameters
    float tht = params[0]; //theta
    float phi = params[1];
    float rho = params[2];

    // TODO: might need epsilon value
    return point.x() * qCos(tht) * qSin(phi)
               + point.y() * qSin(tht) * qSin(phi)
               + point.z() * qCos(phi)
               - rho
           == 0;
}

void NormalPlane::getBestFit(QVector<QVector3D> points)
{
    QVector<ParamPair> params = buildParameters();

    // Value for best fit chosen by having max votes
    // TODO: this is naive.
    ParamPair* bestFit = nullptr;

    for (QVector3D point : points)
    {
        for (ParamPair& pair : params)
        {
            if (isIntersecting(point, pair.second))
            {
                ++pair.first;
                // Check if this is the best fit.
                if (bestFit == nullptr || pair.first > bestFit->first)
                {
                    bestFit = &pair;
                }
            }
        }
    }

    // Update position and orientation(?) accordingly


}
