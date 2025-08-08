#include "Plane.h"
#include <QtMath>
#include <QDebug>

Plane::Plane() {
    shapeType = PrimitiveType::PLANE;
}

Plane::~Plane() {}

std::vector<ParamPair> Plane::buildParameters(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> const points, float maxMagnitude) const
{
    // Three params: theta, phi, rho
    // Theta: [0 , 2pi)
    // Phi: [0, pi]
    // Rho: from 0 to the largest magnitude across points increments of rho/50

    std::vector<ParamPair> output;
    for (float theta = 0; theta < 2 * M_PI; ++theta)
    {
        for(float phi = 0; phi <= M_PI; ++phi)
        {
            for(float rho = 0; rho <= maxMagnitude; rho += maxMagnitude / 50.0f)
            {
                std::vector<float> params({theta, phi, rho});
                output.push_back(ParamPair(pcl::PointIndices::Ptr(new pcl::PointIndices), params));
            }
        }
    }
    return output;
}

bool Plane::isIntersecting(pcl::PointXYZ const point, std::vector<float> const params, float const maxMagnitude) const
{

    float threshold = maxMagnitude / 200.0f;
    // Parameters
    float tht = params[0]; //theta
    float phi = params[1];
    float rho = params[2];

    float eqn = point.x * qCos(tht) * qSin(phi)
                + point.y * qSin(tht) * qSin(phi)
                + point.z * qCos(phi);

    // TODO: might need epsilon value
    // qDebug() << eqn;
    return qAbs(eqn - rho) < threshold;
}
