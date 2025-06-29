#ifndef PRIMITIVESHAPE_H
#define PRIMITIVESHAPE_H

#include <QVector3D>
#include <QVector>
#include <QPair>
#include <pcl/point_types.h>
using ParamPair = QPair<unsigned int, std::vector<float>>;

class PrimitiveShape
{
//TODO: Separate hough transform logic from shape instances
public:
    PrimitiveShape();

    QVector3D position;
    QVector3D orientation;

    // TODO: return type probably wrong
    // virtual QVector<float> getBestFit(QVector<QVector3D> const points) const = 0;
    // Build the initial matrix of parameters and flatten them to a 2D vector of floats
    virtual std::vector<ParamPair> buildParameters(std::vector<pcl::PointXYZ> const points, float const maxMagnitude) const = 0;
    virtual bool isIntersecting(pcl::PointXYZ const point, std::vector<float> const params, float const maxMagnitude) const = 0;
};

#endif // PRIMITIVESHAPE_H
