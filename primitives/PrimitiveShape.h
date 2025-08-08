#ifndef PRIMITIVESHAPE_H
#define PRIMITIVESHAPE_H

#include "primitives/PrimitiveType.h"
#include <pcl/point_cloud.h>
#include <vector>
#include <QVector3D>
#include <QPair>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
using ParamPair = QPair<pcl::PointIndices::Ptr, std::vector<float>>;

class PrimitiveShape
{
//TODO: Separate hough transform logic from shape instances
public:
    virtual ~PrimitiveShape() = default;
    PrimitiveType  shapeType;


    std::vector<float> parameters = {};
    QVector3D position;
    QVector3D orientation;

    pcl::PointIndices::Ptr pointIndices; //Indices of the points this shape overlaps with. TODO: better variable name

    // TODO: return type probably wrong
    std::vector<float> getBestFit(pcl::PointCloud<pcl::PointXYZ>::Ptr const cloud, pcl::PointIndices::Ptr const indices);
    // Build the initial matrix of parameters and flatten them to a 2D vector of floats
    virtual std::vector<ParamPair> buildParameters(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> const points, float const maxMagnitude) const = 0;
    virtual bool isIntersecting(pcl::PointXYZ const point, std::vector<float> const params, float const maxMagnitude) const = 0;
};

#endif // PRIMITIVESHAPE_H
