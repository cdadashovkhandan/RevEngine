#ifndef PRIMITIVESHAPE_H
#define PRIMITIVESHAPE_H

#include "primitives/PrimitiveType.h"
#include <pcl/point_cloud.h>
#include <vector>
#include <QVector3D>
#include <QPair>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include "primitives/RenderShape.h"
using ParamPair = QPair<pcl::PointIndices::Ptr, std::vector<float>>;

class PrimitiveShape
{
public:
    virtual ~PrimitiveShape() = default;
    PrimitiveType  shapeType;

    std::vector<float> parameters = {};
    float mfe = -1.0f;
    QVector3D position;
    QVector3D orientation;

    pcl::PointIndices::Ptr pointIndices; //Indices of the points this shape overlaps with. TODO: better variable name

    virtual float calculateMFE(pcl::PointCloud<pcl::PointXYZ>::Ptr const cloud) = 0;
    std::vector<float> getBestFit(pcl::PointCloud<pcl::PointXYZ>::Ptr const cloud, pcl::PointIndices::Ptr const indices);
    virtual std::vector<ParamPair> buildParameters(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> const points, float const maxMagnitude) const = 0;
    virtual bool isIntersecting(pcl::PointXYZ const point, std::vector<float> const params, float const maxMagnitude) const = 0;

    virtual std::shared_ptr<RenderShape> getRenderShape() const = 0;
protected:
    virtual std::vector<Eigen::Vector3f> getBaseVertices() const = 0;
};

#endif // PRIMITIVESHAPE_H
