#ifndef CIRCLE_H
#define CIRCLE_H

#include "PrimitiveShape.h"

class Circle : public PrimitiveShape
{
public:
    Circle();

    // PrimitiveShape interface
public:
    float calculateMFE(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) override;
    std::vector<ParamPair> buildParameters(const std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points, const float maxMagnitude) const override;
    bool isIntersecting(const pcl::PointXYZ point, const std::vector<float> params, const float maxMagnitude) const override;
    std::shared_ptr<RenderShape> getRenderShape() const override;

protected:
    std::vector<Eigen::Vector3f> getBaseVertices() const override;
};

#endif // CIRCLE_H
