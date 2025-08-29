#ifndef PLANE_H
#define PLANE_H

#include "PrimitiveShape.h"

class Plane : public PrimitiveShape
{

public:
    explicit Plane();
    ~Plane() override;

    std::vector<ParamPair> buildParameters(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> const points,
                                           float const maxMagnitude) const override;

    bool isIntersecting(pcl::PointXYZ const point, std::vector<float> const params, float const maxMagnitude) const override;

    float calculateMFE(pcl::PointCloud<pcl::PointXYZ>::Ptr const cloud) override;

    std::shared_ptr<RenderShape> getRenderShape() override;

    QString toString() const override;

    BoundingBox* getBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr) override;
protected:
    void generateVertices() override;
private:
    Eigen::Vector3f getNormal() const;
};

#endif // PLANE_H
