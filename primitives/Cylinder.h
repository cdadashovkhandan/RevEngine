// #ifndef CYLINDER_H
// #define CYLINDER_H

// #include "PrimitiveShape.h"

// class Cylinder : public PrimitiveShape
// {

// public:
//     explicit Cylinder();
//     ~Cylinder() override;
//     std::vector<ParamPair> buildParameters(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> const points, float const maxMagnitude) const override;
//     bool isIntersecting(pcl::PointXYZ const point, std::vector<float> const params, float const maxMagnitude) const override;
//     float calculateMFE(pcl::PointCloud<pcl::PointXYZ>::Ptr const cloud) override;

//     std::shared_ptr<RenderShape> getRenderShape() const override;
// protected:
//     std::vector<float> getDistancesToPoints() const;
//     void getBaseVertices() const override;
// private:
//     Eigen::Vector3f getNormal() const;
// };

// #endif // CYLINDER_H
