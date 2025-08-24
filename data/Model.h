#ifndef MODEL_H
#define MODEL_H

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QPair>
#include "primitives/PrimitiveShape.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

// Core data class for both point clouds AND recognized shapes from said point clouds.
class Model
{
public:
    Model(PointCloud::Ptr pointCloud);
    ~Model();

    float scaleFactor = 1.0f;
    PointCloud::Ptr pointCloud; // Model cannot exist without pcloud.
    PointCloud::Ptr pointCloudDownsampled = nullptr; //TODO: replace with indices?

    //TODO: these shouldn't be pointers.
    std::vector<PrimitiveShape*>* shapes = nullptr; // Model can exist without shapes.
    std::vector<pcl::PointIndices::Ptr>* clusterIndices = nullptr;
    std::vector<Eigen::Vector3f>* normals = nullptr;
};

#endif // MODEL_H
