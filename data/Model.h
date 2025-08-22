#ifndef MODEL_H
#define MODEL_H

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QPair>
#include "primitives/PrimitiveShape.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

// Core data class for both point clouds AND Breps/meshes of specific objects
class Model
{

public:
    float scaleFactor = 1.0f;
    PointCloud::Ptr pointCloud; // Model cannot exist without pcloud.
    PointCloud::Ptr pointCloudDownsampled = nullptr; //TODO: replace with indices?
    std::vector<PrimitiveShape*>* shapes = nullptr; // Model can exist without shapes.
    std::vector<pcl::PointIndices::Ptr>* clusterIndices = nullptr;
    std::vector<Eigen::Vector3f>* normals = nullptr;
    Model(PointCloud::Ptr pointCloud);
};

#endif // MODEL_H
