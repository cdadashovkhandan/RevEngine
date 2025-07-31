#ifndef MODEL_H
#define MODEL_H

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QPair>
#include "Mesh.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

// Core data class for both point clouds AND Breps/meshes of specific objects
class Model
{

public:
    PointCloud::Ptr pointCloud; // Model cannot exist without pcloud.
    PointCloud::Ptr pointCloudDownsampled;
    Mesh* mesh = nullptr; // Model can exist without mesh.
    std::vector<pcl::PointIndices>* pointIndices = nullptr;
    std::vector<Eigen::Vector3f>* normals = nullptr;
    Model(PointCloud::Ptr pointCloud);
};

#endif // MODEL_H
