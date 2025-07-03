#ifndef MODEL_H
#define MODEL_H

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Mesh.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

// Core data class for both point clouds AND Breps/meshes of specific objects
class Model
{

public:
    PointCloud* pointCloud; // Model cannot exist without pcloud.
    Mesh* mesh = nullptr; // Model can exist without mesh.
    std::vector<pcl::PointIndices>* pointIndices = nullptr;
    Model(PointCloud* pointCloud);
};

#endif // MODEL_H
