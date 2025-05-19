#ifndef MODEL_H
#define MODEL_H

#include "Mesh.h"
#include "PointCloud.h"
// Core data class for both point clouds AND Breps/meshes of specific objects
class Model
{

private:
    PointCloud* pointCloud; // Model cannot exist without pcloud.
    Mesh* mesh = nullptr; // Model can exist without mesh.
public:
    Model();
};

#endif // MODEL_H
