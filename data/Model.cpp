#include "Model.h"

Model::Model(PointCloud::Ptr pointCloud) {
    this->pointCloud = pointCloud;
}

Model::~Model()
{
    if (normals != nullptr)
    {
        normals->clear();
        delete normals;
    }

    if (clusterIndices != nullptr)
    {
        clusterIndices->clear();
        delete clusterIndices;
    }

    if (shapes != nullptr)
    {
        qDeleteAll(*shapes);
        shapes->clear();
        delete shapes;
    }
}
