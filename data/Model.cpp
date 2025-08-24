#include "Model.h"

Model::Model(PointCloud::Ptr pointCloud) {
    this->pointCloud = pointCloud;
}

Model::~Model()
{
    normals->clear();
    delete normals;

    clusterIndices->clear();
    delete clusterIndices;

    qDeleteAll(*shapes);
    shapes->clear();
    delete shapes;
}
