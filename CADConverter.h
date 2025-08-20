#ifndef CADCONVERTER_H
#define CADCONVERTER_H

#include "data/Model.h"
#include "primitives/PrimitiveShape.h"

#include <QMatrix4x4>
#include <Settings.h>

class CADConverter
{    
public:
    CADConverter(Settings* s);

    Model* preprocess(Model& model) const;

    Model *recognizeShapes(Model &model) const;

    std::vector<Eigen::Vector3f>*  getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr const cloudPtr) const;

    void downsample(PointCloud::Ptr input, PointCloud::Ptr target) const;

    std::vector<pcl::PointIndices::Ptr>* cluster(PointCloud::Ptr input, bool const useRansac) const;

    void shrink(PointCloud::Ptr cloud) const;
private:
    Settings* settings;

    Eigen::Matrix4f buildRotationMatrix(Eigen::Vector3f const target, Eigen::Vector3f const source) const;

    void alignCloudWithZAxis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, std::vector<Eigen::Vector3f> normals) const;

    PrimitiveShape* getShape(const PrimitiveType type) const;
    bool isCloudSparse(PointCloud::Ptr cloud) const;
};

#endif // CADCONVERTER_H
