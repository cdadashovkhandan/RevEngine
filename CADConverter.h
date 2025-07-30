#ifndef CADCONVERTER_H
#define CADCONVERTER_H

#include "HoughTransformer.h"
#include "data/Model.h"

#include <QMatrix4x4>
#include <Settings.h>
class CADConverter
{    
public:
    CADConverter(Settings* s);
    HoughTransformer* houghTransformer;
    friend class ModelManager; //TODO maybe not necessary
    Model* convertModel(Model& model) const;

    std::vector<Eigen::Vector3f>*  getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr const cloudPtr) const;

    void downsample(PointCloud::Ptr input, PointCloud::Ptr target) const;
    std::vector<pcl::PointIndices>* cluster(PointCloud::Ptr input) const;
    void shrink(PointCloud::Ptr cloud) const;
private:
    /* TODO:
     * Prim families
     * discretized region T
     *
     */
    QVector<QVector3D>* transform(QVector<QVector3D>& points, QMatrix4x4 const tMatrix) const;

    Settings* settings;
    float maxDistance = 0.15f; // TODO: put this into Settings and make it adjustable
    // QVector<QVector3D> getNeighbors(const QVector3D target, const QVector<QVector3D> points) const;
    Eigen::Matrix4f buildRotationMatrix(Eigen::Vector3f const target, Eigen::Vector3f const source) const;
    void alignCloudWithZAxis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, std::vector<Eigen::Vector3f> normals) const;
    float calculateMFE(std::vector<pcl::PointXYZ> const points, std::vector<float> const distances) const;

    template <typename Allocator>
    void getMinMax(const std::vector<pcl::PointXYZ, Allocator> points, Eigen::Vector3f &minPoint, Eigen::Vector3f &maxPoint) const;
};

#endif // CADCONVERTER_H
