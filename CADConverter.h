#ifndef CADCONVERTER_H
#define CADCONVERTER_H

#include "HoughTransformer.h"
#include "data/Model.h"

#include <QMatrix4x4>
class CADConverter
{
public:
    CADConverter();
    HoughTransformer* houghTransformer;
    friend class ModelManager; //TODO maybe not necessary
    Model* convertModel(Model& model) const;

    std::vector<QPair<float, Eigen::Vector3f>> getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr const cloudPtr) const;
private:
    /* TODO:
     * Prim families
     * discretized region T
     *
     */
    QVector<QVector3D>* transform(QVector<QVector3D>& points, QMatrix4x4 const tMatrix) const;

    float maxDistance = 0.15f; // TODO: put this into Settings and make it adjustable
    // QVector<QVector3D> getNeighbors(const QVector3D target, const QVector<QVector3D> points) const;
    Eigen::Matrix4f buildRotationMatrix(Eigen::Vector3f const target, Eigen::Vector3f const source) const;
    void alignCloudWithZAxis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, std::vector<Eigen::Vector3f> normals) const;
    float calculateMFE(std::vector<pcl::PointXYZ> const points, std::vector<float> const distances) const;
};

#endif // CADCONVERTER_H
