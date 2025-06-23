#include "CADConverter.h"

#include <primitives/NormalPlane.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

// #include <data/KDTree.h>
CADConverter::CADConverter()
{
    houghTransformer = new HoughTransformer();
}

/**
 * @brief CADConverter::getCentroid Return the center point of the point cloud, AKA the average.
 * @param points
 * @return
 */
// QVector3D CADConverter::getCentroid(QVector<QVector3D> &points) const
// {
//     QVector3D centroid(0,0,0);

//     for (QVector3D point : points)
//     {
//         centroid += point;
//     }

//     return centroid / float(points.size());
// }

/**
 * @brief CADConverter::transform Apply transformation to entire point cloud.
 * @param points
 * @param tMatrix The transformation matrix.
 * @return
 */
QVector<QVector3D>* CADConverter::transform(QVector<QVector3D>& points, QMatrix4x4 const tMatrix) const
{
    for (QVector3D& point : points)
    {
        QVector4D hPoint = QVector4D(point.x(), point.y(), point.z(), 1);
        hPoint = tMatrix * hPoint;
        point = QVector3D(hPoint);
    }
    return &points;
}

/**
 * @brief CADConverter::convertModel Generate a B-rep CAD model from a Model's Point Cloud
 * @param model
 * @return
 */
Model* CADConverter::convertModel(Model& model) const
{
    // Main function for everything:

    // I. Preprocessing

    // 1. Translate the point cloud to align its center with center of coordinate system

    PointCloud* pCloud = model.pointCloud;

    qDebug("Centering Point Cloud...");

    Eigen::Matrix<float, 4, 1> centroid;

    int result = pcl::compute3DCentroid(*pCloud, centroid);

    if (result != 0)
    {
        Eigen::Matrix4f tMatrix = Eigen::Matrix4f::Identity();
        tMatrix(0, 3) = -centroid.x();
        tMatrix(1, 3) = -centroid.y();
        tMatrix(2, 3) = -centroid.z();

        // tMatrix.translation() << centroid;

        pcl::transformPointCloud(*pCloud, *pCloud, tMatrix);
        qDebug("Centering successful");
    }
    else
        qDebug("Centering failed, continuing...");


    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    //TODO: I might be better off just storing clouds as shared pointers right off the bat
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(pCloud);
    kdtree.setInputCloud (cloudPtr);

    //build kd tree out of nodes
    // QVector<KDNode> nodes;
    // for (QVector3D point : pCloud -> points)
    // {
    //     nodes.push_back(KDNode(point));
    // }

    // KDTree cloudTree(&nodes);

    // QVector<QVector3D> normals = getNormals(cloudTree);

    // Find normals of points, vote for major normal direction

    // Align major normal direction with z-axis

    //II. Recognition

    //Initialize hough space

    // segment and express space as discrete matrix.



    //III. Postprocessing

    return &model;
}

QVector<QVector3D> CADConverter::getNormals(KDTree& tree) const
{
    qDebug("Calculating normals...");
    QVector<QVector3D> normals;

    for (KDNode const node : tree.allnodes)
    {
        size_t neighborCount = 3;
        QVector<KDNode> neighbors;
        QVector<QVector3D> points(neighborCount);
        tree.k_nearest_neighbors(node.point, neighborCount, &neighbors, nullptr);

        std::transform(neighbors.begin(), neighbors.end(), points.begin(), [](KDNode n) { return n.point; });
        QVector<float> params = houghTransformer->getBestFit<NormalPlane>(points);

        float tht = params[0]; //theta
        float phi = params[1];
        float rho = params[2];

        // Build normal vector from chosen parameters
        QVector3D normal(qCos(tht)*qSin(phi), qSin(phi)*qSin(tht), qCos(phi));
        normals.append(normal);
    }

    return normals;
}


// QVector<QVector3D> CADConverter::getNeighbors(QVector3D const target, QVector<QVector3D> const points) const
// {
//     QVector<QVector3D> result;
//     result.append(target); // TODO: might be unnecessary
//     for (QVector3D point : points)
//     {
//         float distance = target.distanceToPoint(point);
//         if (distance <= maxDistance && distance > 0)
//         {
//             result.append(point);
//         }
//     }

//     return result;
// }
