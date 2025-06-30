#include "CADConverter.h"

#include <primitives/NormalPlane.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

CADConverter::CADConverter()
{
    houghTransformer = new HoughTransformer();
}

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtrDownsampled(new pcl::PointCloud<pcl::PointXYZ>());
    //TODO: I might be better off just storing clouds as shared pointers right off the bat
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(pCloud);

    qDebug("Creating downsampled copy...");
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;

    float downSampleFactor = 0.5f;

    voxelGrid.setInputCloud(cloudPtr);
    voxelGrid.setLeafSize(downSampleFactor, downSampleFactor, downSampleFactor);
    voxelGrid.filter(*cloudPtrDownsampled);

    qDebug() << "Downsampling complete. Reduced from " << pCloud->size() << " to " << cloudPtrDownsampled->size() << " points.";

    qDebug("Centering Point Cloud...");

    Eigen::Matrix<float, 4, 1> centroid;

    int result = pcl::compute3DCentroid(*cloudPtrDownsampled, centroid);

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
        qWarning("Centering failed, continuing...");

    std::vector<QPair<float, Eigen::Vector3f>> normals = getNormals(cloudPtrDownsampled);

    //TODO: this is for debugging only!
    std::sort(normals.begin(), normals.end(), [](QPair<float, Eigen::Vector3f> a,
                                                 QPair<float, Eigen::Vector3f> b)
              { return a.first < b.first; });
    //alignCloudWithZAxis(cloudPtr, normals);

    //II. Recognition

    //Initialize hough space

    // segment and express space as discrete matrix.

    //III. Segmentation

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance (0.02); // 2cm

    ec.setMinClusterSize (100);

    ec.setMaxClusterSize (25000);

    ec.setSearchMethod (tree);

    ec.setInputCloud (cloud_filtered);

    ec.extract (cluster_indices);

    return &model;
}

void CADConverter::alignCloudWithZAxis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, std::vector<Eigen::Vector3f> const normals) const
{
    // vote for major normal direction
    qDebug("Aligning with z-axis...");
    Eigen::Vector3f average = std::accumulate(normals.begin(), normals.end(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    average /= float(normals.size());
    // Align major normal direction with z-axis

    Eigen::Vector3f zAxis(0.0f, 0.0f, 1.0f);
    Eigen::Matrix4f rotationMatrix = buildRotationMatrix(zAxis, average.normalized());

    pcl::transformPointCloud(*cloudPtr, *cloudPtr, rotationMatrix);
    qDebug("Alignment complete");
}

Eigen::Matrix4f CADConverter::buildRotationMatrix(Eigen::Vector3f const target, Eigen::Vector3f const source) const
{
    //TODO: proper variable names, this ain't MATLAB
    Eigen::Matrix3f GG{
        {target.dot(source),             -target.cross(source).norm(),   0.0f},
        {target.cross(source).norm(),    target.dot(source),             0.0f},
        {0.0f,                           0.0f,                           1.0f}
    };

    Eigen::Matrix3f FFi;
    FFi << target,
        (source - target.dot(source)*target).normalized(),
        target.cross(source);

    Eigen::Matrix3f rotation = FFi * GG * FFi.inverse();

    //TODO: maybe there's a way to do this better?
    // rebuild in homogeneous coordinates
    Eigen::Matrix4f rotMatrix;
    rotMatrix.setIdentity();
    rotMatrix.block<3,3>(0,0) = rotation;

    return rotMatrix;

}

std::vector<QPair<float, Eigen::Vector3f>> CADConverter::getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr const cloudPtr) const
{
    qDebug("Calculating normals...");
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloudPtr);

    size_t neighborCount = 3; //TODO: delegate to Settings

    std::vector<int> neighborIndeces(neighborCount);
    std::vector<float> neighborDistances(neighborCount);
    std::vector<QPair<float, Eigen::Vector3f>> normals;


    for (pcl::PointXYZ const point : cloudPtr->points)
    {
        if (kdtree.nearestKSearch(point, neighborCount, neighborIndeces, neighborDistances) > 0)
        {
            std::vector<pcl::PointXYZ> neighbors(neighborCount);
            // QVector<QVector3D> points(neighborCount);
            // tree.k_nearest_neighbors(node.point, neighborCount, &neighbors, nullptr);

            std::transform(neighborIndeces.begin(),
                           neighborIndeces.end(),
                           neighbors.begin(),
                           [&cloudPtr](int const n) { return (*cloudPtr)[n]; });

            std::vector<float> params = houghTransformer->getBestFit<NormalPlane>(neighbors);

            float tht = params[0]; //theta
            float phi = params[1];
            // float rho = params[2];

            // Build normal vector from chosen parameters
            Eigen::Vector3f normal(qCos(tht)*qSin(phi), qSin(phi)*qSin(tht), qCos(phi));

            std::vector<float> normalDistances(neighborCount);
            std::transform(neighbors.begin(), neighbors.end(), normalDistances.begin(), [normal](pcl::PointXYZ const point){
                return (normal - point.getVector3fMap()).norm();
            });

            float mfe = calculateMFE(neighbors, normalDistances);


            normals.push_back(QPair(mfe, normal.normalized()));
        }
    }

    qDebug("Normals calculated successfully");
    return normals;
}


float CADConverter::calculateMFE(std::vector<pcl::PointXYZ> const points, std::vector<float> const distances) const
{
    /*
    Matlab equivalent:
        function mfe=MFE(xyz,dist)

        base=max(xyz(:,1))-min(xyz(:,1));
        h=max(xyz(:,2))-min(xyz(:,2));
        diag=sqrt(base^2+h^2);
        h=max(xyz(:,3))-min(xyz(:,3));
        Fin=sqrt(diag^2+h^2);

        mfe=mean(dist)/Fin;

        end
    */

    pcl::PointXYZ maxPoint(0,0,0);
    pcl::PointXYZ minPoint(0,0,0);
    // Get min and max values of each axis
    for (pcl::PointXYZ const point : points)
    {
        maxPoint.x = qMax(maxPoint.x, point.x);
        maxPoint.y = qMax(maxPoint.y, point.y);
        maxPoint.z = qMax(maxPoint.z, point.z);

        minPoint.x = qMin(minPoint.x, point.x);
        minPoint.y = qMin(minPoint.y, point.y);
        minPoint.z = qMin(minPoint.z, point.z);
    }

    float base = maxPoint.x - minPoint.x;
    float diag = qSqrt(qPow(base, 2)+ qPow(maxPoint.y - minPoint.y, 2));
    float fin = qSqrt(qPow(diag, 2)+ qPow(maxPoint.z - minPoint.z, 2));
    float meanDistance = std::accumulate(distances.begin(), distances.end(), 0.0f) / float(distances.size());
    return meanDistance / fin;
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
