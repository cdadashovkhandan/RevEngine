#include "CADConverter.h"


CADConverter::CADConverter()
{
    houghTransformer = new HoughTransformer();
}

/**
 * @brief CADConverter::getCentroid Return the center point of the point cloud, AKA the average.
 * @param points
 * @return
 */
QVector3D CADConverter::getCentroid(QVector<QVector3D> &points) const
{
    QVector3D centroid(0,0,0);

    for (QVector3D point : points)
    {
        centroid += point;
    }

    return centroid / float(points.size());
}

/**
 * @brief CADConverter::transform Apply transformation to entire point cloud.
 * @param points
 * @param tMatrix The transformation matrix.
 * @return
 */
QVector<QVector3D>* CADConverter::transform(QVector<QVector3D>& points, QMatrix4x4 tMatrix) const
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

    QVector3D centroid = getCentroid(pCloud->points);

    // build transformation matrix to translate the entire damn thing (basically subtract centroid from every point)
    QMatrix4x4 tMatrix(
        1.0f, 0.0f, 0.0f, -centroid.x(),
        0.0f, 1.0f, 0.0f, -centroid.y(),
        0.0f, 0.0f, 1.0f, -centroid.z(),
        0.0f, 0.0f, 0.0f, 1.0f
        );

    transform(pCloud->points, tMatrix);
    // maybe create basic transform-rotate-whatever helper functions to make things easier?

    qDebug("Calculating normals...");

    // Find normals of points, vote for major normal direction

    // Align major normal direction with z-axis

    //II. Recognition

    //Initialize hough space

    // segment and express space as discrete matrix.



    //III. Postprocessing

    return &model;
}
