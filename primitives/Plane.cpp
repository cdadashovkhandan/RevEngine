#include "Plane.h"
#include <QtMath>
#include <QDebug>
#include <Util.h>
#include <QtMath>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

Plane::Plane() {
    shapeType = PrimitiveType::PLANE;
}

Plane::~Plane() {
    delete boundingBox;
}

/**
 * @brief Plane::buildParameters
 * @param points
 * @param maxMagnitude
 * @return
 */
std::vector<ParamPair> Plane::buildParameters(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> const points, float maxMagnitude) const
{
    // Three params: theta, phi, rho
    // Theta: [0 , 2pi)
    // Phi: [0, pi]
    // Rho: from 0 to the largest magnitude across points increments of rho/50

    std::vector<ParamPair> output;
    qDebug() << "Building Parameters for Hough Transform...";
    qDebug() << "Max Magnitude: " << maxMagnitude;
    for (int theta = 0; theta < 360; ++theta)
    {
        for (int phi = 0; phi <= 180; ++phi)
        {
            for(float rho = 0; rho <= maxMagnitude; rho += maxMagnitude / 50.0f)
            {
                float phi_rad = qDegreesToRadians(phi);
                float theta_rad = qDegreesToRadians(theta);
                std::vector<float> params({theta_rad, phi_rad, rho});
                output.push_back(ParamPair(pcl::PointIndices::Ptr(new pcl::PointIndices), params));
            }
        }
    }

    qDebug() << "Parameters built: " << output.size();
    return output;
}

/**
 * @brief Plane::isIntersecting Check if a given point is intersecting an instance of this shape with given parameters.
 * @param point Point to be intersected,
 * @param params Parameters of the shape.
 * @param maxMagnitude
 * @return
 */
bool Plane::isIntersecting(pcl::PointXYZ const point, std::vector<float> const params, float const maxMagnitude) const
{
    //TODO: maybe use getNormal() and multiply it by point?

    float threshold = maxMagnitude / 200.0f;
    // Parameters
    float tht = params[0]; //theta
    float phi = params[1];
    float rho = params[2];

    float eqn = point.x * qCos(tht) * qSin(phi)
                + point.y * qSin(tht) * qSin(phi)
                + point.z * qCos(phi);


    return qAbs(eqn - rho) < threshold;
}                                 
/**
 * @brief Plane::calculateMFE Calculate Mean Fitting error for a given set of points.
 * @param cloud
 * @return
 */
float Plane::calculateMFE(pcl::PointCloud<pcl::PointXYZ>::Ptr const cloud)
{
    auto points = cloud->points;
    float tht = parameters[0]; //theta
    float phi = parameters[1];
    float rho = parameters[2];

    float a = qCos(tht)*qSin(phi);
    float b = qSin(phi)*qSin(tht);
    float c = qCos(phi);

    std::vector<float> distances(recognizedIndices->indices.size());

    for (int index : recognizedIndices->indices)
    {
        pcl::PointXYZ point = points[index];
        float d = qAbs(a * point.x + b * point.y + c * point.z + rho);
        distances.push_back(d / (qPow(a, 2) + qPow(b, 2) + qPow(c, 2)));
    }

    if (parameters.empty())
        throw "Tried to calculate MFE with unitialized parameters."; //TODO: proper exception handling

    Eigen::Vector3f maxPoint(0,0,0);
    Eigen::Vector3f minPoint(0,0,0);

    // Get min and max values of each axis.
    Util::getMinMax(points, minPoint, maxPoint);

    float base = maxPoint.x() - minPoint.x();
    float diag = qSqrt(qPow(base, 2) + qPow(maxPoint.y() - minPoint.y(), 2));
    float fin = qSqrt(qPow(diag, 2) + qPow(maxPoint.z() - minPoint.z(), 2));
    float meanDistance = std::accumulate(distances.begin(), distances.end(), 0.0f) / float(distances.size());
    mfe = meanDistance / fin;
    return mfe;
}

/**
 * @brief Plane::getRenderShape Get a RenderShape with all the parameters and transformations applied.
 * @return
 */
std::shared_ptr<RenderShape> Plane::getRenderShape()
{
    std::shared_ptr<RenderShape> renderShape(new RenderShape());

    if (vertices.empty())
        generateVertices();

    renderShape->vertices = vertices;

    // Simple indices forming a plane.
    renderShape->indices = { 0, 1, 3, 1, 2, 3 };
    return renderShape;
}

QString Plane::toString() const
{
    QString output("Plane | %1 points | theta: %2, phi: %3, rho: %4");
    return output.arg(QString::number(recognizedIndices->indices.size()),
                    QString::number(parameters[0], 'g', 3),
                    QString::number(parameters[1], 'g', 3),
                    QString::number(parameters[2], 'g', 3));
}

void Plane::generateVertices()
{
    float size = 0.5f;
    // Unit plane projected on xy plane.
    vertices.push_back(Eigen::Vector3f(size, size, 0.0f)); // top right
    vertices.push_back(Eigen::Vector3f(size, -size, 0.0f)); // bottom right
    vertices.push_back(Eigen::Vector3f(-size, -size, 0.0f)); // bottom left
    vertices.push_back(Eigen::Vector3f(-size, size, 0.0f)); // top left

    Eigen::Vector3f normal = getNormal();

    Eigen::Vector3f axis = (Eigen::Vector3f::UnitZ()).cross(normal);

    // Get the axis of rotation, which is the cross product of the target normal and the Z-axis
    float angle = qAcos(Eigen::Vector3f::UnitZ().dot(normal));
    axis.normalize();
    Eigen::Quaternionf quat;

    float lenX = qAbs(boundingBox->max.x() - boundingBox->min.x());
    float lenY = qAbs(boundingBox->max.y() - boundingBox->min.y());

    Eigen::Vector3f scale(lenX, lenY, 1.0f);

    quat = Eigen::AngleAxisf(angle, axis);
    for (Eigen::Vector3f& vert : vertices)
    {
        // Scale to match target plane
        vert = vert.cwiseProduct(scale);

        // Align with target plane
        vert = quat * vert;

        // Move by rho in the direction of the normal.
        vert += parameters[2] * normal;
    }
}

/**
 * @brief Plane::getNormal Get normal vector of the entire plane.
 * @return
 */
Eigen::Vector3f Plane::getNormal() const
{
    float tht = parameters[0]; //theta
    float phi = parameters[1];

    float a = qCos(tht)*qSin(phi);
    float b = qSin(phi)*qSin(tht);
    float c = qCos(phi);

    Eigen::Vector3f normal(a, b, c);
    normal.normalize();

    return normal;
}

/**
 * @brief Plane::getBoundingBox Get an oriented bounding box of the plane
 * @param cloudPtr
 * @return
 */
BoundingBox* Plane::getBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr)
{
    qDebug("Calculating bounding box...");
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloudPtr);
    extract.setIndices(recognizedIndices);
    extract.filter(*filteredCloud);

    Eigen::Vector3f normal = getNormal();

    Eigen::Vector3f axis = (Eigen::Vector3f::UnitZ()).cross(normal);

    float angle = qAcos(Eigen::Vector3f::UnitZ().dot(normal));
    axis.normalize();
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::AngleAxisf rot(angle, axis);

    transform.rotate(rot);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloudPtr, *transformedCloud, transform.inverse());

    // Get min and max points
    Eigen::Vector3f minPoint(0,0,0);
    Eigen::Vector3f maxPoint(0,0,0);

    //TODO: remove once debugged.
    Util::getMinMax(cloudPtr->points, minPoint, maxPoint);
    qDebug() << "OG Min point: (" << minPoint.x() << ", " <<  minPoint.y() << ", " << minPoint.z() << ")";
    qDebug() << "OG Max point: (" << maxPoint.x() << ", " <<  maxPoint.y() << ", " << maxPoint.z() << ")";


    Util::getMinMax(transformedCloud->points, minPoint, maxPoint);

    qDebug("Bounding box calculated.");
    qDebug() << "Min point: (" << minPoint.x() << ", " <<  minPoint.y() << ", " << minPoint.z() << ")";
    qDebug() << "Max point: (" << maxPoint.x() << ", " <<  maxPoint.y() << ", " << maxPoint.z() << ")";
    boundingBox = new BoundingBox(minPoint, maxPoint);

    return boundingBox;
}
