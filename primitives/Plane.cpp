#include "Plane.h"
#include <QtMath>
#include <QDebug>
#include <Util.h>

Plane::Plane() {
    shapeType = PrimitiveType::PLANE;
}

Plane::~Plane() {}

std::vector<ParamPair> Plane::buildParameters(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> const points, float maxMagnitude) const
{
    // Three params: theta, phi, rho
    // Theta: [0 , 2pi)
    // Phi: [0, pi]
    // Rho: from 0 to the largest magnitude across points increments of rho/50

    std::vector<ParamPair> output;
    for (float theta = 0; theta < 2 * M_PI; ++theta)
    {
        for(float phi = 0; phi <= M_PI; ++phi)
        {
            for(float rho = 0; rho <= maxMagnitude; rho += maxMagnitude / 50.0f)
            {
                std::vector<float> params({theta, phi, rho});
                output.push_back(ParamPair(pcl::PointIndices::Ptr(new pcl::PointIndices), params));
            }
        }
    }
    return output;
}

bool Plane::isIntersecting(pcl::PointXYZ const point, std::vector<float> const params, float const maxMagnitude) const
{

    float threshold = maxMagnitude / 200.0f;
    // Parameters
    float tht = params[0]; //theta
    float phi = params[1];
    float rho = params[2];

    float eqn = point.x * qCos(tht) * qSin(phi)
                + point.y * qSin(tht) * qSin(phi)
                + point.z * qCos(phi);

    // TODO: might need epsilon value
    // qDebug() << eqn;
    return qAbs(eqn - rho) < threshold;
}

/**
 * @brief Plane::calculateMFE Calculate Mean Fitting error for a given set of points and their distances from estimates.
 * @param points
 * @param distances
 * @return
 */
float Plane::calculateMFE(pcl::PointCloud<pcl::PointXYZ>::Ptr const cloud)
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

    auto points = cloud->points;
    float tht = parameters[0]; //theta
    float phi = parameters[1];
    float rho = parameters[2];

    float a = qCos(tht)*qSin(phi);
    float b = qSin(phi)*qSin(tht);
    float c = qCos(phi);

    std::vector<float> distances(pointIndices->indices.size());

    for (int index : pointIndices->indices)
    {
        pcl::PointXYZ point = points[index];
        float d = qAbs(a * point.x + b * point.y + c * point.z + rho);
        distances.push_back(d / (qPow(a, 2) + qPow(b, 2) + qPow(c, 2)));
    }

    if (parameters.empty())
        throw "Tried to calculate MFE with unitialized parameters."; //TODO: proper exception handling

    Eigen::Vector3f maxPoint(0,0,0);
    Eigen::Vector3f minPoint(0,0,0);
    // Get min and max values of each axis
    Util::getMinMax(points, minPoint, maxPoint);

    float base = maxPoint.x() - minPoint.x();
    float diag = qSqrt(qPow(base, 2) + qPow(maxPoint.y() - minPoint.y(), 2));
    float fin = qSqrt(qPow(diag, 2) + qPow(maxPoint.z() - minPoint.z(), 2));
    float meanDistance = std::accumulate(distances.begin(), distances.end(), 0.0f) / float(distances.size());
    mfe = meanDistance / fin;
    return mfe;
}

std::shared_ptr<RenderShape> Plane::getRenderShape() const
{
    std::shared_ptr<RenderShape> renderShape(new RenderShape());

    auto verts = getBaseVertices();

    Eigen::Vector3f normal = getNormal();


    // Eigen::Vector3f centroid = std::accumulate(verts.begin(), verts.end(), Eigen::Vector3f(0.0f,0.0f,0.0f)) / (float) verts.size();


    Eigen::Vector3f axis = (Eigen::Vector3f::UnitZ()).cross(normal);

    float angle = qAcos(Eigen::Vector3f::UnitZ().dot(normal));
    axis.normalize();
    for (Eigen::Vector3f& vert : verts)
    {
        Eigen::Quaternionf quat;
        quat = Eigen::AngleAxisf(angle, axis);
        vert = quat * vert;
    }


    renderShape->vertices = verts;

    renderShape->indices = { 0, 1, 3, 1, 2, 3 };
    return renderShape;
}

std::vector<Eigen::Vector3f> Plane::getBaseVertices() const
{
    std::vector<Eigen::Vector3f> vertices;

    float scale = 0.5f;
    // Unit plane projected on xy plane.
    vertices.push_back(Eigen::Vector3f(scale, scale, 0.0f)); // top right
    vertices.push_back(Eigen::Vector3f(scale, -scale, 0.0f)); // bottom right
    vertices.push_back(Eigen::Vector3f(-scale, -scale, 0.0f)); // bottom left
    vertices.push_back(Eigen::Vector3f(-scale, scale, 0.0f)); // top left


    return vertices;
}

Eigen::Vector3f Plane::getNormal() const
{
    float tht = parameters[0]; //theta
    float phi = parameters[1];
    float rho = parameters[2];


    float a = qCos(tht)*qSin(phi);
    float b = qSin(phi)*qSin(tht);
    float c = qCos(phi);

    Eigen::Vector3f normal(a, b, c);
    normal.normalize();

    return normal;
}
