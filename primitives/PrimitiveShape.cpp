#include "PrimitiveShape.h"
#include "Util.h"

#include <pcl/filters/extract_indices.h>

std::vector<float> PrimitiveShape::getBestFit(pcl::PointCloud<pcl::PointXYZ>::Ptr const cloud, pcl::PointIndices::Ptr const indices)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.filter(*filteredCloud);
    // std::vector<pcl::PointXYZ> points = cloud->points

    //TODO: pick a better place for this
    float maxMagnitude = std::max_element(filteredCloud->points.begin(),
                                          filteredCloud->points.end(),
                                          [](pcl::PointXYZ const a, pcl::PointXYZ const b)
                                          {
                                              return a.getVector3fMap().norm() < b.getVector3fMap().norm();
                                          })->getVector3fMap().norm();

    std::vector<ParamPair> params = buildParameters(filteredCloud->points, maxMagnitude);

    // Value for best fit chosen by having max votes
    // TODO: this is naive.
    ParamPair* bestFit = nullptr;

    // for (pcl::PointXYZ const point : filteredCloud->points)
    for (int const index : indices->indices)
    {
        pcl::PointXYZ const point = cloud->points[index];
        for (ParamPair& pair : params)
        {
            if (isIntersecting(point, pair.second, maxMagnitude))
            {
                pair.first->indices.push_back(index);

                // Check if this is the best fit.
                    if (bestFit == nullptr || pair.first->indices.size() > bestFit->first->indices.size())
                    bestFit = &pair;
            }
        }
    }

    // If no best fit was found
    if (bestFit == nullptr)
    {
        qDebug("Best Hough Transform fit not found");
        parameters = std::vector<float>({0,0,0});
        return parameters;
        //throw; //TODO: proper error handling
    }

    pointIndices = bestFit->first;
    parameters = bestFit->second;
    return parameters;
}

/**
 * @brief CADConverter::calculateMFE Calculate Mean Fitting error for a given set of points and their distances from estimates.
 * @param points
 * @param distances
 * @return
 */
float PrimitiveShape::calculateMFE(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> const points, std::vector<float> const distances) const
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

    Eigen::Vector3f maxPoint(0,0,0);
    Eigen::Vector3f minPoint(0,0,0);
    // Get min and max values of each axis
    Util::getMinMax(points, minPoint, maxPoint);

    float base = maxPoint.x() - minPoint.x();
    float diag = qSqrt(qPow(base, 2) + qPow(maxPoint.y() - minPoint.y(), 2));
    float fin = qSqrt(qPow(diag, 2) + qPow(maxPoint.z() - minPoint.z(), 2));
    float meanDistance = std::accumulate(distances.begin(), distances.end(), 0.0f) / float(distances.size());
    return meanDistance / fin;
}
