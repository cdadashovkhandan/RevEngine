#include "PrimitiveShape.h"

#include <pcl/filters/extract_indices.h>

#include "Util.h"


/**
 * @brief PrimitiveShape::getBestFit Perform a (series of) Hough Transform(s) to find the best fitting shape for a given set of points.
 * @param cloud The input point cloud
 * @param indices The indices indiating the cluster in question
 * @return If the shape was successfully found.
 */
bool PrimitiveShape::getBestFit(pcl::PointCloud<pcl::PointXYZ>::Ptr const cloud, pcl::PointIndices::Ptr const indices)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.filter(*filteredCloud);

    float maxMagnitude = std::max_element(filteredCloud->points.begin(),
                                          filteredCloud->points.end(),
                                          [](pcl::PointXYZ const a, pcl::PointXYZ const b)
                                          {
                                              return a.getVector3fMap().norm() < b.getVector3fMap().norm();
                                          })->getVector3fMap().norm();

    std::vector<ParamPair> params = buildParameters(filteredCloud->points, maxMagnitude);

    // Value for best fit chosen by having max votes
    ParamPair* bestFit = nullptr;

    for (int const index : indices->indices)
    {
        pcl::PointXYZ const point = cloud->points[index];

#pragma omp parallel for
        for (ParamPair& pair : params)
        {
            if (isIntersecting(point, pair.second, maxMagnitude))
            {
                pair.first->indices.push_back(index);

                // Check if this is the best fit.
#pragma omp critical
                if (bestFit == nullptr || pair.first->indices.size() > bestFit->first->indices.size())
                {
                    bestFit = &pair;
                }
            }
        }
    }

    // If no best fit was found
    if (bestFit == nullptr)
    {
        qDebug("Best Hough Transform fit not found");
        return false;
    }

    recognizedIndices = bestFit->first;
    parameters = bestFit->second;
    return true;
}




