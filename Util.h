#ifndef UTIL_H
#define UTIL_H

#include <vector>
#include <pcl/impl/point_types.hpp>


class Util
{
public:
    /**
     * @brief Util::getMinMax Get the minimum and maximum value for each dimension and store them in vectors.
     * @param points
     * @param minPoint minimum values in every dimension.
     * @param maxPoint maximum values in every dimension.
     */
    template <typename Allocator> // appease compiler to work with any allocator
    static void getMinMax(std::vector<pcl::PointXYZ, Allocator> const points, Eigen::Vector3f& minPoint, Eigen::Vector3f& maxPoint)
    {
        for (pcl::PointXYZ const point : points)
        {
            maxPoint = maxPoint.cwiseMax(point.getVector3fMap());

            minPoint = minPoint.cwiseMin(point.getVector3fMap());
        }
    }
};


#endif // UTIL_H
