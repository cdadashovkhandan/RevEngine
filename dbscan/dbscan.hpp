#ifndef DBSCAN_HPP
#define DBSCAN_HPP

#pragma once

#include <cassert>
#include <vector>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>

std::vector<pcl::PointIndices::Ptr>* dbscan(const std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>& data, float eps, int min_pts);

// template<size_t dim>
// auto dbscan(const std::span<float>& data, float eps, int min_pts)
// {
//     static_assert(dim == 2 or dim == 3, "This only supports either 2D or 3D points");
//     assert(data.size() % dim == 0);

//     if(dim == 2)
//     {
//         auto * const ptr = reinterpret_cast<float const*> (data.data());
//         auto points = std::span<const point2>
//     }
// }

#endif //DBSCAN_HPP
