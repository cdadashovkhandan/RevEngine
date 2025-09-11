#ifndef DBSCAN_HPP
#define DBSCAN_HPP

#pragma once

#include <cassert>
#include <vector>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>

std::vector<pcl::PointIndices::Ptr>* dbscan(const std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>& data, float eps, int min_pts);

#endif //DBSCAN_HPP
