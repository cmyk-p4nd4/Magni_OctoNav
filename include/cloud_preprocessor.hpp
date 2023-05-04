#pragma once

#include <typedefs.h>

pcl::PointCloud<pcl::PointNormal>::Ptr process_cloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud, bool remove_ground = true);

pcl::PointCloud<pcl::PointNormal>::Ptr extract_ground(const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud, bool negative = false);

Eigen::Vector4f extract_floor_coeff(const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud);