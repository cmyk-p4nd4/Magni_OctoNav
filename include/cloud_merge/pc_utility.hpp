#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template <typename Point>
void translatePointCloudToOrigin(const typename pcl::PointCloud<Point>::Ptr& cloud);

template <typename Point>
void removeTooFarPoints(const typename pcl::PointCloud<Point>::ConstPtr& cloud,
												const typename pcl::PointCloud<Point>::Ptr& out);

#include <pc_utility_impl.hpp>