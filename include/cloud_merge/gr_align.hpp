#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>

/**
 * @brief Runs the cloud matching algorithm
 * 
 * @param[in] cloud_src source cloud
 * @param[in] cloud_tgt target cloud
 * @param[in] final_transformation transformation matrix from the alignment algorithm
 * @param[in] fitness LCP value
 * @return whether the order of alignment is swapped
 */
bool gr_algo_align(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud_src,
									 const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud_tgt,
									 Eigen::Matrix4f * final_transformation = nullptr,
									 float* fitness = nullptr,
									 const bool& skip_search = false);

void refine_align(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud_src,
									const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud_tgt,
									Eigen::Matrix4f& final_transformation,
									const Eigen::Matrix4f& initial_guess = Eigen::Matrix4f::Identity(),
									float* fitness = nullptr);

float computeOverlapRatio(const pcl::KdTreeFLANN<pcl::PointNormal>::ConstPtr& targetTree,
													const pcl::PointCloud<pcl::PointNormal>::ConstPtr& query_cloud,
													const float queryRadius);

/**
 * \brief Query the average spacing of a point cloud.
 * @param cloud The point cloud.
 * @param k The number of nearest points used.
 * @param accurate True to use every point to get an accurate calculation; false to obtain aa approximate
 *                 measure, which uses only a subset (i.e., less than samples) of the points.
 * @param samples  Use how many samples of points for the calculation.
 * @return The average spacing of the point cloud.
 */
float average_spacing(const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud,
											int k,
											bool accurate = false,
											int samples = 10000);