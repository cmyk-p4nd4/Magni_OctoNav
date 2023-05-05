#include <cloud_preprocessor.hpp>

#include <iostream>

#include "option_parser.h"
#include <pc_utility.hpp>

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/flann_search.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/plane_clipper3D.h>

#include <Eigen/Core>

pcl::PointCloud<pcl::PointNormal>::Ptr process_cloud(const PointCloudT::ConstPtr& cloud, bool remove_ground) {

	PointCloudT::Ptr p_cloud(new PointCloudT());
	PointCloudN::Ptr p_cloud_normal(new PointCloudN());

	removeTooFarPoints<PointT>(cloud, p_cloud);
	
	// translatePointCloudToOrigin<PointT>(p_cloud);

	pcl::NormalEstimationOMP<PointT, NormalT> ne(4);
	ne.setInputCloud(p_cloud);
	ne.setKSearch(OptionParser::knn_size);

	pcl::search::FlannSearch<PointT>::Ptr tree(new pcl::search::FlannSearch<PointT>);
	ne.setSearchMethod(tree);
	ne.compute(*p_cloud_normal);

	if(p_cloud_normal->size() != p_cloud->size()) {
		std::cout << "Computed Normal and Input Size mismatch!\r\n";
		return nullptr;
	}

	pcl::RegionGrowing<PointT, NormalT> reg;
	reg.setSearchMethod(tree);
	reg.setInputCloud(p_cloud);
	reg.setInputNormals(p_cloud_normal);

	reg.setMinClusterSize(OptionParser::min_cluster_size);
	reg.setNumberOfNeighbours(OptionParser::grow_nn_size);
	reg.setSmoothnessThreshold(DEG2RAD(OptionParser::norm_angle_diff));
	reg.setCurvatureThreshold(OptionParser::curvature_thres);

	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);

	pcl::IndicesPtr index(new pcl::Indices());
	for(auto& cluster : clusters) {
		index->insert(index->end(), cluster.indices.cbegin(), cluster.indices.cend());
	}

	PointCloudNT::Ptr output(new PointCloudNT);
	{
		PointCloudNT::Ptr cloud_pn(new PointCloudNT);
		// combined both data
		pcl::concatenateFields(*p_cloud, *p_cloud_normal, *cloud_pn);

		pcl::ExtractIndices<PointNT> extract;
		extract.setIndices(index);
		extract.setInputCloud(cloud_pn);
		extract.filter(*output);
	}

	if (remove_ground) {
		// remove the ground points associated to the cloud
		PointCloudNT::Ptr temp = extract_ground(output, true);
		temp.swap(output);
	}

	return output;
}

template <typename Point>
static typename pcl::PointIndices
plane_clip(const typename pcl::PointCloud<Point>::ConstPtr& src_cloud,
					 const Eigen::Vector4f& plane) {
	pcl::PlaneClipper3D<Point> clipper(plane);
	pcl::PointIndices indices;

	clipper.clipPointCloud3D(*src_cloud, indices.indices);

	return indices;
}

template <typename Point>
static std::vector<int>
ground_clip(const typename pcl::PointCloud<Point>::ConstPtr& src_cloud,
						const Eigen::Vector2f& limit, bool negative = false) {
	typename pcl::PassThrough<Point> pass;
	std::vector<int> indices;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(limit.coeff(0), limit.coeff(1));
	pass.setNegative(negative);
	pass.setInputCloud(src_cloud);

	pass.filter(indices);
	return indices;
}
Eigen::Vector4f extract_floor_coeff(const pcl::PointCloud<pcl::PointNormal>::ConstPtr & cloud) {
	// crop ground plane out (index based)
	std::vector<int> ground_idx = ground_clip<PointNT>(cloud, Eigen::Vector2f(-0.05, 0.15));

	// create a RANSAC plane segmente model
	pcl::SampleConsensusModelPlane<PointNT>::Ptr model_p(
		new pcl::SampleConsensusModelPlane<PointNT>(cloud, ground_idx));
	pcl::RandomSampleConsensus<PointNT> ransac(model_p);
	ransac.setNumberOfThreads(4);
	ransac.setDistanceThreshold(0.08);
	ransac.computeModel();

	Eigen::VectorXf coeff;
	ransac.getModelCoefficients(coeff);
	return coeff.head<4>();
}

/**
 * @brief Extract ground cloud from point cloud
 * 
 * @param cloud input cloud
 * @param negative set to true if remove ground instead of retaining it, otherwise false
 * @return pcl::PointCloud<pcl::PointNormal>::Ptr 
 */
pcl::PointCloud<pcl::PointNormal>::Ptr
extract_ground(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud, bool negative) {
	// crop ground plane out (index based)
	std::vector<int> ground_idx = ground_clip<PointNT>(cloud, Eigen::Vector2f(-0.05, 0.15));

	// create a RANSAC plane segmente model
	pcl::SampleConsensusModelPlane<PointNT>::Ptr model_p(
		new pcl::SampleConsensusModelPlane<PointNT>(cloud, ground_idx));
	pcl::RandomSampleConsensus<PointNT> ransac(model_p);
	ransac.setNumberOfThreads(4);
	ransac.setDistanceThreshold(0.08);
	ransac.computeModel();

	// retrieve segmented index from RANSAC model
	boost::shared_ptr<std::vector<int>> inliers(new std::vector<int>);
	ransac.getInliers(*inliers);

	// Extract clouds specified
	pcl::PointCloud<PointNT>::Ptr remain(new pcl::PointCloud<PointNT>);
	pcl::ExtractIndices<PointNT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(boost::shared_ptr<std::vector<int>> (new std::vector<int>(ground_idx)));
	extract.setNegative(negative);
	extract.filter(*remain);
	return remain;
}
