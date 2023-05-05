#include "option_parser.h"
#include <gr_align.hpp>

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/super4pcs.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/search/flann_search.h>

#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>

#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/gicp/gicp_settings.hpp>

#include <Eigen/Dense>

#include <boost/format.hpp>

using namespace pcl;

typedef PointNormal PointNT;
typedef PointCloud<PointNT> PointCloudNT;

template <typename PointType>
static void downsampleCloud(const typename PointCloud<PointType>::ConstPtr& cloud,
														float downsampleSize,
														typename PointCloud<PointType>::Ptr& out) {
	pcl::VoxelGrid<PointType> vox;
	vox.setLeafSize(downsampleSize, downsampleSize, downsampleSize);
	vox.setInputCloud(cloud);
	vox.filter(*out);
}

bool gr_algo_align(const PointCloudNT::ConstPtr& cloud_src,
									 const PointCloudNT::ConstPtr& cloud_tgt,
									 Eigen::Matrix4f * final_transformation,
									 float* fitness,
									 const bool& skip_search) {

	pcl::Super4PCS<PointNT, PointNT> pcs_align;
	pcl::Super4PCS<pcl::PointNormal, pcl::PointNormal>::OptionType& options = pcs_align.getOptions();
	OptionParser::setOptionsFromArgs(options);

	pcs_align.setMaximumIterations(OptionParser::align_iter_n);

	registration::TransformationEstimation2D<PointNT, PointNT>::Ptr trans_est(
		new registration::TransformationEstimation2D<PointNT, PointNT>());
	pcs_align.setTransformationEstimation(trans_est);

	pcs_align.setTransformationEpsilon(1e-5);

	pcs_align.setInputTarget(cloud_tgt);
	pcs_align.setInputSource(cloud_src);

	if (!skip_search) {
		const float source_spacing = average_spacing(cloud_src, 6, false, 50000);
		const float target_spacing = average_spacing(cloud_tgt, 6, true);

		std::printf("Target Cloud Resolution: %.3f\t Size: %ld\r\n", target_spacing, cloud_tgt->size());
		std::printf("Source Cloud Resolution: %.3f\t Size: %ld\r\n", source_spacing, cloud_src->size());

		pcl::KdTreeFLANN<pcl::PointNormal>::Ptr globalTree(new pcl::KdTreeFLANN<pcl::PointNormal>);
		PointCloudNT::Ptr downsampled_cloud_tgt(new PointCloudNT);
		downsampleCloud<pcl::PointNormal>(cloud_tgt, target_spacing * 1.5, downsampled_cloud_tgt);
		globalTree->setInputCloud(downsampled_cloud_tgt);

		float initial_overlap_guess = 0.7f;
		float final_overlap_guess = 0.3f;

		float max_lap = 0.0f;
		float best_est = 0.0f;
		float best_fit = 0.0f;

		for(float overlap = initial_overlap_guess; overlap > final_overlap_guess; overlap -= 0.01f) {
			options.configureOverlap(overlap);

			PointCloudNT::Ptr testcloud(new PointCloudNT());
			pcs_align.align(*testcloud);
			float fit = pcs_align.getFitnessScore();
			float r = computeOverlapRatio(globalTree, testcloud, target_spacing * 2);
			std::cout << boost::format("Estimate: %1$.3f\t Fitness: %2$.3f\t Overlap Ratio: %3$.3f") % overlap % pcs_align.getFitnessScore() % r;
			if(r >= max_lap + 5e-4f && fit > best_fit) {
				best_est = options.getOverlapEstimation();
				max_lap = r;
				best_fit = fit;
				std::cout << " -> Updated!";
			}
			std::cout << std::endl;
		}
		options.configureOverlap(best_est);
	}

	PointCloudNT::Ptr testcloud(new PointCloudNT());
	pcs_align.align(*testcloud);

	if(fitness != nullptr) {
		*fitness = pcs_align.getFitnessScore();
	}
	if(final_transformation != nullptr) {
		*final_transformation = pcs_align.getFinalTransformation();
	}
	std::cout << std::endl;

	return false;
}

void refine_align(const PointCloudNT::ConstPtr& cloud_src,
									const PointCloudNT::ConstPtr& cloud_tgt,
									Eigen::Matrix4f& final_transformation,
									const Eigen::Matrix4f& initial_guess,
									float* fitness) {

	PointCloudNT::Ptr src_filtered(new PointCloudNT());
	PointCloudNT::Ptr tgt_filtered(new PointCloudNT());

	downsampleCloud<PointNT>(cloud_src, 0.5, src_filtered);
	downsampleCloud<PointNT>(cloud_tgt, 0.5, tgt_filtered);

	PointCloudNT::Ptr _matcher_unused(new PointCloudNT());

	// pcl::GeneralizedIterativeClosestPoint<PointNT, PointNT> matcher;
	fast_gicp::FastVGICP<PointNT, PointNT> matcher;
	// matcher.setResolution(0.1);
	matcher.setNumThreads(6);
	matcher.setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
	matcher.setRegularizationMethod(fast_gicp::RegularizationMethod::FROBENIUS);

	matcher.setInputSource(src_filtered);
	matcher.setInputTarget(tgt_filtered);

	matcher.setMaxCorrespondenceDistance(5);
	matcher.setCorrespondenceRandomness(20);
	matcher.setTransformationEpsilon(1e-5);
	// matcher.setTransformationRotationEpsilon(1e-5);
	matcher.setEuclideanFitnessEpsilon(0.01);

	matcher.setMaximumIterations(100);
	matcher.align(*_matcher_unused, initial_guess);

	final_transformation = matcher.getFinalTransformation();
	if(fitness != nullptr) {
		*fitness = matcher.getFitnessScore();
	}
}

float computeOverlapRatio(const pcl::KdTreeFLANN<pcl::PointNormal>::ConstPtr& targetTree,
													const pcl::PointCloud<pcl::PointNormal>::ConstPtr& query_cloud,
													const float queryRadius) {

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	size_t pointCount = 0;

	// search for nearby points in the target cloud frame
	for(const PointNormal& query_pt : query_cloud->points) {
		int foundCount = targetTree->radiusSearch(
			query_pt, queryRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		if(foundCount < 2)
			continue;
		pointCount++;
	}

	float ratio = static_cast<float>(pointCount) / static_cast<float>(query_cloud->size());

	return ratio;
}

float average_spacing(const PointCloud<PointNormal>::ConstPtr& cloud,
											int k,
											bool accurate,
											int samples) {
	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setInputCloud(cloud);

	double total = 0.0;
	int num = cloud->size();

	size_t step = 1;
	if(!accurate && num > samples)
		step = num / samples;
	size_t total_count = 0;
	for(int i = 0; i < num; i += step) {
		const PointNormal& pn = cloud->at(i);
		std::vector<int> k_indices;
		std::vector<float> k_sqr_distances;
		int nbs = kdtree.nearestKSearch(pn, k, k_indices, k_sqr_distances);
		if(nbs <= 1 || nbs != (int)k_sqr_distances.size()) {
			continue;
		}

		double avg = 0.0;
		for(int i = 1; i < nbs; ++i) { // starts from 1 to exclude itself
			avg += std::sqrt(k_sqr_distances[i]);
		}
		total += (avg / nbs);
		++total_count;
	}

	return static_cast<float>(total / total_count);
}