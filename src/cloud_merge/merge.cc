/**
 * @file pointcloud_growing.cc
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-12-08
 * 
 * @copyright Copyright (c) 2022
 * 
 * @note 
 *  Run with:
 *  `rosrun pointcloud_coop_slam pointcloud_growing -p map_102.pcd -filter 1 -near -n 25 -c 4 -g 20 -s 10 -mc 70`
 *  Run with:
 *  ```
 *    for file in $(find vlp_map_20*([0-9]).pcd); do 
 *        rosrun pointcloud_coop_slam pointcloud_growing -p ${file} -filter 1 -near -n 35 -c 6 -g 30 -s 4 -mc 100
 *    done
 *  ```
 *  Run with
 * 	`./bin/overlap_4pcs -f ~/project-ws/catkin_ws/data/vlp_map_4??([0-2]).pcd -d 0.1 -o 0.6`
 *
 * 
 */

#include <algorithm>
#include <fstream>
#include <iostream>
#include <thread>
#include <utility>
#include <vector>

/* LIB C */
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>

// pcl includes

#include <pcl/common/file_io.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/common/transforms.h>

#include <boost/chrono.hpp>

#include <pc_utility.hpp>
#include <mt_plotter.hpp>
#include <cloud_preprocessor.hpp>
#include <gr_align.hpp>
#include "option_parser.h"
#include "typedefs.h"
#include <utils.hpp>

#include <ros/ros.h>

#include <Eigen/Core>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

std::array<float, 10> overlap_guess_list;

typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

int main(int argc, char** argv) {
	using namespace pcl::console;
	using namespace boost::chrono;
	using namespace pcl;

	ros::init(argc ,argv,"overlap_registration_node");

	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	ros::Publisher global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1, true);


	int ret_val = OptionParser::parse_argument(argc, argv);
	if (ret_val != 0) {
		OptionParser::printUsage(argv);
		return ret_val;
	}


	PCDReader reader;
	std::vector<PointCloudT::Ptr> clouds;
	/* Load some files */
	for(auto input = OptionParser::clouds_inputs.cbegin();
			input != OptionParser::clouds_inputs.cend();
			input++)
	{
		PointCloudT::Ptr pc(new PointCloudT);
		std::cout << "Loading file " << *input << "\r\n";
		if (reader.read(*input, *pc) < 0) {
			std::cout << "Failed to load " << *input << "\r\n";
			continue;
		}
		clouds.emplace_back(pc);
	}

	if (clouds.empty()) {
		std::cout << "No input" << std::endl;
		return -1;
	}

	std::vector<PointCloudNT::Ptr> cloud_info_vec;
	for(auto pc_it = clouds.cbegin(); pc_it != clouds.cend(); pc_it++) {
		PointCloudNT::Ptr pcnt = process_cloud(*pc_it);
		cloud_info_vec.emplace_back(pcnt);
	}

	if (cloud_info_vec.size() < 2) {
		std::cout << "Cloud size < 2. Cannot perform cloud alignment. Abort" << std::endl;
		return -2;
	}

	sensor_msgs::PointCloud2 cloud_msgs;

	// list of cloud with incremental registration
	// alignment are done to the element in front
	// pc1 <-> pc2 --> cb1
	// cb1 <-> pc3 --> cb2 ... etc
	
	PointCloudNT::Ptr global_cloud(new PointCloudNT());

	for (size_t i = 1; i < cloud_info_vec.size(); i++) {
		PointCloudNT::Ptr target = cloud_info_vec.at(i - 1);
		if (!global_cloud->empty()) {
			target = global_cloud;
		}
		PointCloudNT::Ptr source = cloud_info_vec.at(i);
		if (target->size() < source->size()) {
			std::swap(target, source);
		}

		float rough_fitness = 0.0f;
		Eigen::Matrix4f rough_transformation = Eigen::Matrix4f::Identity();
		gr_algo_align(source, target, &rough_transformation, &rough_fitness, OptionParser::skip_benchmarking);

		Eigen::Matrix4f transform;
		float final_fitness = 0.0f;
		refine_align(source, target, transform, rough_transformation, &final_fitness);

		PointCloudNT::Ptr t_cloud(new PointCloudNT());
		pcl::transformPointCloud(*source, *t_cloud, transform);

		PointCloudNT combined_cloud;
		combined_cloud += *target;
		combined_cloud += *t_cloud;

		*global_cloud += combined_cloud;

		pcl::toROSMsg(*global_cloud, cloud_msgs);

		cloud_msgs.header.frame_id = "map";
		cloud_msgs.header.stamp = ros::Time::now();
		global_map_pub.publish(cloud_msgs);
		ros::spinOnce();
	}

	// std::cout << "Merge Complete" << std::endl;
	ROS_INFO_STREAM("Merge Complete");

	ros::spin();

	// pcl::VoxelGrid<PointNT> vox;
	// vox.setInputCloud(global_cloud);
	// vox.setLeafSize(0.05, 0.05,0.05);
	// {
	// 	PointCloudNT::Ptr pc(new PointCloudNT());
	// 	vox.filter(*pc);
	// 	global_cloud.swap(pc);
	// }

	// pcl::visualization::PCLVisualizer visu;
	// visu.addPointCloud(global_cloud, ColorHandlerT(global_cloud, 255, 255, 255), "combined");
	// visu.spin();

	std::cout << std::endl;

	return (0);
}