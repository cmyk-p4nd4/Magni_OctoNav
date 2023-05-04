#include <cmath>

#include <pc_utility.hpp>
#include <mt_plotter.hpp>
#include <cloud_preprocessor.hpp>
#include <gr_align.hpp>
#include "option_parser.h"
#include "typedefs.h"
#include "utils.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <plade.h>

typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

int main (int argc, char **argv) {

  using namespace pcl;

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
		PointCloudNT::Ptr pcnt = process_cloud(*pc_it, true);
		cloud_info_vec.emplace_back(pcnt);
	}

	if (cloud_info_vec.size() < 2) {
		std::cout << "Cloud size < 2. Cannot perform cloud alignment. Abort" << std::endl;
		return -2;
	}

  std::vector<Eigen::Matrix4f> transformations;
  // incrementally register two pair of clouds
	for(std::size_t i = 0; i < cloud_info_vec.size() - 1; i++) {
    bool subj_swapped = false;
    
    PointCloudNT::Ptr target(new PointCloudNT(*cloud_info_vec.at(i)));
		PointCloudNT::Ptr source(new PointCloudNT(*cloud_info_vec.at(i + 1)));

    if (source->size() >= target->size() * 1.2f) {
      std::swap(source, target);
      subj_swapped = true;
    }

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    bool success = registration(transformation, target, source);
    if (!success) {
      std::cout << "Registration Failed!" <<"\r\n";
      continue;
    }

    if (subj_swapped) {
      transformation = transformation.inverse().eval();
    }

		// Eigen::Vector3f ea = util::rot_to_euler(transformation);

		// Eigen::AngleAxisf pitchAngle(ea.y(), Eigen::Vector3f::UnitY());
    // Eigen::AngleAxisf yawAngle(ea.z(), Eigen::Vector3f::UnitZ());
		// Eigen::Matrix3f rot_x_fixed = (yawAngle * pitchAngle).toRotationMatrix();

		// transformation.topLeftCorner(3,3) = rot_x_fixed;

		transformations.push_back(transformation);
	}

	for(std::size_t i = 0; i < cloud_info_vec.size() - 1; i++) {
		util::Timer t;
		PointCloudNT::Ptr target(new PointCloudNT(*cloud_info_vec.at(i)));
		PointCloudNT::Ptr source(new PointCloudNT(*cloud_info_vec.at(i + 1)));

		// PointCloudNT::Ptr target_wall = extract_ground(target, true);
		// PointCloudNT::Ptr source_wall = extract_ground(source, true);

		Eigen::Matrix4f &guess = transformations.at(i);

		Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity();
		refine_align(source, target, final_transformation, guess);

		transformations.at(i) = final_transformation;
	}

  std::vector<int> ports;
  pcl::visualization::PCLVisualizer vis("Viewer");
	vis.addCoordinateSystem();
  bool keyEntered = true;
	auto callbackFunc = [&keyEntered](const pcl::visualization::KeyboardEvent& event) {
		if (event.getKeySym () == "n" && event.keyDown ()) {
      keyEntered = true;
    }
	};

	int index = 0;
  PointCloudNT::Ptr transformed_source(new PointCloudNT());
  PointCloudNT::Ptr target = nullptr;

  auto disconnectable = vis.registerKeyboardCallback(callbackFunc);

  while (!vis.wasStopped()) {
    if (keyEntered) {
      keyEntered = false;
      vis.removeAllPointClouds();
      
      target = cloud_info_vec.at(index);
      transformed_source->clear();

			pcl::transformPointCloud(
				*cloud_info_vec.at(index + 1), *transformed_source, transformations.at(index));
				
			vis.addPointCloud<PointNT>(target, ColorHandlerT(target, 255, 255, 255), "target");
			vis.addPointCloud<PointNT>(
				transformed_source, ColorHandlerT(transformed_source, 0, 255, 0), "source");

      index = (index + 1) % static_cast<int>(transformations.size());
		}

    vis.spinOnce();
  }

	return 0;
}