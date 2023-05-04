#include <pcl/common/io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>

#include <iostream>
#include <string>

#include "option_parser.h"

std::vector<int> region_growing(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& raw,
																const pcl::PointCloud<pcl::Normal>::Ptr& normal) {

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-0.05, 1.5);
	pass.setInputCloud(raw);

	pcl::IndicesPtr a(new pcl::Indices());
	pass.filter(*a);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setInputCloud(raw);
	reg.setIndices(a);
	reg.setInputNormals(normal);
	reg.setSmoothModeFlag(false);

	reg.setMinClusterSize(OptionParser::min_cluster_size);
	reg.setNumberOfNeighbours(OptionParser::grow_nn_size);
	reg.setSmoothnessThreshold(DEG2RAD(OptionParser::norm_angle_diff));
	reg.setCurvatureThreshold(OptionParser::curvature_thres);

	std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  std::vector<int> indices;
  for (auto & cluster : clusters) {
    indices.insert(indices.end(), cluster.indices.cbegin(), cluster.indices.cend());
  }

	return indices;
}

int main(int argc, char** argv) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::Normal>::Ptr ne_cloud(new pcl::PointCloud<pcl::Normal>());
	pcl::PointCloud<pcl::PointNormal>::Ptr result(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PLYWriter w;

	for(int i = 1; i < argc; i++) {
		int ret = pcl::io::loadPCDFile(std::string(argv[i]), *cloud);

		if(ret < 0) {
			continue;
		}

		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne(6);
		ne.setInputCloud(cloud);
		ne.setKSearch(30);
		ne.compute(*ne_cloud);

    pcl::PointIndices point;
    point.indices = region_growing(cloud, ne_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr c1(new pcl::PointCloud<pcl::PointXYZ>());
	  pcl::PointCloud<pcl::Normal>::Ptr c2(new pcl::PointCloud<pcl::Normal>());
    
    pcl::copyPointCloud(*cloud, point, *c1);
    pcl::copyPointCloud(*ne_cloud, point, *c2);

		pcl::concatenateFields(*c2, *c1, *result);

		std::string fullname = std::string(argv[i]);
		auto index = fullname.find_last_of(".");

		w.write<pcl::PointNormal>(fullname.substr(0, index) + ".ply", *result, true, true);

		cloud->clear();
		ne_cloud->clear();
		result->clear();
	}

	return 0;
}