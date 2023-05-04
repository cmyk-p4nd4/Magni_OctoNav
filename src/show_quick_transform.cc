#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <Eigen/Core>

int main(void) {

	pcl::PointCloud<pcl::PointNormal>::Ptr target(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointCloud<pcl::PointNormal>::Ptr source(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointCloud<pcl::PointNormal>::Ptr transformed_source(
		new pcl::PointCloud<pcl::PointNormal>());

	Eigen::Matrix4d transformation;
	transformation << 0.211327 ,   0.977383,  0.00798653   ,  39.7168,
  -0.977407  ,  0.211352, -0.00239336   ,  73.1286,
-0.00402716, -0.00730026  ,  0.999965 ,   0.107074,
	0., 0., 0., 1.;

	pcl::PLYReader r;

	r.read<pcl::PointNormal>("/home/felix/project-ws/catkin_ws/data/vlp_map_402.ply", *target);
	r.read<pcl::PointNormal>("/home/felix/project-ws/catkin_ws/data/vlp_map_403.ply", *source);

	pcl::transformPointCloud(*source, *transformed_source, transformation);

	pcl::visualization::PCLVisualizer visu;
	visu.addPointCloud<pcl::PointNormal>(
		transformed_source,
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(0, 255, 255),
		"transformed");
	visu.addPointCloud<pcl::PointNormal>(
		target,
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(0, 255, 0),
		"target");
	visu.spin();

	return 0;
}