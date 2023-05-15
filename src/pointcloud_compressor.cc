#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <magni_octonav/PointCloudByte.h>

#include <sstream>
#include <memory>

ros::Publisher pub;

std::unique_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZ>> PointCloudEncoder;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
  // Create a container for the data.
  magni_octonav::PointCloudByte output;
  // std_msgs::String outputString;

  // convert to pointxyzrgba type
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*input, *temp_cloud);

  // stringstream to store compressed point cloud
  std::stringstream compressedData;
  // output pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>());

  // compress point cloud
  PointCloudEncoder->encodePointCloud(temp_cloud, compressedData);

  auto str = compressedData.str();

  std::vector<uint8_t> &buffer = output.bytes;

  buffer.insert(buffer.end(), str.cbegin(), str.cend());

  output.header.frame_id = input->header.frame_id;
  output.header.stamp = ros::Time::now();

  // Publish the data.
  pub.publish(output);
}

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "cloud_compressor");
  ros::NodeHandle nh;

  // instantiate point cloud compression for encoding and decoding
  PointCloudEncoder.reset(new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, false));

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<magni_octonav::PointCloudByte>("/velodyne_points/compressed", 1);

  // Spin
  ros::spin();
}