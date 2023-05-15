#include <ros/ros.h>

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>
#include <magni_octonav/PointCloudByte.h>

std::unique_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZ>> PointCloudDecoder;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());

ros::Publisher pc_pub;


void byteStreamCallback(const magni_octonav::PointCloudByteConstPtr& byte_msg) {
  pc->clear();

  std::stringstream st;
  for (std::size_t i = 0; i < byte_msg->bytes.size(); i++) {
    st << static_cast<char>(byte_msg->bytes.at(i));
  }

  PointCloudDecoder->decodePointCloud(st, pc);

  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(*pc, *cloud_msg);

  cloud_msg->header.frame_id = byte_msg->header.frame_id;
  cloud_msg->header.stamp = ros::Time::now();
  pc_pub.publish(cloud_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "octree_decompress_node");
  ros::NodeHandle nh;

  PointCloudDecoder.reset(new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, false));

  ros::Subscriber sub = nh.subscribe<magni_octonav::PointCloudByte>("/velodyne_points/compressed", 10, &byteStreamCallback);
  pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/decompressed_cloud", 1, true);

  ros::spin();
}