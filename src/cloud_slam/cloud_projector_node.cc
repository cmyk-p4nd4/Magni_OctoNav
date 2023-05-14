#include <ros/ros.h>
#include <cloud_slam/cloud_projector.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_projector_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // create CloudProjector class
  magni_octonav::CloudProjector n(nh, pnh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}