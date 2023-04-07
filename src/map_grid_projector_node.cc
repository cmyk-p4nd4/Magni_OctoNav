#include <map_grid_projector.hpp>

#include <ros/ros.h>


int main(int argc, char ** argv) {
    ros::init(argc, argv, "map_grid_projector_node");

    const ros::NodeHandle nh;
    const ros::NodeHandle pnh("~");

    MapGridProjector projector(nh, pnh);


    ROS_INFO("MapGridGeneartor Running\r\n");
    try {
      ros::spin();
    } catch (std::runtime_error &e) {
      ROS_ERROR("map_grid_projector_node exception: %s", e.what());
      return -1;
    }

    return 0;
} 