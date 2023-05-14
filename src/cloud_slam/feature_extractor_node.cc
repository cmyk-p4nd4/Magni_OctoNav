#include <ros/ros.h>

#include <cloud_slam/feature_extractor.hpp>

int main (int argc, char ** argv) {
    ros::init(argc, argv, "feature_extractor_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    magni_octonav::FeatureExtractor n(nh, pnh);

    // start up node
    ros::spin();
}