#pragma once

#include <memory>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <cloud_slam/cloud_type.hpp>

#include <boost/optional.hpp>

#include <Eigen/Core>

namespace magni_octonav {

class FeatureExtractor {
public:
  FeatureExtractor(ros::NodeHandle &_nh, ros::NodeHandle &_pnh) {
    this->init_params(_nh, _pnh);
  }

  ~FeatureExtractor() {}

  void init_params(ros::NodeHandle &_nh, ros::NodeHandle &_pnh);

protected:
  void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

	boost::optional<Eigen::Vector4f> floor_detection(const pcl::PointCloud<PointXYZRLCI>::ConstPtr &_seg_cloud);

  void ground_points_align(const pcl::PointCloud<pcl::PointXYZ> &_ground_points);

private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  // subscribers
  ros::Subscriber cloud_sub_;

  // publishers
  ros::Publisher floor_coeff_pub_;
  ros::Publisher feature_cloud_pub_;

  // transform listener
  tf::TransformListener tf_listener_;

  // frame ids
  std::string base_frame_id_;

	// cloud dimension info
  unsigned channel_count_;
  unsigned laser_count_;

  double angle_resolution;
  double beam_resolution;
};

}  // namespace magni_octonav