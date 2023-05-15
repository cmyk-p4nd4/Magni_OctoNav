#pragma once

#include <memory>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <cloud_slam/cloud_type.hpp>

#include <boost/optional.hpp>

#include <Eigen/Core>

#include <fast_gicp_warp/fast_gicp_warp.hpp>

namespace magni_octonav {

class FeatureExtractor {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
  ros::Publisher odom_pub_;

  // transform listener
  tf::TransformListener tf_listener_;
  // transform broadcaster
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster keyframe_broadcaster;

  // frame ids
  std::string base_frame_id_;
  std::string odom_frame_id_;

  // keyframe parameters
  double keyframe_delta_trans;  // minimum distance between keyframes
  double keyframe_delta_angle;  //
  double keyframe_delta_time;   //

  // registration validation by thresholding
  bool transform_thresholding;  //
  double max_acceptable_trans;  //
  double max_acceptable_angle;

  std::string odom_pub_topic_;

  bool publish_transform;

  ros::Time prev_time;
  Eigen::Matrix4f prev_trans;                         // previous estimated transform from keyframe
  Eigen::Matrix4f keyframe_pose;                      // keyframe pose
  ros::Time keyframe_stamp;                           // keyframe time
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr keyframe;  // keyframe point cloud

  fast_gicp::FastGICPWarp<pcl::PointXYZ, pcl::PointXYZ>::Ptr matcher_;

};

}  // namespace magni_octonav