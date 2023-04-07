// SPDX-License-Identifier: BSD-2-Clause

#pragma once

#include <vector>
#include <numeric>

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_types.h>

#define DELETE_PTR_CHECK(ptr_var)                                              \
  if (ptr_var != nullptr) {                                                    \
    delete ptr_var;                                                            \
    ptr_var = nullptr;                                                         \
  }

class MapGridProjector {
public:
  using PointT = pcl::PointXYZI;
  using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>;

  struct KeyFrameSnapshot {
    using Ptr = boost::shared_ptr<KeyFrameSnapshot>;
  };


  MapGridProjector(const ros::NodeHandle &nh_ = ros::NodeHandle(), const ros::NodeHandle &private_nh_ = ros::NodeHandle("~"));
  virtual ~MapGridProjector();


  void scan_handler(const sensor_msgs::LaserScan::ConstPtr & scan_msg);

private:
  inline float ProbabilityToLogOdds(float prob) const {
    return std::log(prob / (1.f - prob));
  }

  inline float LogOddsToProbability(float odd) const {
    return 1.f / (1.f + std::exp(-odd));
  }

  float inverse_sensor_model(const Eigen::Vector2i& c, const Eigen::Vector2i& range, const double& resolution) const {

    if(std::sqrt((double)c.squaredNorm()) < (std::sqrt((double)range.squaredNorm()) - 0.5 * resolution)) {
      return m_log_odd_free;
    }
    if(std::sqrt((double)c.squaredNorm()) > (std::sqrt((double)range.squaredNorm()) + 0.5 * resolution)) {
      return m_log_odd_occ;
    }

    return m_log_odd_prior;
  }

  std::vector<Eigen::Vector2i> bresenham_line(Eigen::Vector2i p1, Eigen::Vector2i p2) const;



  ros::NodeHandle m_nh;
  ros::NodeHandle m_pnh;

  std::string m_worldFrameId; // the map frame
  std::string m_sensorFrameId; // base frame of the sensor

  double m_minScanRange;    // minimum range of the laser scanner
  double m_maxScanRange;    // maximum range of the laser scanner

  double m_resolution;      // map resolution

  nav_msgs::OccupancyGridPtr m_gridmap_p;

  tf::TransformListener tf_listener;

  message_filters::Subscriber<sensor_msgs::PointCloud2> *m_map_point_sub;
  message_filters::Subscriber<sensor_msgs::LaserScan> *m_laser_sub;
  tf::MessageFilter<sensor_msgs::LaserScan> *m_tf_scan_sub;

  ros::Publisher m_map_grid_pub;

  const float m_log_odd_free;
  const float m_log_odd_occ;
  const float m_log_odd_prior;
};