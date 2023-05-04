// SPDX-License-Identifier: BSD-2-Clause

#pragma once

#include <vector>
#include <array>
#include <numeric>
#include <memory>

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

#include <opencv4/opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define DELETE_PTR_CHECK(ptr_var)                                              \
  if (ptr_var != nullptr) {                                                    \
    delete ptr_var;                                                            \
    ptr_var = nullptr;                                                         \
  }

template <typename T>
struct point {
  inline point() : x(0), y(0) {}
  inline point(T _x, T _y) : x(_x), y(_y) {}
  T x, y;
};

typedef point<int> IntPoint;
typedef point<unsigned> UIntPoint;
typedef point<double> Point;

class MapGridInfo {
public:

  using Bound = std::array<Point, 2>;  // data ordered in (lx, ly, ux, uy)

  MapGridInfo() {}
  ~MapGridInfo() {}

  void setResolution(double _res) {
    this->map_res = _res;
    this->updateInfo();
  }
  const double getResolution() const { return this->map_res; }

  const size_t getMapSizeX() const { return this->mapSizeX; }
  const size_t getMapSizeY() const { return this->mapSizeY; }

  void setUpperBound(Point bound) {
    this->setUpperBound(bound.x, bound.y);
  }
  
  void setUpperBound(double x, double y) {
    this->max_x = x;
    this->max_y = y;
    this->updateInfo();
  }

  const Point getUpperBound() const { return Point(this->max_x, this->max_y); }

  void setLowerBound(Point bound) {
    this->setLowerBound(bound.x, bound.y);
  }

  void setLowerBound(double x, double y) {
    this->min_x = x;
    this->min_y = y;
    this->updateInfo();
  }

  const Point getLowerBound() const { return Point(this->min_x, this->min_y); }

  const Bound getMapBound() const { return {Point(min_x, min_y), Point(max_x, max_y)}; }

  void updateInfo() {
    mapSizeX = static_cast<size_t>(std::ceil((max_x - min_x) / map_res));
    mapSizeY = static_cast<size_t>(std::ceil((max_y - min_y) / map_res));
    mapSizeHalfX = mapSizeX >> 1;
    mapSizeHalfY = mapSizeY >> 1;
  }

  const Eigen::Vector2d GridToPoint(const Eigen::Vector2i & grid) const {
    auto p = this->GridToPoint(IntPoint(grid.x(), grid.y()));
    return Eigen::Vector2d(p.x, p.y);
  }

  const Point GridToPoint(const IntPoint & grid) const {
    double _gx = min_x + static_cast<double>(grid.x * map_res);
    double _gy = min_y + static_cast<double>(grid.y * map_res);
    return Point(_gx, _gy);
  }

  const Eigen::Vector2i PointToGrid(const Eigen::Vector2d & point) const {
    auto p = this->PointToGrid(Point(point.x(), point.y()));
    return Eigen::Vector2i(p.x, p.y);
  }

  const IntPoint PointToGrid(const Point & point) const {
    int _gy = static_cast<int>(std::round((point.x - min_x) / map_res));
    int _gx = static_cast<int>(std::round((point.y - min_y) / map_res));
    return IntPoint(_gx, _gy);
  }

private:

  double min_x, max_x;
  double min_y, max_y;
  
  double map_res;

  size_t mapSizeX;
  size_t mapSizeY;
  size_t mapSizeHalfX;
  size_t mapSizeHalfY;
};

class MapGridProjector {
public:
  using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>;
  using Vector2u = Eigen::Matrix<unsigned int, 2, 1>;

  MapGridProjector(const ros::NodeHandle &nh_ = ros::NodeHandle(), const ros::NodeHandle &private_nh_ = ros::NodeHandle("~"));
  ~MapGridProjector() {};


  void scan_handler(const sensor_msgs::LaserScan::ConstPtr & scan);

protected:
  // void initMap(const sensor_msgs::LaserScan::ConstPtr & scan_msg);

private:
  inline double ProbabilityToLogOdds(const double prob) const {
    return std::log(prob / (1.f - prob));
  }

  inline double LogOddsToProbability(const double odd) const {
    return 1.f / (1.f + std::exp(-odd));
  }

  double inverse_sensor_model(const Eigen::Vector2i& c, const Eigen::Vector2i& range, const double& resolution) const {
    if(std::sqrt((double)c.squaredNorm()) < (std::sqrt((double)range.squaredNorm()) - 0.5 * resolution)) {
      return m_log_odd_free;
    }
    if(std::sqrt((double)c.squaredNorm()) > (std::sqrt((double)range.squaredNorm()) + 0.5 * resolution)) {
      return m_log_odd_occ;
    }
    return m_log_odd_prior;
  }

  inline double quaternion_to_yaw(tf::Quaternion q) {
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    return std::atan2(siny_cosp, cosy_cosp);
  }

  template <int _el>
  static inline bool compare(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) {
    return p1.coeff(static_cast<Eigen::Index>(_el)) < p2.coeff(static_cast<Eigen::Index>(_el));
  }

  inline double normalize_angle(double angle) {
    return std::fmod((angle + M_PI), (2 * M_PI)) - M_PI;
  }

  int8_t ratioToMap(double value) const {
    if (value > m_log_odd_clamp_max) {
      return static_cast<int8_t>(100);
    }

    if (value < m_log_odd_clamp_min) {
      return static_cast<int8_t>(0);
    }

    return static_cast<int8_t>(-1);
  }

  /* returns a vector of cells the laser passes through */
  std::vector<IntPoint> raycast(const Eigen::Vector2i p1, const Eigen::Vector2i p2);

  void constructMap(const std::vector<Eigen::Vector3d> & points, const Eigen::Translation2d & pose);

  ros::NodeHandle m_nh;
  ros::NodeHandle m_pnh;

  std::string m_worldFrameId; // the map frame
  std::string m_sensorFrameId; // base frame of the sensor

  double m_log_odd_free;
  double m_log_odd_occ;
  const double m_log_odd_prior = 0.0;
  const double m_log_odd_clamp_max = 0.8;
  const double m_log_odd_clamp_min = 0.2;

  double m_resolution;      // map resolution

  std::string map_topic;    // topic to public map to
  std::string scan_topic;   // topic to subscribe laser scan from

  nav_msgs::OccupancyGridPtr m_gridmap_ptr;
  MapGridInfo m_mapInfo;
  bool initMap;

  cv::Mat m_cell_log_odd;

  tf::TransformListener tf_listener;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> m_map_point_sub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> m_laser_sub;
  std::unique_ptr<tf::MessageFilter<sensor_msgs::LaserScan>> m_tf_scan_sub;
  std::unique_ptr<tf::MessageFilter<sensor_msgs::PointCloud2>> m_tf_cloud_sub;

  ros::Publisher m_map_grid_pub;
};