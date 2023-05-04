// SPDX-License-Identifier: BSD-2-Clause

#include <algorithm>
#include <limits>
#include <map_grid_projector.hpp>
#include <sstream>
#include <iomanip>

#include <boost/bind.hpp>

#include <ros/console.h>

#include <geometry_msgs/TransformStamped.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf_conversions/tf_eigen.h>

static const Eigen::IOFormat CommaInitFmt(4, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

MapGridProjector::MapGridProjector(const ros::NodeHandle &nh_,
                                   const ros::NodeHandle &private_nh_)
    : m_nh(nh_), m_pnh(private_nh_), m_map_point_sub(nullptr),
      m_laser_sub(nullptr), m_tf_scan_sub(nullptr),

      m_resolution(0.05), map_topic("/projected_map"), scan_topic("/scan"),
      initMap(false) {

  m_pnh.param<std::string>("world_frame", m_worldFrameId, "map");
  m_pnh.param<std::string>("base_frame", m_sensorFrameId, "velodyne");
  m_pnh.param<std::string>("map_topic", map_topic, map_topic);
  m_pnh.param<std::string>("scan_topic", scan_topic, scan_topic);

  double prob_free = 0.3, prob_occ = 0.75;
  double r = 0.05;
  m_pnh.param<double>("free_prob", prob_free, prob_free);
  m_pnh.param<double>("occpuied_prob", prob_occ, prob_occ);
  m_pnh.param<double>("map_resolution", r, 0.05);
  m_mapInfo.setResolution(r);

  m_log_odd_free = ProbabilityToLogOdds(prob_free);
  m_log_odd_occ = ProbabilityToLogOdds(prob_occ);

  // subscriber
  m_map_point_sub.reset(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(
          m_nh, "cloud_in", 16));
  m_laser_sub.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(
      m_nh, scan_topic, 16));
  m_tf_scan_sub.reset(new tf::MessageFilter<sensor_msgs::LaserScan>(
      *m_laser_sub, tf_listener, m_worldFrameId, 16));
  m_tf_scan_sub->registerCallback(boost::bind(&MapGridProjector::scan_handler,
                                              this, boost::placeholders::_1));

  // publisher
  m_map_grid_pub = m_nh.advertise<nav_msgs::OccupancyGrid>(map_topic, 10, true);

  m_gridmap_ptr.reset(new nav_msgs::OccupancyGrid());
}

void MapGridProjector::scan_handler(const sensor_msgs::LaserScan::ConstPtr &scan) {
  ROS_DEBUG("Entered Callback!");

  if (!tf_listener.waitForTransform(m_worldFrameId, m_sensorFrameId,
                                    scan->header.stamp, ros::Duration(0.5))) {
    ROS_WARN_STREAM("Timed out waiting for transform from "
                    << m_worldFrameId << " to " << m_sensorFrameId << " at "
                    << scan->header.stamp.toSec());
    return;
  }


  tf::StampedTransform sensorToWorldTf;
  try {
    tf_listener.lookupTransform(m_worldFrameId, scan->header.frame_id,
                                scan->header.stamp, sensorToWorldTf);
  } catch (tf::LookupException &ex) {
    ROS_ERROR_STREAM("Transform error of sensor data: "
                     << ex.what() << ", quitting callback");
    return;
  }

  double world_x = sensorToWorldTf.getOrigin().x();
  double world_y = sensorToWorldTf.getOrigin().y();
  double world_yaw = this->quaternion_to_yaw(sensorToWorldTf.getRotation());

  std::vector<Eigen::Vector3d> scan_points;

  const double angle_start = scan->angle_min;
  const double angle_step = scan->angle_increment;
  const double angle_end = scan->angle_max;
  for (size_t i = 0; i < scan->ranges.size(); i++) {
    double R = scan->ranges.at(i);
    double angle = angle_start + angle_step * i;
    if (angle >= angle_end) {
      break;
    }
    if (R < scan->range_min || R > scan->range_max) {
      continue;
    }

    double world_laser_x = R * std::cos(angle + world_yaw) + world_x;
    double world_laser_y = R * std::sin(angle + world_yaw) + world_y;


    // Eigen::Vector3d point = {R * std::cos(angle), R * std::sin(angle), 0.0};

    // Eigen::Vector3d pointToWorld = planar_pose * point;
    // scan_points.push_back(pointToWorld);
  }

  // this->constructMap(scan_points, translate);

  // std::transform(m_cell_log_odd.begin<double>(), m_cell_log_odd.end<double>(),
  //                std::back_inserter(m_gridmap_ptr->data),
  //                boost::bind(&MapGridProjector::ratioToMap, this, _1));



  // Point origin = m_mapInfo.getLowerBound();
  // geometry_msgs::Pose pose;
  // pose.position.x = origin.x;
  // pose.position.y = origin.y;
  // pose.orientation.w = 1.0;

  // m_gridmap_ptr->info.resolution = m_mapInfo.getResolution();
  // m_gridmap_ptr->info.origin = pose;
  // m_gridmap_ptr->info.width = m_mapInfo.getMapSizeX();
  // m_gridmap_ptr->info.height = m_mapInfo.getMapSizeY();

  // m_gridmap_ptr->header.frame_id = m_worldFrameId;
  // m_gridmap_ptr->header.stamp = ros::Time::now();

  // this->m_map_grid_pub.publish(m_gridmap_ptr);

  // // need to clear the vector since we use std::back_inserter during
  // // construction
  // m_gridmap_ptr->data.clear();
}

void MapGridProjector::constructMap(const std::vector<Eigen::Vector3d> &points,
                                    const Eigen::Translation2d &pose) {

  double curr_min_x = std::min_element(points.cbegin(), points.cend(),
                                       &MapGridProjector::compare<0>)->x();
  double curr_max_x = std::max_element(points.cbegin(), points.cend(),
                                       &MapGridProjector::compare<0>)->x();
  double curr_min_y = std::min_element(points.cbegin(), points.cend(),
                                       &MapGridProjector::compare<1>)->y();
  double curr_max_y = std::max_element(points.cbegin(), points.cend(),
                                       &MapGridProjector::compare<1>)->y();

  /*
    Data Layout
    (0, 0)
    +------------> X (column)
    |
    |
    |
    |
    V             x(cellSizeX - 1, cellSizeY - 1)
    Y (Row)
  */

  // map not init-ed
  // take the first scan as the map
  if (!initMap) {
    m_mapInfo.setLowerBound(curr_min_x, curr_min_y);
    m_mapInfo.setUpperBound(curr_max_x, curr_max_y);

    m_cell_log_odd = cv::Mat::zeros(m_mapInfo.getMapSizeX(),
                                    m_mapInfo.getMapSizeY(), CV_64F);

    ROS_INFO("Created Grid Map with %ld X %ld @ %.4f m/pix\r\n",
             m_mapInfo.getMapSizeX(), m_mapInfo.getMapSizeY(),
             m_mapInfo.getResolution());

    this->initMap = true;
  } else {
    auto oldBound = m_mapInfo.getMapBound();
    Point lBound = m_mapInfo.getLowerBound();
    Point uBound = m_mapInfo.getUpperBound();

    lBound.x = std::min(lBound.x, curr_min_x);
    lBound.y = std::min(lBound.y, curr_min_y);
    m_mapInfo.setLowerBound(lBound); // overwrite

    uBound.x = std::max(uBound.x, curr_max_x);
    uBound.y = std::max(uBound.y, curr_max_y);
    m_mapInfo.setUpperBound(uBound); // overwrite

    MapGridInfo::Bound newBound = m_mapInfo.getMapBound();

    long old_lx_map =
        std::lround(oldBound.front().x / m_mapInfo.getResolution());
    long old_ly_map =
        std::lround(oldBound.front().y / m_mapInfo.getResolution());
    long old_ux_map =
        std::lround(oldBound.back().x / m_mapInfo.getResolution());
    long old_uy_map =
        std::lround(oldBound.back().y / m_mapInfo.getResolution());

    long new_lx_map =
        std::lround(newBound.front().x / m_mapInfo.getResolution());
    long new_ly_map =
        std::lround(newBound.front().y / m_mapInfo.getResolution());
    long new_ux_map =
        std::lround(newBound.back().x / m_mapInfo.getResolution());
    long new_uy_map =
        std::lround(newBound.back().y / m_mapInfo.getResolution());

    // compute integer different for 4 sides
    long diff_x_pre = std::abs(old_lx_map - new_lx_map);
    long diff_x_post = std::abs(old_ux_map - new_ux_map);
    long diff_y_pre = std::abs(old_ly_map - new_ly_map);
    long diff_y_post = std::abs(old_uy_map - new_uy_map);

    long accu_diff = diff_x_pre + diff_x_post + diff_y_pre + diff_y_post;

    if (accu_diff > 0) {
      cv::Mat temp;

      cv::copyMakeBorder(this->m_cell_log_odd, temp, diff_y_pre, diff_y_post,
                        diff_x_pre, diff_x_post, cv::BORDER_CONSTANT,
                        cv::Scalar(0.0));
      cv::swap(this->m_cell_log_odd, temp);

      ROS_INFO_THROTTLE(1, "Resize Grid Map to %ld X %ld @ %.4f m/pix",
                        m_mapInfo.getMapSizeX(), m_mapInfo.getMapSizeY(),
                        m_mapInfo.getResolution());
    }
  }

  Eigen::Vector2i robot_pose_grid = m_mapInfo.PointToGrid(pose.vector());
  std::stringstream bigss;
  for (size_t i = 0; i < points.size(); i++) {
    Eigen::Vector2d p(points.at(i).x(), points.at(i).y());
    Eigen::Vector2i laser_world_grid = m_mapInfo.PointToGrid(p);
    ROS_DEBUG_STREAM_THROTTLE(1, "\r\n" << robot_pose_grid.format(CommaInitFmt) << "\r\n" << laser_world_grid.format(CommaInitFmt));

    std::vector<IntPoint> cell_index = this->raycast(robot_pose_grid, laser_world_grid);
    bigss << std::setw(4) << std::setfill('0') << i << ":\r\n";

    {
      std::stringstream ss;
      for (size_t j = 0; j < cell_index.size(); j++) {
        ss << '(' << std::setw(4) << cell_index[j].x << ", " << std::setw(4) << cell_index[j].y
           << "), ";
        ROS_WARN_COND(cell_index[j].x < 0 || cell_index[j].y < 0, "Scan #%04ld: (%4d, %4d)", i,cell_index[j].x, cell_index[j].y);
      }
      ss << "\r\n";
      bigss << ss.rdbuf();
    }
    
    for (auto cell = cell_index.cbegin(); cell != cell_index.cend() - 1; cell++) {
      m_cell_log_odd.at<double>(cell->x, cell->y) += m_log_odd_free;
    }
    m_cell_log_odd.at<double>(cell_index.back().x, cell_index.back().y) += m_log_odd_occ;
  }
  ROS_DEBUG_STREAM_ONCE("\r\n" << bigss.rdbuf());
}

std::vector<IntPoint> MapGridProjector::raycast(const Eigen::Vector2i p1, const Eigen::Vector2i p2) {
  int x0 = p1.x(), y0 = p1.y();
  int x1 = p2.x(), y1 = p2.y();

  const int dx =  std::abs(x1-x0), sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1-y0), sy = y0 < y1 ? 1 : -1; 
  int err = dx + dy, e2; /* error value e_xy */

  std::vector<IntPoint> pixels;

  while (!(x0==x1 && y0==y1)) {
    e2 = err << 1;
    if (e2 >= dy) { err += dy; x0 += sx; } /* e_xy+e_x > 0 */
    if (e2 <= dx) { err += dx; y0 += sy; } /* e_xy+e_y < 0 */

    pixels.emplace_back(IntPoint(x0, y0));
  }

  return pixels;
}
