// SPDX-License-Identifier: BSD-2-Clause

#include <map_grid_projector.hpp>
#include <limits>
#include <algorithm>
#include <sstream>

#include <boost/bind.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>


#define quote(x) #x


MapGridProjector::MapGridProjector(const ros::NodeHandle &nh_,
                                   const ros::NodeHandle &private_nh_)
    : m_nh(nh_), m_pnh(private_nh_),
      m_map_point_sub(nullptr), m_laser_sub(nullptr), m_tf_scan_sub(nullptr),

      
      m_log_odd_free(ProbabilityToLogOdds(0.3)),
      m_log_odd_occ(ProbabilityToLogOdds(0.75)), m_log_odd_prior(0) {

  m_pnh.param<std::string>("world_frame", m_worldFrameId, "map");
  m_pnh.param<std::string>("base_frame", m_sensorFrameId, "velodyne");
  m_pnh.param<double>("min_scan_range", m_minScanRange, 0.0);
  m_pnh.param<double>("max_scan_range", m_maxScanRange, 100.0);
  m_pnh.param<double>("resolution", m_resolution, 0.05);

  m_map_point_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_nh, "cloud_in", 16);
  m_laser_sub = new message_filters::Subscriber<sensor_msgs::LaserScan>(m_nh, "scan", 16);
  m_tf_scan_sub = new tf::MessageFilter<sensor_msgs::LaserScan> (*m_laser_sub, tf_listener, m_worldFrameId, 16);
  m_tf_scan_sub->registerCallback(boost::bind(&MapGridProjector::scan_handler, this, boost::placeholders::_1));

  ROS_INFO("Subscribing Callbacks");

  m_map_grid_pub = m_nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 10, true);
}

MapGridProjector::~MapGridProjector() {
  DELETE_PTR_CHECK(m_tf_scan_sub);
  DELETE_PTR_CHECK(m_map_point_sub);
  DELETE_PTR_CHECK(m_laser_sub);
}

void MapGridProjector::scan_handler(const sensor_msgs::LaserScan::ConstPtr & scan_msg) {
  ROS_DEBUG("Entered Callback!");

  if (!tf_listener.waitForTransform(m_worldFrameId, m_sensorFrameId, scan_msg->header.stamp, ros::Duration(1.5))) {
    ROS_WARN_STREAM("Timed out waiting for transform from "
                    << m_worldFrameId << " to " << m_sensorFrameId << " at "
                    << scan_msg->header.stamp.toSec());
    return;
  }

  tf::StampedTransform sensorToWorldTf;
  try {
    tf_listener.lookupTransform(m_worldFrameId, scan_msg->header.frame_id, scan_msg->header.stamp, sensorToWorldTf);
  } catch (tf::LookupException& ex) {
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }

  tf::Pose identity(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0));

  Eigen::Matrix4f pose;
  pcl_ros::transformAsMatrix(sensorToWorldTf, pose);
  Eigen::Matrix3f pose2d;
  


  uint32_t scan_count = scan_msg->ranges.size();
  std::vector<Eigen::Vector3d> scan_points_;
  scan_points_.resize(scan_count);

  for (size_t i = 0; i < scan_msg->ranges.size(); i++) {

    double r = scan_msg->ranges.at(i);
    if (r > scan_msg->range_max || r < scan_msg->range_min) {
      continue;
    }

    double theta = scan_msg->angle_min + i * scan_msg->angle_increment;

    Eigen::Vector4d point = {r * cos(theta), r * sin(theta), 0, 0};

    Eigen::Vector4d laser_world = pose * point;
    
  }


  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

  ROS_INFO_STREAM_THROTTLE_NAMED(
      1, "MapGridProjector", std::endl << 
      sensorToWorld.format(Eigen::IOFormat(5, 0, ", ", "\n", "[", "]")));

  geometry_msgs::TransformStamped geoStamp;
  
  // pcl_ros::transformPointCloud()


  return;
}


// nav_msgs::OccupancyGrid::Ptr generate(const std::vector<KeyFrameSnapshot::Ptr>& keyframes, double resolution) const {
//   if(keyframes.empty()) {
//     std::cerr << "Keyframe is empty!" << std::endl;
//     return nullptr;
//   }

//   if (resolution <= 0.05) {
//     resolution = 0.05;
//   }

//   tf::StampedTransform sensorToWorldTf;
//   try {
//     tf_listener.lookupTransform("map", keyframes.back()->cloud->header.frame_id, ros::Time(0), sensorToWorldTf);
//   } catch (tf::LookupException& ex) {
//     ROS_ERROR("%s", ex.what());
//     return nullptr;
//   }
//   Eigen::Matrix4f sensorToWorld;
//   pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);


//   double minX = std::numeric_limits<double>::infinity();
//   double maxX = -minX;
//   double minY = std::numeric_limits<double>::infinity();
//   double maxY = -minY;
//   double minZ, maxZ;

//   pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
//   cloud->reserve(keyframes.front()->cloud->size() * keyframes.size());

//   for(const auto& keyframe : keyframes) {
//     Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>();
//     for(const auto& src_pt : keyframe->cloud->points) {
//       PointT dst_pt;
//       dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
//       dst_pt.intensity = src_pt.intensity;
//       cloud->push_back(dst_pt);
//     }
//   }

//   pcl::PassThrough<PointT> crop;
//   crop.setInputCloud(cloud);
//   crop.setFilterFieldName("z");
//   crop.setFilterLimits(-0.5, 1.2);

//   pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());  
//   crop.filter(*filtered);


//   minX = std::min_element(filtered->begin(), filtered->end(), [](PointT a, PointT b) {return a.x < b.x;})->x;
//   maxX = std::max_element(filtered->begin(), filtered->end(), [](PointT a, PointT b) {return a.x < b.x;})->x;

//   minY = std::min_element(filtered->begin(), filtered->end(), [](PointT a, PointT b) {return a.y < b.y;})->y;
//   maxY = std::max_element(filtered->begin(), filtered->end(), [](PointT a, PointT b) {return a.y < b.y;})->y;

//   // [min|max]X and [min|maxY] should have the absolute [min|max] value for the world frame


//   /*
//     (0, 0)
//     +------------> X (column)                   
//     |
//     |
//     |
//     |
//     V             x(cellSizeX - 1, cellSizeY - 1)
//     Y (Row)
//   */

//   uint32_t gridWidth = static_cast<uint32_t>(std::fabs(maxX - minX) / resolution) + 1; // column
//   uint32_t gridHeight = static_cast<uint32_t>(std::fabs(maxY - minY) / resolution) + 1; // Row
//   // uint32_t cellSizeX = static_cast<uint32_t>(std::fabs(maxX - minX) );
//   // uint32_t cellSizeY = static_cast<uint32_t>(std::fabs(maxY - minY) );
//   nav_msgs::MapMetaData meta;
//   meta.resolution = resolution;
//   meta.height = gridHeight;
//   meta.width = gridWidth; 
//   meta.origin = geometry_msgs::Pose();
//   ROS_INFO_THROTTLE_NAMED(10, "MapGridGenerator", "Map Size is: %dx%d\r\n", meta.width, meta.height);

//   /* set map origin to bottom left of the world frame such that middle is (0,0) in world
     
//     +---------------> X (column)                   
//     |
//     |
//     |       x (0,0)
//     |
//     |
//     V              x ((cellSizeX / 2) - 1, (cellSizeY /2) - 1)
//     Y (Row)
//   */
//   meta.origin.position.x = -0.5 * double(std::fabs(maxX + minX));
//   meta.origin.position.y = -0.5 * double(std::fabs(maxY + minY));

//   std::vector<float> log_odd_cell;
//   nav_msgs::OccupancyGridPtr map_grid(new nav_msgs::OccupancyGrid());
//   map_grid->info = meta;

//   map_grid->data.resize(gridHeight * gridWidth);
//   log_odd_cell.resize(gridHeight * gridWidth);
//   std::fill(map_grid->data.begin(), map_grid->data.end(), static_cast<int8_t>(-1));
//   std::fill(log_odd_cell.begin(), log_odd_cell.end(), 0.0f);

//   auto worldToGrid = [=, &meta](const Eigen::Vector2f & pose) {
//     int _x = std::lround((pose.x() - meta.origin.position.x) / resolution);
//     int _y = std::lround((pose.y() - meta.origin.position.y) / resolution);
//     return Eigen::Vector2i(_x, _y);
//   };

//   for(const auto& keyframe : keyframes) {
//     sensor_msgs::LaserScanConstPtr scan_data = keyframe->scan;
//     float angle_start = scan_data->angle_min;
//     // get the yaw component from keyframe pose
//     float theta_keyframe = Eigen::Quaternionf(keyframe->pose.rotation().cast<float>()).z();
//     Eigen::Vector2f robot_pose = Eigen::Vector2f(keyframe->pose.translation().cast<float>().topRows<2>());
//     Eigen::Vector2i robot_pose_grid = worldToGrid(robot_pose);

//     for (size_t i = 0; i < scan_data->ranges.size(); i++) {
//       float R = scan_data->ranges.at(i);
//       // skip out of bound value
//       if (R < scan_data->range_min || R > scan_data->range_max) continue;

//       float theta = angle_start + i * scan_data->angle_increment + theta_keyframe;
//       float lx = scan_data->ranges.at(i) * std::cos(theta);
//       float ly = scan_data->ranges.at(i) * std::sin(theta);

//       // translate to world frame
//       // extract (x,y) from keyframe pose and add scan (x,y)
//       Eigen::Vector2f laser_world = robot_pose + Eigen::Vector2f(lx, ly);
//       Eigen::Vector2i laser_world_grid = worldToGrid(laser_world);

//       std::vector<Eigen::Vector2i> cells = bresenham_line(robot_pose_grid, laser_world_grid);

//       // std::stringstream ss;

//       // iterate all cells
//       for (const auto &cell : cells) {
//         float _log = inverse_sensor_model(cell, laser_world_grid, resolution);
//         // log odd update
//         // log_odd_cell.at(cell.x() + map_grid->info.width * cell.y()) += _log;
//         // ss << boost::format("(%d, %d) ") % cell.x() % cell.y();
//       }
//       // ss << std::endl;
//       // ROS_INFO_STREAM_COND(i % 600 == 0, ss.rdbuf());
//     }
//   }

//   // convert ratio cells to map grid
//   auto ratioToMap = [this](float value) -> int8_t {
//     // not a recommanded practice
//     // but it does the work by now
//     if (std::fabs(value - _log_odd_prior) < 1e-5) {
//       return static_cast<int8_t>(-1);
//     }
//     if (value < this->_log_odd_free) {
//       return static_cast<int8_t>(0);
//     }
//     if (value > this->_log_odd_occ) {
//       return static_cast<int8_t>(100);
//     }
//     return static_cast<int8_t>(this->LogOddsToProbability(value) * 100.f);
//   };

//   std::transform(log_odd_cell.cbegin(), log_odd_cell.cend(), map_grid->data.begin(), ratioToMap);

//   return map_grid;
// }

std::vector<Eigen::Vector2i> MapGridProjector::bresenham_line(Eigen::Vector2i p1, Eigen::Vector2i p2) const {
	std::vector<Eigen::Vector2i> cell_xy;
	float m = 1.0 * (p2.y() - p1.y()) / (p2.x() - p2.x());
  bool flag = false;
  if(m > -1.0 || m < 1.0) {
    flag = true;
    // swap two values
    p1.x() ^= p1.y(); p2.x() ^= p2.y();
    p1.y() ^= p1.x(); p2.y() ^= p2.x();
    p1.x() ^= p1.y(); p2.x() ^= p2.y();
    // recompute slope
    m = 1.0 * (p2.y() - p1.y()) / (p2.x() - p2.x());
  }

  float delta = 0.5 - m;
  // swap two points since octant 3 - 6
  if(p1.x() > p2.x()) {
    p1.x() ^= p2.x() ^= p1.x() ^= p2.x();
    p1.y() ^= p2.y() ^= p1.y() ^= p2.y();
  }
  while(p1.x() != p2.x()) {
		if (m > 0 && delta < 0) {
			++p1.y(); ++delta;
		} else if (m < 0 && delta > 0) {
			--p1.y(); --delta;
		}
		delta -= m;
		++p1.x();
		if (flag) cell_xy.push_back(Eigen::Vector2i(p1.y(), p1.x()));
		else cell_xy.push_back(p1);
  }

  return cell_xy;
}