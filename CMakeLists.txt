cmake_minimum_required(VERSION 3.0.2)
project(magni_octonav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs
  map_msgs
  move_base
  move_base_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  visualization_msgs
  
  pcl_ros
  eigen_conversions
  pcl_conversions

)

# set (Boost_USE_MULTITHREADED ON)
# set (Boost_USE_STATIC_LIBS ON)
# set (Boost_USE_STATIC_RUNTIME OFF)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  nav_msgs roscpp rospy sensor_msgs std_msgs tf tf2 visualization_msgs pcl_ros eigen_conversions pcl_conversions
  DEPENDS PCL
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(SRC
  src/map_grid_projector.cc
)

catkin_install_python(PROGRAMS scripts/grid_mapper.py
				# scripts/message_handler.py
				# scripts/utils.py
				# scripts/grid_map.py
				# scripts/bresenham.py
				# scripts/rtime_gmapping_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add_executable(map_grid_projector src/map_grid_projector_node.cc ${SRC})
# target_link_libraries(map_grid_projector ${catkin_LIBRARIES} ${PCL_LIBRARIES} Boost::headers Boost::system)