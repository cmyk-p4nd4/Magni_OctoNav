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
  fast_gicp

)

# set (Boost_USE_MULTITHREADED ON)
# set (Boost_USE_STATIC_LIBS ON)
# set (Boost_USE_STATIC_RUNTIME OFF)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenGR REQUIRED)
find_package(OpenMP REQUIRED)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  nav_msgs roscpp rospy sensor_msgs std_msgs tf tf2 visualization_msgs pcl_ros eigen_conversions pcl_conversions fast_gicp
  DEPENDS PCL
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenGR_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)


set(OVERLAP_SRC
	# src/merge.cc
	src/cloud_preprocessor.cc
	src/gr_align.cc
	src/option_parser.cc
	src/plane_detection.cc
	src/utils.cc
  src/overlap4pcs.cc
)
set(SRC
  src/map_grid_projector.cc
  # src/
)



add_executable(map_grid_projector src/map_grid_projector_node.cc ${SRC})
target_link_libraries(map_grid_projector ${catkin_LIBRARIES} ${PCL_LIBRARIES} Boost::headers Boost::system)

catkin_install_python(PROGRAMS 
  scripts/grid_mapper.py
  scripts/explore.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(overlap_4pcs ${OVERLAP_SRC} )
target_link_libraries(overlap_4pcs ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} Eigen3::Eigen gr::algo gr::utils gr::accel OpenMP::OpenMP_CXX)