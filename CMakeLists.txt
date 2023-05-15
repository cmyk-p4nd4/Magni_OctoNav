cmake_minimum_required(VERSION 3.12)
project(magni_octonav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall -Warray-bounds -g -ggdb -march=native -mfpmath=sse -ffast-math)
add_compile_definitions(EIGEN_MAX_ALIGN_BYTES=32)

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
  tf_conversions
  pcl_conversions
  fast_gicp
  
  hdl_graph_slam
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenGR REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_message_files(
  FILES
  FloorCoeff.msg
  PointCloudByte.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    nav_msgs
    message_runtime
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
  DEPENDS PCL
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenGR_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(CLOUDMERGE_SRC_DIR src/cloud_merge)
set(CLOUDMERGE_SRC
	${CLOUDMERGE_SRC_DIR}/merge.cc
	${CLOUDMERGE_SRC_DIR}/cloud_preprocessor.cc
	${CLOUDMERGE_SRC_DIR}/gr_align.cc
	${CLOUDMERGE_SRC_DIR}/option_parser.cc
	${CLOUDMERGE_SRC_DIR}/utils.cc
)

# cloud merge
add_executable(cloud_merge ${CLOUDMERGE_SRC})
target_link_libraries(cloud_merge ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} Eigen3::Eigen gr::algo gr::utils gr::accel OpenMP::OpenMP_CXX)
target_include_directories(cloud_merge PRIVATE include/cloud_merge)

# map_grid_projector
add_executable(map_grid_projector src/map_grid_projector_node.cc src/map_grid_projector.cc)
target_link_libraries(map_grid_projector ${catkin_LIBRARIES} ${PCL_LIBRARIES} Boost::headers Boost::system ${OpenCV_LIBS})

# pointcloud_compressor
add_executable(pointcloud_compressor_node src/pointcloud_compressor.cc)
target_link_libraries(pointcloud_compressor_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pointcloud_compressor_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

# octree decompressor
add_executable(octree_decompress_node src/octree_decompress.cc)

set(CUSTOM_TARGET_FLAGS
  "-Wall;-Warray-bounds;-g;-ggdb;-O3;-ffast-math;-fPIC;-msse;-msse2;-mssse3;"
)
set_target_properties(
  octree_decompress_node 
  PROPERTIES 
  COMPILE_OPTIONS "${CUSTOM_TARGET_FLAGS}"
  COMPILE_DEFINITIONS "BOOST_ALL_NO_LIB"
)
target_link_libraries(octree_decompress_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(octree_decompress_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_library(fast_gicp_warp SHARED
  src/fast_gicp_warp/fast_gicp_warp.cc
)
target_link_libraries(fast_gicp_warp
  ${PCL_LIBRARIES}
)
if (OPENMP_FOUND)
    if (TARGET OpenMP::OpenMP_CXX)
        target_link_libraries(fast_gicp_warp OpenMP::OpenMP_CXX)
    endif ()
endif ()
target_include_directories(fast_gicp_warp PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_install_python(PROGRAMS 
  scripts/grid_mapper.py
  scripts/explore.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(SLAM_DIR
  src/cloud_slam
)

add_library(cloud_projector_nodelet 
  ${SLAM_DIR}/cloud_projector.cc
  ${SLAM_DIR}/cloud_projector_nodelet.cc
)
target_link_libraries(cloud_projector_nodelet
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

add_library(feature_extractor_nodelet 
  ${SLAM_DIR}/feature_extractor.cc
  ${SLAM_DIR}/feature_extractor_nodelet.cc
)
target_link_libraries(feature_extractor_nodelet
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)
add_dependencies(feature_extractor_nodelet ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(cloud_projector_node
  ${SLAM_DIR}/cloud_projector.cc
  ${SLAM_DIR}/cloud_projector_node.cc
)
target_link_libraries(cloud_projector_node
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

add_executable(feature_extractor_node
  ${SLAM_DIR}/feature_extractor.cc
  ${SLAM_DIR}/feature_extractor_node.cc
)
target_link_libraries(feature_extractor_node
  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)
add_dependencies(feature_extractor_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

# install libraries from this cmake as specified by nodelet
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
  cloud_projector_nodelet
  feature_extractor_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)