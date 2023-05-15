# Magni Octonav
This is the codebase of my research work during my master degree programme developed and used in ROS framework

# Content
This codebase provide both simulation environment and real hardware environment for the Magni Silver

# Requirements

This package requires the following dependencies

### Non-ROS Packages

- OpenMP
- PCL
- [g2o](https://github.com/RainerKuemmerle/g2o)
- [OpenGR](https://github.com/STORM-IRIT/OpenGR)
- OpenCV
- Eigen3

### ROS Packages

- pcl_ros
- move_base
- robot_localization
- velodyne_pointcloud
- interactive_marker_twist_server
- twist_mux
- spatio_temporal_voxel_layer
- [fast_gicp](https://github.com/SMRT-AIST/fast_gicp)
- [hdl_graph_slam_rt](https://github.com/cmyk-p4nd4/hdl_graph_slam_rt)
- [octomap_mapping](https://github.com/OctoMap/octomap_mapping)
- [magni_robot](https://github.com/UbiquityRobotics/magni_robot)

If you want to run this package under simulation environment then the following extra dependencies are required:
- gazebo_ros_pkgs
- diff_drive_controller
- velodyne_simulator

# Installation

ROS Dependencies all-in-one

```bash
# minimal install
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-libg2o ros-$ROS_DISTRO-navigation \
  ros-$ROS_DISTRO-velodyne-pointcloud ros-$ROS_DISTRO-interactive-marker-twist-server \
  ros-$ROS_DISTRO-twist-mux ros-$ROS_DISTRO-spatio-temporal-voxel-layer ros-$ROS_DISTRO-robot-localization \
  ros-$ROS_DISTRO-octomap-mapping

# extra dependencies for simulation
sudo apt-get install ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-diff-drive-controller \
  ros-$ROS_DISTRO-velodyne-simulator
```

Non-ROS Dependencies

```bash
sudo apt-get install libpcl-dev libeigen3-dev libspdlog-dev libsuitesparse-dev \
  qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5 libopencv-dev python3-opencv
```

## Building

```bash
# only for OpenGR
git clone https://github.com/STORM-IRIT/OpenGR --recursive
cd OpenGR && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make install -j6

# in another terminal 
cd ~/catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git --recursive
git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive
git clone https://github.com/cmyk-p4nd4/hdl_graph_slam_rt --recursive
git clone https://github.com/cmyk-p4nd4/Magni_OctoNav --recursive
git clone https://github.com/UbiquityRobotics/magni_robot --recursive
rosdep install --from-paths . --ignore-src -r -y

# for catkin_make users
cd .. && catkin_make --only-pkg-with-deps -DCMAKE_BUILD_TYPE=Release magni_octonav

# for catkin tools users
cd .. && catkin build magni_octonav --cmake-args -DCMAKE_BUILD_TYPE=Release --
```

# Launching 
## Simulation

Starting up the simulation environment 
```bash
scripts/gzkiller.py roslaunch magni_octonav magni_world.launch
```
Note Gazebo 9 does not automatically terminate its gzclient and gzserver when it is invoked by roslaunch. Therefore the prefix script `scripts/gzkiller.py` is needed to correctly terminate gzclient and gzserver when Ctrl+C is pressed. 
If you are using Gazebo 11, you can ignore this and just run roslaunch normally

## Main features

Depends on your environemnt. You can either launch
```bash
roslaunch magni_octonav magni_navigation_3d_sim.launch # Simulation environment
```
or 
```bash
roslaunch magni_octonav magni_navigation_3d.launch # on the actual robot
```