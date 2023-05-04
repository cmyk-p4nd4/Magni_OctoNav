#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <vector>

class Plane {
public:

  // template <class InputIt>
  Plane(const std::vector<int> && other) : _pointIndices(other){}

  std::vector<int> _pointIndices;
  Eigen::VectorXf _coeff;
};

namespace PlaneDetection {
  void detect(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud_pn,
  int min_support = 1000);
}
