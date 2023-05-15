#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <immintrin.h>

#include <cloud_slam/cloud_type.hpp>

typedef __m256 v8sf;	// vector of 8 float (avx)
typedef __m256i v8si;	// vector of 8 int (avx)
typedef __m256d v4sd; // vector of 4 double (avx)

namespace magni_octonav {

class CloudProjector {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CloudProjector(ros::NodeHandle &_nh, ros::NodeHandle &_pnh) {
    init_params(_nh, _pnh);
  }
  ~CloudProjector() {
    msg_connection.disconnect();
    this->tf_cloud_msg_filter_.reset();
    this->cloud_sub_.reset();
  }

  void init_params(ros::NodeHandle &_nh, ros::NodeHandle &_pnh);

protected:
  void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  void sortInputCloud(const pcl::PointCloud<PointXYZRing>::ConstPtr &cloud, pcl::PointCloud<pcl::PointWithRange>::Ptr &_sorted_cloud);
  void labelSegment(const pcl::PointCloud<pcl::PointWithRange>::ConstPtr &_scloud, std::vector<int> &_label);
  void computeSmoothness(const pcl::PointCloud<pcl::PointWithRange>::ConstPtr &_scloud, std::vector<float> &_curvature);
  /**
   * @brief Perform Breath-first search on a cloud
   */
  void labelComponent_bfs(const pcl::PointCloud<pcl::PointWithRange>::ConstPtr &_scloud, std::vector<int> &_label);
  
  inline v8sf hypot_avx(v8sf _x, v8sf _y) {
    return this->hypot_avx(_x, _y, _mm256_set1_ps(0.0f));
  }

  inline v8sf hypot_avx(v8sf _x, v8sf _y, v8sf _z) {
    v8sf xx = _x * _x;
    v8sf yy = _y * _y;
    v8sf zz = _z * _z;

    v8sf sq_dist_vec = _mm256_add_ps(xx, _mm256_add_ps(yy, zz));
    return _mm256_sqrt_ps(sq_dist_vec);
  }

  inline double atan_approx(const double &_z) const {
    // Store the coefficients
    const double a1 = 0.9999993329;
    const double a3 = -0.3332985605;
    const double a5 = 0.1994653599;
    const double a7 = -0.1390853351;
    const double a9 = 0.0964200441;
    const double a11 = -0.0559098861;
    const double a13 = 0.0219612288;
    const double a15 = -0.0040540580;

    const double z_sq = _z * _z;
    return _z * (a1 + z_sq * (a3 + z_sq * (a5 + z_sq * (a7 + z_sq * (a9 + z_sq * (a11 + z_sq * (a13 + z_sq * a15)))))));
  }

  inline v8sf atan_approx(const v8sf &_z) const {
    // Store the coefficients vector
    const v8sf a1  = _mm256_set1_ps( 0.9999993329f);
    const v8sf a3  = _mm256_set1_ps(-0.3332985605f);
    const v8sf a5  = _mm256_set1_ps( 0.1994653599f);
    const v8sf a7  = _mm256_set1_ps(-0.1390853351f);
    const v8sf a9  = _mm256_set1_ps( 0.0964200441f);
    const v8sf a11 = _mm256_set1_ps(-0.0559098861f);
    const v8sf a13 = _mm256_set1_ps( 0.0219612288f);
    const v8sf a15 = _mm256_set1_ps(-0.0040540580f);

    // compute approxmiation by Horner's Method
    const v8sf z_sq = _z * _z;
    v8sf result;
    result =                               a15;
    result = _mm256_fmadd_ps(z_sq, result, a13);
    result = _mm256_fmadd_ps(z_sq, result, a11);
    result = _mm256_fmadd_ps(z_sq, result,  a9);
    result = _mm256_fmadd_ps(z_sq, result,  a7);
    result = _mm256_fmadd_ps(z_sq, result,  a5);
    result = _mm256_fmadd_ps(z_sq, result,  a3);
    result = _mm256_fmadd_ps(z_sq, result,  a1);
    result = _mm256_mul_ps(_z, result);
    return result;
  }

  __attribute_noinline__ double atan2_fast(const double &_y, const double &_x);

  __attribute_noinline__ v8sf atan2_fast(const v8sf &_y, const v8sf &_x);

private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  // subscribers
  std::unique_ptr<tf::MessageFilter<sensor_msgs::PointCloud2>> tf_cloud_msg_filter_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;

  message_filters::Connection msg_connection;

  // publishers
  ros::Publisher filter_cloud_pub_;
  ros::Publisher outlier_cloud_pub_;

  // transform listener
  tf::TransformListener tf_listener_;

  // frame ids
  std::string base_frame_id_;
  std::string sensor_frame_id_;

  // cloud dimension info
  unsigned channel_count_;
  unsigned laser_count_;

  double angle_resolution;
  double beam_resolution;

};

}  // namespace magni_octonav