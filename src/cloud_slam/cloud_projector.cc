
#include <queue>
#include <array>
#include <cmath>
#include <list>
#include <utility>
#include <iostream>
#include <iomanip>

#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>

#include <cloud_slam/cloud_projector.hpp>

#define __attribute_aligned__(x) __attribute__((aligned(x)))

void dump_vector(__m256 vec) {
  float val[8];
  memcpy(val, &vec, sizeof(val));
  std::printf("v8: %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f\n", val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
}

std::ostream &operator<<(std::ostream &os, std::vector<float> &v) {
  for (auto _v : v) {
    os << std::setw(8) << os.precision(4) << std::fixed << _v;
  }
  os << "\r\n";
  return os;
}
std::ostream &operator<<(std::ostream &os, std::vector<std::int32_t> &v) {
  for (auto _v : v) {
    os << std::setw(6) << _v;
  }
  os << "\r\n";
  return os;
}

namespace magni_octonav {

void CloudProjector::init_params(ros::NodeHandle &_nh, ros::NodeHandle &_pnh) {
  this->nh = _nh;
  this->private_nh = _pnh;

  this->base_frame_id_ = private_nh.param<std::string>("base_frame", "base_link");
  this->sensor_frame_id_ = private_nh.param<std::string>("sensor_frame", "velodyne");

  this->angle_resolution = private_nh.param<double>("angle_resolution", 0.00347435014745802414);  // in radian
  this->beam_resolution = private_nh.param<double>("beam_resolution", 0.03490658503988659154);    // in radian
  this->channel_count_ = (unsigned)private_nh.param<int>("channel_size", 16);                     // default VLP-16

  this->laser_count_ = static_cast<unsigned>(std::floor(2 * M_PI / this->angle_resolution));

  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/velodyne_points", 1));
  tf_cloud_msg_filter_.reset(new tf::MessageFilter<sensor_msgs::PointCloud2>(*cloud_sub_, tf_listener_, base_frame_id_, 16, nh));
  msg_connection = tf_cloud_msg_filter_->registerCallback(boost::bind(&CloudProjector::cloud_handler, this, _1));

  filter_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1, true);
  outlier_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("outlier_cloud", 1, true);
}

void CloudProjector::cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  if (!ros::ok()) {
    return;
  }

  pcl::PointWithRange pointR;
  pointR.range = -1.f;

  sensor_msgs::PointCloud2Ptr world_cloud_msg(new sensor_msgs::PointCloud2());
  pcl::PointCloud<PointXYZRing>::Ptr input_pc(new pcl::PointCloud<PointXYZRing>());

  tf::StampedTransform laser_to_base_tf;
  try {
    this->tf_listener_.lookupTransform(this->base_frame_id_, this->sensor_frame_id_, ros::Time(0), laser_to_base_tf);
  } catch (const tf::LookupException &ex) {
    ROS_WARN_STREAM_THROTTLE(1, "tf::LookupException: " << ex.what());
    return;
  } catch (const tf::ExtrapolationException &ex) {
    ROS_WARN_STREAM_THROTTLE(1, "tf::ExtrapolationException: " << ex.what());
    return;
  }

  pcl_ros::transformPointCloud(this->base_frame_id_, laser_to_base_tf, *cloud_msg, *world_cloud_msg);
  pcl::fromROSMsg(*world_cloud_msg, *input_pc);

  pcl::PointCloud<pcl::PointWithRange>::Ptr cloud_sorted(new pcl::PointCloud<pcl::PointWithRange>(laser_count_, channel_count_, pcl::PointWithRange(pointR)));
  pcl::PointCloud<PointXYZRLCI>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZRLCI>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier(new pcl::PointCloud<pcl::PointXYZ>());

  // first sort the point cloud to an array like ordering
  sortInputCloud(input_pc, cloud_sorted);

  std::vector<std::int32_t> labels;
  labelSegment(cloud_sorted, labels);

  std::vector<float> curvature;
  // compute smoothness of a point
  computeSmoothness(cloud_sorted, curvature);

  const size_t c_size = cloud_sorted->size();
  assert(labels.size() == c_size && c_size == curvature.size());

  for (size_t index = 0; index < cloud_sorted->size(); index++) {
    pcl::PointWithRange & p = cloud_sorted->at(index);
    // check valid point
    bool isValid = p.range > 0.05f && curvature.at(index) > -1.f && labels.at(index) != -1;
    if (!isValid) {
      continue;
    }
    PointXYZRLCI point;
    point.getVector4fMap() = cloud_sorted->at(index).getVector4fMap();
    point.index = static_cast<std::int32_t>(index);
    point.range = cloud_sorted->at(index).range;
    point.curvature = curvature.at(index);
    point.label = labels.at(index);
    if (labels.at(index) != INT32_MAX) {
      cloud_filtered->push_back(point);
    } else {
      pcl::PointXYZ p;
      p.getVector4fMap() = point.getVector4fMap();
      cloud_outlier->push_back(p);
    }
  }

  if (this->filter_cloud_pub_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_filtered, *cloud_msg);

    cloud_msg->header.frame_id = this->base_frame_id_;
    cloud_msg->header.stamp = ros::Time::now();

    this->filter_cloud_pub_.publish(cloud_msg);
  }

  if (this->outlier_cloud_pub_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_outlier, *cloud_msg);

    cloud_msg->header.frame_id = this->base_frame_id_;
    cloud_msg->header.stamp = ros::Time::now();

    this->outlier_cloud_pub_.publish(cloud_msg);
  }
}

void CloudProjector::sortInputCloud(const pcl::PointCloud<PointXYZRing>::ConstPtr &cloud, pcl::PointCloud<pcl::PointWithRange>::Ptr &_sorted_cloud) {
  _sorted_cloud->resize(laser_count_ * channel_count_);
  _sorted_cloud->height = channel_count_;
  _sorted_cloud->width = laser_count_;
  _sorted_cloud->is_dense = false;
  
  // used for SIMD
  const v8sf angle_res_vec = _mm256_set1_ps(this->angle_resolution);
  // [v8, y8, z8] ordering
  float vec_xyz[24] __attribute_aligned__(32) = {0};

  const size_t batch_size = cloud->size() >> 3ul;
  for (size_t i = 0; i < batch_size; i++) {  // index `i` is in multiple of 8
    // unpack values
    for (size_t j = 0; j < 8; j++) {
      vec_xyz[j] = cloud->at(8 * i + j).x;
      vec_xyz[8 + j] = cloud->at(8 * i + j).y;
      vec_xyz[16 + j] = cloud->at(8 * i + j).z;
    }

    // copy to vector
    v8sf x = _mm256_load_ps(&vec_xyz[0]);
    v8sf y = _mm256_load_ps(&vec_xyz[8]);
    v8sf z = _mm256_load_ps(&vec_xyz[16]);
    v8sf dist_vec = this->hypot_avx(x, y, z);

    /* Value are in range of [-pi, pi]
     * But we map to [0, 2pi] for indexing */
    v8sf theta = M_PIf32 - atan2_fast(y, x);
    v8sf column_index_vec = _mm256_round_ps((theta / angle_res_vec), _MM_FROUND_CUR_DIRECTION);

    float dist[8] __attribute_aligned__(32) = {0};
    float ci[8] __attribute_aligned__(32) = {0};
    _mm256_store_ps(dist, dist_vec);
    _mm256_store_ps(ci, column_index_vec);

    // unpack vector back to vector
    for (size_t k = 0; k < 8; k++) {
      pcl::PointWithRange pr;
      pr.getVector3fMap() = cloud->at(8 * i + k).getVector3fMap();
      pr.range = dist[k];
      // r = hori_size_cout * ring_id
      unsigned r_idx = this->laser_count_ * static_cast<unsigned>(cloud->at(8 * i + k).ring);
      // h = point_hori_index
      unsigned h_idx = static_cast<unsigned>(ci[k]) % this->laser_count_;
      _sorted_cloud->at(r_idx + h_idx) = pr;
    }
  }

  // do the remaining
  for (size_t i = batch_size << 3ul; i < cloud->size(); i++) {
    PointXYZRing pt = cloud->at(i);
    float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    double theta = M_PIf64 - atan2_fast(pt.y, pt.x);
    unsigned col_idx = static_cast<unsigned>(std::round(theta / this->angle_resolution)) % this->laser_count_;
    unsigned r_idx = this->laser_count_ * cloud->at(i).ring;
    pcl::PointWithRange pr;
    pr.getVector4fMap() = pt.getVector4fMap();
    pr.range = dist;
    _sorted_cloud->at(r_idx + col_idx) = pr;
  }
}

void CloudProjector::labelSegment(const pcl::PointCloud<pcl::PointWithRange>::ConstPtr &_scloud, std::vector<std::int32_t> &_label) {
  /**
   * By default all points have label of -1
   * 0 means that this point does not associate to any patch
   * -1 means this element cannot be evaluated
   * 1 means this point is a ground point
   * >1 means this point belongs to a patch
   */
  _label.resize(laser_count_ * channel_count_, 0);

  // mark ground segments
  for (size_t i = 0; i < 7; i++) {
    for (size_t j = 0; j < this->laser_count_; j++) {
      size_t curr_idx = this->laser_count_ * i + j;
      size_t upper_idx = this->laser_count_ * (i + 1) + j;

      pcl::PointWithRange lower_p = _scloud->at(curr_idx);   // point in question
      pcl::PointWithRange upper_p = _scloud->at(upper_idx);  // point above

      bool valid = lower_p.range > 0.01f && upper_p.range > 0.01f;
      if (!valid) {
        _label.at(curr_idx) = -1;
        continue;
      }

      double diff_x = upper_p.x - lower_p.x;
      double diff_y = upper_p.y - lower_p.y;
      double diff_z = upper_p.z - lower_p.z;

      double angle = this->atan2_fast(diff_z, std::hypot(diff_x, diff_y)) * 180.0 / M_PI;
      if (angle <= 10.f) {
        _label.at(curr_idx) = 1;
        _label.at(upper_idx) = 1;
      }
    }
  }

  // mark other segments with label
  labelComponent_bfs(_scloud, _label);

  return;
}

void CloudProjector::computeSmoothness(const pcl::PointCloud<pcl::PointWithRange>::ConstPtr &_scloud, std::vector<float> &_curvature) {
  const long col_size = static_cast<long>(laser_count_);
  const long row_size = static_cast<long>(channel_count_);

  auto warp_index = [](const long &i, const size_t &n) { return (i + n) % n; };
  _curvature.resize(laser_count_ * channel_count_, -1.0f);

  for (long i = 0; i < row_size; i++) {
    for (long j = 0; j < col_size; j++) {
      if (_scloud->at(i * col_size + j).range < 0.01f) {
        continue;
      }
      int point_count = 0;
      float cv = 0.0f;
      for (long k = j - 5; k <= j + 5; k++) {
        float r = _scloud->at(i * col_size + warp_index(k, col_size)).range;
        if (k == j || r < 0.01f) continue;
        cv += r;
        point_count++;
      }
      cv -= -point_count * _scloud->at(i * col_size + j).range;
      _curvature.at(i * col_size + j) = (cv * cv) / (point_count * _scloud->at(i * col_size + j).range);
    }
  }
}

void CloudProjector::labelComponent_bfs(const pcl::PointCloud<pcl::PointWithRange>::ConstPtr &_scloud, std::vector<int> &_label) {
  struct Index {
    int i, j;
    Index() {}
    Index(const Index &o) : i(o.i), j(o.j) {}
    Index(int ii, int jj) : i(ii), j(jj) {}
  };

  const std::array<Index, 4> nbhr4 = {Index(-1, 0), Index(1, 0), Index(0, 1), Index(0, -1)};

  std::list<std::list<Index>> clusters;

  const size_t width = laser_count_;
  const size_t height = channel_count_;
  int bfs_label = 2;

  const double tang = DEG2RAD(45.0);

  for (size_t row = 0; row < height; row++) {
    for (size_t col = 0; col < width; col++) {
      if (_label.at(row * width + col) != 0) {
        continue;
      } 
      std::queue<Index, std::list<Index>> search_index;
      // patch of points marked by this iteration of bfs
      std::list<Index> patch;
      search_index.push(Index(row, col));
      patch.push_back(Index(row,col));

      while (!search_index.empty()) {
        Index curr = search_index.front();
        search_index.pop();
        size_t thisIdx = curr.i * width + curr.j;

        _label.at(thisIdx) = bfs_label;

        for (auto it = nbhr4.begin(); it != nbhr4.end(); it++) {
          Index next(it->i + curr.i, it->j + curr.j);
          if (next.i >= (int)height || next.i < 0) {
            continue;
          }

          next.j %= width;
          if (next.j < 0) {
            next.j += width;
          }
          size_t nextIdx = next.i * width + next.j;
          // skip examined points
          if (_label.at(nextIdx) != 0) {
            continue;
          }

          double alpha = it->i == 0 ? this->angle_resolution : this->beam_resolution;

          float d1 = std::max(_scloud->at(thisIdx).range, _scloud->at(nextIdx).range);
          float d2 = std::min(_scloud->at(thisIdx).range, _scloud->at(nextIdx).range);

          double diff = atan2_fast(d2 * std::sin(alpha), (d1 - d2 * std::cos(alpha)));

          if (diff >= tang) {
            search_index.push(next);
            patch.push_back(next);
            _label.at(nextIdx) = bfs_label;
          }
        }
      }
      // end of BFS
      bfs_label++;
      clusters.push_back(patch);
    }
  }

  // check patch validity
  for (auto cl_iter = clusters.begin(); cl_iter != clusters.end(); cl_iter++) {
    bool validPatch = false;
    if (cl_iter->size() >= 30ul) {
      validPatch = true;
    } else if (cl_iter->size() >= 5ul) {
      std::set<size_t> s;
      for (const auto &c : *cl_iter) s.insert(c.i);
      if (s.size() >= 3) {
        validPatch = true;
      }
    }

    if (!validPatch) {
      // remove this patch of points
      for (auto patch = cl_iter->begin(); patch != cl_iter->end(); patch++) {
        size_t index = patch->i * width + patch->j;
        _label.at(index) = INT32_MAX;
      }
    }
  }
}

__attribute_noinline__ double CloudProjector::atan2_fast(const double &_y, const double &_x) {
  const double pi = M_PI;
  const double pi_2 = M_PI_2;

  /* For fast approximation, the input is limited to [-1, +1]
   *
   * @note \n
   * atan(y/x) + atan(x/y) =  pi/2 if x/y >=0;
   * atan(y/x) + atan(x/y) = -pi/2 if x/y <0
   */
  const bool swap = std::fabs(_x) < fabs(_y);
  // make denominator > nominator such that approxmiation holds
  double atan_input = swap ? _x / _y : _y / _x;
  double results = atan_approx(atan_input);

  // adjust atan output from above swap
  results = swap ? std::copysign(pi_2, atan_input) - results : results;
  // 2nd quadrant and 3rd quadrant
  results = _x < 0.0 ? std::copysign(pi, _y) + results : results;
  return results;
}

__attribute_noinline__ v8sf CloudProjector::atan2_fast(const v8sf &_y, const v8sf &_x) {
  // load some constants as vector
  const v8sf pi_vec = _mm256_set1_ps(M_PIf32);
  const v8sf pi_2_vec = _mm256_set1_ps(M_PI_2f32);

  // setup vector for sign extraction (sign located at last bit)
  const v8sf sign_mask = _mm256_castsi256_ps(_mm256_set1_epi32(0x80'0000'00));

  // explanation see the function above

  const v8sf _xi = _mm256_andnot_ps(sign_mask, _x);
  const v8sf _yi = _mm256_andnot_ps(sign_mask, _y);
  const v8sf swap = _mm256_cmp_ps(_yi,  // |y|
                                  _xi,  // |x|
                                  _CMP_GT_OS);

  // v8sf atan_input = swap ? _xi / _yi : _yi / _xi;
  v8sf atan_input = _mm256_div_ps(
      _mm256_blendv_ps(_y, _x, swap), // pick the lowest between |y| and |x| for each number
      _mm256_blendv_ps(_x, _y, swap)  // and the highest.
    );
  // approximate atan
  v8sf results = atan_approx(atan_input);

  // std::copysignf but in vector of 8 float
  v8sf cpsign_ps = _mm256_or_ps(pi_2_vec, _mm256_and_ps(atan_input, sign_mask));
  v8sf temp = cpsign_ps - results;

  // apply results columnwise
  results = _mm256_blendv_ps(results, temp, swap);

  const v8sf x_sign_mask = _mm256_castsi256_ps(_mm256_srai_epi32(_mm256_castps_si256(_x), 31));
  // use the mask to perform the adjustment only when the sign
  // if positive, and use the sign bit of `y` to know whether to add
  // `pi` or `-pi`
  cpsign_ps = _mm256_xor_ps(pi_vec, _mm256_and_ps(sign_mask, _y));
  results = _mm256_and_ps(cpsign_ps, x_sign_mask) + results;

  return results;
}

}  // namespace magni_octonav