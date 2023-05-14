#include <fast_gicp_warp/fast_gicp_warp.hpp>

#include <fast_gicp/so3/so3.hpp>

namespace fast_gicp {

template<typename PointSource, typename PointTarget>
FastGICPWarp<PointSource, PointTarget>::FastGICPWarp() {
#ifdef _OPENMP
  num_threads_ = omp_get_max_threads();
#else
  num_threads_ = 1;
#endif

  k_correspondences_ = 20;
  reg_name_ = "FastGICPWarp";
  corr_dist_threshold_ = std::numeric_limits<float>::max();

  constraint_ << 0,0,0,0,0,0;

  regularization_method_ = RegularizationMethod::FROBENIUS;
  source_kdtree_.reset(new pcl::search::KdTree<PointSource>);
  target_kdtree_.reset(new pcl::search::KdTree<PointTarget>);
}

template<typename PointSource, typename PointTarget>
FastGICPWarp<PointSource, PointTarget>::~FastGICPWarp() {}

template<typename PointSource, typename PointTarget>
void FastGICPWarp<PointSource, PointTarget>::setFreedomConstraint(const Eigen::Matrix<int, 6, 1>& constraint) {
  Eigen::Matrix<int, 6, 1> ones = Eigen::Matrix<int,6,1>::Ones();
  this->constraint_ = ones - constraint.cwiseQuotient(constraint);
}

template<typename PointSource, typename PointTarget>
double FastGICPWarp<PointSource, PointTarget>::linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) {
  this->update_correspondences(trans);

  double sum_errors = 0.0;
  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs(num_threads_);
  std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bs(num_threads_);
  for (int i = 0; i < num_threads_; i++) {
    Hs[i].setZero();
    bs[i].setZero();
  }

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
  for (int i = 0; i < (int)input_->size(); i++) {
    int target_index = correspondences_[i];
    if (target_index < 0) {
      continue;
    }

    const Eigen::Vector4d mean_A = input_->at(i).getVector4fMap().template cast<double>();

    const Eigen::Vector4d mean_B = target_->at(target_index).getVector4fMap().template cast<double>();

    const Eigen::Vector4d transed_mean_A = trans * mean_A;
    const Eigen::Vector4d error = mean_B - transed_mean_A;

    sum_errors += error.transpose() * mahalanobis_[i] * error;

    if (H == nullptr || b == nullptr) {
      continue;
    }

    Eigen::Matrix<double, 4, 6> dtdx0 = Eigen::Matrix<double, 4, 6>::Zero();
    dtdx0.block<3, 3>(0, 0) = skewd(transed_mean_A.head<3>());
    dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> jlossexp = dtdx0;

    Eigen::Matrix<double, 6, 1> factor = this->constraint_.cast<double>() * 3e8;

    Eigen::Matrix<double, 6, 6> Hi = jlossexp.transpose() * mahalanobis_[i] * jlossexp + Eigen::MatrixXd(factor.asDiagonal());
    Eigen::Matrix<double, 6, 1> bi = jlossexp.transpose() * mahalanobis_[i] * error;

    Hs[omp_get_thread_num()] += Hi;
    bs[omp_get_thread_num()] += bi;
  }

  if (H && b) {
    H->setZero();
    b->setZero();
    for (int i = 0; i < num_threads_; i++) {
      (*H) += Hs[i];
      (*b) += bs[i];
    }
  }

  return sum_errors;
}

}  // namespace fast_gicp

template class fast_gicp::FastGICPWarp<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::FastGICPWarp<pcl::PointXYZI, pcl::PointXYZI>;
template class fast_gicp::FastGICPWarp<pcl::PointNormal, pcl::PointNormal>;