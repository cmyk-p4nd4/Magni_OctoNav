#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/registration.h>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/gicp_settings.hpp>
#include <fast_gicp/gicp/lsq_registration.hpp>


namespace fast_gicp {
/**
 * @brief Fast GICP algorithm optimized for multi threading with OpenMP
 */
template<typename PointSource, typename PointTarget>
class FastGICPWarp : public FastGICP<PointSource, PointTarget> {
public:
  using Scalar = float;
  using Matrix4 = typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4;

  using PointCloudSource = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudTarget;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

#if PCL_VERSION >= PCL_VERSION_CALC(1, 10, 0)
  using Ptr = pcl::shared_ptr<FastGICP<PointSource, PointTarget>>;
  using ConstPtr = pcl::shared_ptr<const FastGICP<PointSource, PointTarget>>;
#else
  using Ptr = boost::shared_ptr<FastGICP<PointSource, PointTarget>>;
  using ConstPtr = boost::shared_ptr<const FastGICP<PointSource, PointTarget>>;
#endif


protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::reg_name_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::target_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::corr_dist_threshold_;

	using fast_gicp::FastGICP<PointSource, PointTarget>::num_threads_;
	using fast_gicp::FastGICP<PointSource, PointTarget>::k_correspondences_;

	using fast_gicp::FastGICP<PointSource, PointTarget>::regularization_method_;
	using fast_gicp::FastGICP<PointSource, PointTarget>::source_kdtree_;
	using fast_gicp::FastGICP<PointSource, PointTarget>::target_kdtree_;

	using fast_gicp::FastGICP<PointSource, PointTarget>::mahalanobis_;
	using fast_gicp::FastGICP<PointSource, PointTarget>::correspondences_;
	using fast_gicp::FastGICP<PointSource, PointTarget>::source_covs_;
	using fast_gicp::FastGICP<PointSource, PointTarget>::target_covs_;

public:
  FastGICPWarp();
  virtual ~FastGICPWarp() override;

  // limit the optimization degrees of freedom
  // the order is [roll, pitch, yaw, x, y, z]
	void setFreedomConstraint(const Eigen::Matrix<int, 6, 1> & constraint);

protected:

  virtual double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) override;

//   virtual 

protected:

	// contraints applied when performing iterative local linearization
	Eigen::Matrix<int, 6, 1> constraint_;

};
}  // namespace fast_gicp