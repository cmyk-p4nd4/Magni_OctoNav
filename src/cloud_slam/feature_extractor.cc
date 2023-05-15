#include <cloud_slam/feature_extractor.hpp>

#include <string>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <Eigen/Core>

#include <magni_octonav/FloorCoeff.h>

namespace magni_octonav {

void FeatureExtractor::init_params(ros::NodeHandle &_nh, ros::NodeHandle &_pnh) {
  this->nh = _nh;
  this->private_nh = _pnh;

  std::string cloud_topic = private_nh.param<std::string>("cloud_topic", "/cloud_projector_nodelet/segmented_cloud");

  odom_frame_id_ = private_nh.param<std::string>("odom_frame_id", "odom");
  odom_pub_topic_ = private_nh.param<std::string>("odom_pub_topic","odom");

  // The minimum tranlational distance and rotation angle between keyframes.
  // If this value is zero, frames are always compared with the previous frame
  keyframe_delta_trans = private_nh.param<double>("keyframe_delta_trans", 0.25);
  keyframe_delta_angle = private_nh.param<double>("keyframe_delta_angle", 0.15);
  keyframe_delta_time = private_nh.param<double>("keyframe_delta_time", 1.0);

  // Registration validation by thresholding
  transform_thresholding = private_nh.param<bool>("transform_thresholding", false);
  max_acceptable_trans = private_nh.param<double>("max_acceptable_trans", 1.0);
  max_acceptable_angle = private_nh.param<double>("max_acceptable_angle", 1.0);

  // Whether to publish scan_matching result to tf
  publish_transform = private_nh.param<bool>("publish_transform", true);

  // subscriber
  cloud_sub_ = this->nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 1, boost::bind(&FeatureExtractor::cloud_handler, this, _1));

  // publisher
  floor_coeff_pub_ = this->private_nh.advertise<magni_octonav::FloorCoeff>("floor_coeff", 1, true);
}

void FeatureExtractor::cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  // Check for PointCloud2 field 'label'
  bool found = false;
  for (size_t i = 0; i < cloud_msg->fields.size(); i++) {
    if (cloud_msg->fields[i].datatype == sensor_msgs::PointField::INT32) {
      if (cloud_msg->fields[i].name == "label") {
        found = true;
        break;
      }
    }
  }

  if (!found) {
    ROS_ERROR_THROTTLE(1, "FeatureExtractor: Field 'label' not presented!");
    return;
  }

  pcl::PointCloud<PointXYZRLCI>::Ptr input_pc(new pcl::PointCloud<PointXYZRLCI>());
  pcl::fromROSMsg(*cloud_msg, *input_pc);

  // obtain plane coeffs for plane
  boost::optional<Eigen::Vector4f> plane_coeff = this->floor_detection(input_pc);
	std::vector<float> coeff(4, std::numeric_limits<float>::quiet_NaN());
	if (!plane_coeff) {
		for (int i = 0;i < 4; i++) {
			coeff.at(i) = (*plane_coeff)[i];
		}
	}






  // publish all data

  if (this->floor_coeff_pub_.getNumSubscribers() > 0) {
    magni_octonav::FloorCoeff msg;
    msg.coeff = coeff;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = cloud_msg->header.frame_id;
    this->floor_coeff_pub_.publish(msg);
  }

}

boost::optional<Eigen::Vector4f> FeatureExtractor::floor_detection(const pcl::PointCloud<PointXYZRLCI>::ConstPtr &_seg_cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  for (auto it = _seg_cloud->begin(); it != _seg_cloud->end(); it++) {
    // only points with label = 1 are potential ground points
    if (it->label != 1) {
      continue;
    }

    pcl::PointXYZ point;
    point.getVector4fMap() = it->getVector4fMap();
    plane_cloud->push_back(point);
  }

  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(plane_cloud, true));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
  ransac.setNumberOfThreads(4);
  ransac.setDistanceThreshold(0.15);
  ransac.computeModel();

  Eigen::VectorXf coeff;
  ransac.getModelCoefficients(coeff);

  double incline = coeff.head<3>().dot(Eigen::Vector3f::UnitZ());
  if (std::abs(incline) < std::cos(15.0 * M_PI / 180.0)) {
		ROS_WARN("FeatureExtractor: Plane inline = %.3f. This is not a flat ground!", incline);
    // the plane is not flat enough
    return boost::none;
  }

	// make the normal upward
  if (coeff.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
    coeff *= -1.0f;
  }

  return Eigen::Vector4f(coeff);
}

void FeatureExtractor::ground_points_align(const pcl::PointCloud<pcl::PointXYZ> &_ground_points) {

}

}  // namespace magni_octonav