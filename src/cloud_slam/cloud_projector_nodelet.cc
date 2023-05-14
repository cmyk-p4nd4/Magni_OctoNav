#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <cloud_slam/cloud_projector.hpp>

namespace magni_octonav {

class CloudProjectorNodelet : public nodelet::Nodelet {
public:
  CloudProjectorNodelet() {}
  virtual ~CloudProjectorNodelet() {}

  virtual void onInit() {
    NODELET_INFO("CloudProjectorNodelet Running");
    node_.reset(new CloudProjector(getMTNodeHandle(), getMTPrivateNodeHandle()));
  }

private:
  boost::shared_ptr<CloudProjector> node_;
};

}  // namespace magni_octonav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(magni_octonav::CloudProjectorNodelet, nodelet::Nodelet)