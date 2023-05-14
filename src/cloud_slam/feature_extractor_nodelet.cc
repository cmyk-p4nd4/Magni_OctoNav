#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <cloud_slam/feature_extractor.hpp>

namespace magni_octonav {
class FeatureExtractorNodelet : public nodelet::Nodelet {
public:
  FeatureExtractorNodelet(){};
  virtual ~FeatureExtractorNodelet() {}

  virtual void onInit() {
		NODELET_INFO("FeatureExtractorNodelet running");
    _node.reset(new FeatureExtractor(getMTNodeHandle(), getMTPrivateNodeHandle()));
  }

private:
  boost::shared_ptr<FeatureExtractor> _node;
};

}  // namespace magni_octonav

PLUGINLIB_EXPORT_CLASS(magni_octonav::FeatureExtractorNodelet, nodelet::Nodelet);