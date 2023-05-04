#include <plane_detection.hpp>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/extract_indices.h>

#include <limits>
#include <numeric>

namespace PlaneDetection {

  [[maybe_unused]]
  static void calculateBBox(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud, pcl::PointXYZ & min, pcl::PointXYZ & max) {
    
    const float n_min(std::numeric_limits<float>::max()), n_max(std::numeric_limits<float>::min());

    float minX(n_min),minY(n_min),minZ(n_min);
    float maxX(n_max),maxY(n_max),maxZ(n_max);


    const std::size_t cloudSize = cloud->size();
    for (std::size_t i = 0; i < cloudSize; i++) {
			minX = (cloud->at(i).x < minX) ? cloud->at(i).x : minX;
			maxX = (cloud->at(i).x > minX) ? cloud->at(i).x : maxX;

			minY = (cloud->at(i).y < minY) ? cloud->at(i).y : minY;
			maxY = (cloud->at(i).y > minY) ? cloud->at(i).y : maxY;

			minZ = (cloud->at(i).z < minZ) ? cloud->at(i).z : minZ;
			maxX = (cloud->at(i).z > minZ) ? cloud->at(i).z : maxZ;
		}
  
    min = pcl::PointXYZ(minX,minY,minZ);
    max = pcl::PointXYZ(maxX,maxY,maxZ);
  }

	void detect(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud_pn,
							std::size_t min_support /* = 1000 */) {

    std::vector<Plane> planes;
    std::vector<int> remaining(cloud_pn->size());
    std::iota(remaining.begin(), remaining.end(), 0);

    pcl::SampleConsensusModelPlane<pcl::PointNormal>::Ptr model_p_ptr(
			new pcl::SampleConsensusModelPlane<pcl::PointNormal>(cloud_pn, true));

    pcl::RandomSampleConsensus<pcl::PointNormal> ransac(model_p_ptr);

    // int retries = 0;
    const std::size_t cloudSize = cloud_pn->size();

    do {

      model_p_ptr->setIndices(remaining);

      ransac.setDistanceThreshold(0.1);
      ransac.setNumberOfThreads(4);
      ransac.computeModel();

      Eigen::VectorXf coeff;
      std::vector<int> inliers;
      ransac.getInliers(inliers);
      ransac.getModelCoefficients(coeff);

      if (inliers.size() < min_support) {
        continue;
      }

    } while (remaining.size() > static_cast<std::size_t>(cloudSize * 0.20));


    
	}
}