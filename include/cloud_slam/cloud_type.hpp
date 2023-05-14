
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace magni_octonav {

/** \brief A point structure representing Euclidean xyz coordinates.
 * with additional beam channel number
 */
struct EIGEN_ALIGN16 PointXYZRing {
  PCL_ADD_POINT4D
	float intensity;
  std::uint16_t ring;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

/** \brief A point structure representing Euclidean xyz coordinates.
 * Together with range, label and curvature
 * and the index value of the point in the range image
 */
struct EIGEN_ALIGN16 PointXYZRLCI {
  PCL_ADD_POINT4D
  float range;
  float curvature;
  std::int32_t label;
  std::int32_t index;
  PCL_MAKE_ALIGNED_OPERATOR_NEW

  inline PointXYZRLCI(const PointXYZRLCI &p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    range = p.range;
    label = p.label;
    curvature = p.curvature;
    index = p.index;
  }
  inline PointXYZRLCI() {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    range = -1.0f;
    curvature = -1.0f;
    label = -1;
    index = -1;
  }
};

}  // namespace magni_octonav

POINT_CLOUD_REGISTER_POINT_STRUCT(magni_octonav::PointXYZRing, (float, x, x)(float, y, y)(float, z, z)(float, intensity,intensity)(std::uint16_t, ring, ring));
POINT_CLOUD_REGISTER_POINT_STRUCT(magni_octonav::PointXYZRLCI, (float, x, x)(float, y, y)(float, z, z)(float, range, range)(float, curvature, curvature)(std::int32_t, label, label)(std::int32_t, index, index));
