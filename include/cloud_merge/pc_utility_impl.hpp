#include <algorithm>

#include <pcl/filters/passthrough.h>

template <typename Point>
static inline pcl::PointXYZ getCenter(const typename pcl::PointCloud<Point>::ConstPtr& cloud) {

	pcl::PointXYZ center;

	auto x = std::minmax_element(
		cloud->begin(), cloud->end(), [](auto& p1, auto& p2) { return p1.x < p2.x; });
	auto y = std::minmax_element(
		cloud->begin(), cloud->end(), [](auto& p1, auto& p2) { return p1.y < p2.y; });
	auto z = std::minmax_element(
		cloud->begin(), cloud->end(), [](auto& p1, auto& p2) { return p1.z < p2.z; });

	center.x = (x.first->x + x.second->x) / 2.0;
	center.y = (y.first->y + y.second->y) / 2.0;
	center.z = (z.first->z + z.second->z) / 2.0;

	return center;
}

template <typename Point>
void translatePointCloudToOrigin(const typename pcl::PointCloud<Point>::Ptr& cloud) {

	pcl::PointXYZ origin = getCenter<Point>(cloud);

	std::transform(cloud->begin(), cloud->end(), cloud->begin(), [&origin](auto& p) {
		p.x -= origin.x;
		p.y -= origin.y;
		return p;
	});
}

template <typename Point>
void removeTooFarPoints(const typename pcl::PointCloud<Point>::ConstPtr& cloud,
												const typename pcl::PointCloud<Point>::Ptr& out) {
	typename pcl::PassThrough<Point> pass;
	std::vector<int> indices;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-0.1, 1.8);
	pass.setInputCloud(cloud);
	pass.filter(*out);
}