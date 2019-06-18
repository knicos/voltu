#ifndef _FTL_REG_CORRESPONDANCES_HPP_
#define _FTL_REG_CORRESPONDANCES_HPP_

#include <ftl/rgbd/source.hpp>
#include <nlohmann/json.hpp>

#include <pcl/common/transforms.h>

#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/transformation_validation.h>
#include <pcl/registration/transformation_validation_euclidean.h>

#include <vector>
#include <string>
#include <map>
#include <tuple>

namespace ftl {
namespace registration {

/**
 * Manage the identified set of correspondances between two sources. There may
 * also be a parent correspondances object to support the chaining of
 * transforms.
 */
class Correspondances {
	public:
	Correspondances(ftl::rgbd::Source *src, ftl::rgbd::Source *targ);
	Correspondances(Correspondances *parent, ftl::rgbd::Source *src);

	ftl::rgbd::Source *source() { return src_; }
	ftl::rgbd::Source *target() { return targ_; }

	/**
	 * Add a new correspondance point. The point will only be added if there is
	 * a valid depth value at that location.
	 * 
	 * @param tx X-Coordinate in target image
	 * @param ty Y-Coordinate in target image
	 * @param sx X-Coordinate in source image
	 * @param sy Y-Coordinate in source image
	 * @return False if point was invalid and not added.
	 */
	bool add(int tx, int ty, int sx, int sy);

	void removeLast();

	const std::vector<std::tuple<int,int,int,int>> &screenPoints() const { return log_; }

	void setPoints(const std::vector<std::tuple<int,int,int,int>> &p) { log_ = p; }

	/**
	 * Calculate a transform using current set of correspondances.
	 * 
	 * @return Validation score of the transform.
	 */
	double estimateTransform();

	double findBest(Eigen::Matrix4f &tr, const std::vector<std::tuple<int,int,int,int>> &p, const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &p2, int K, int N);

	void convertToPCL(const std::vector<std::tuple<int,int,int,int>> &p, std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &p2);

	/**
	 * Get the estimated transform. This includes any parent transforms to make
	 * it relative to root camera.
	 */
	Eigen::Matrix4f transform();

	void setTransform(Eigen::Matrix4f &t) { uptodate_ = true; transform_ = t; }

	private:
	Correspondances *parent_;
	ftl::rgbd::Source *targ_;
	ftl::rgbd::Source *src_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr targ_cloud_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_;
	Eigen::Matrix4f transform_;
	bool uptodate_;
	std::vector<std::tuple<int,int,int,int>> log_;
};

}
}

#endif  // _FTL_REG_CORRESPONDANCES_HPP_
