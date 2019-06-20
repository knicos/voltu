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

	void clear();
	void clearCorrespondances();

	bool capture(cv::Mat &rgb1, cv::Mat &rgb2);

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
	void getFeatures3D(std::vector<Eigen::Vector4d> &f);
	void getTransformedFeatures(std::vector<Eigen::Vector4d> &f);
	void getTransformedFeatures2D(std::vector<Eigen::Vector2i> &f);

	void setPoints(const std::vector<std::tuple<int,int,int,int>> &p) { log_ = p; }

	/**
	 * Calculate a transform using current set of correspondances.
	 * 
	 * @return Validation score of the transform.
	 */
	double estimateTransform(Eigen::Matrix4d &);
	double estimateTransform(Eigen::Matrix4d &T, const std::vector<int> &src_feat, const std::vector<int> &targ_feat);

	double findBestSubset(Eigen::Matrix4d &tr, int K, int N);

	double icp();
	double icp(const Eigen::Matrix4d &T_in, Eigen::Matrix4d &T_out, const std::vector<int> &idx);

	void convertToPCL(const std::vector<std::tuple<int,int,int,int>> &p, std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &p2);

	/**
	 * Get the estimated transform. This includes any parent transforms to make
	 * it relative to root camera.
	 */
	Eigen::Matrix4d transform();

	void setTransform(Eigen::Matrix4d &t) { uptodate_ = true; transform_ = t; }

	private:
	Correspondances *parent_;
	ftl::rgbd::Source *targ_;
	ftl::rgbd::Source *src_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr targ_cloud_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_;
	Eigen::Matrix4d transform_;
	bool uptodate_;
	std::vector<std::tuple<int,int,int,int>> log_;
	cv::Mat src_index_;
	cv::Mat targ_index_;
	std::vector<int> targ_feat_;
	std::vector<int> src_feat_;
};

}
}

#endif  // _FTL_REG_CORRESPONDANCES_HPP_
