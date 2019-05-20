#ifndef _FTL_RECONSTRUCT_REGISTRATION_HPP_
#define _FTL_RECONSTRUCT_REGISTRATION_HPP_

#include <ftl/config.h>
#include <opencv2/opencv.hpp>

#ifdef HAVE_PCL

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>

namespace ftl {
namespace registration {

/* Find transformation matrix for transforming clouds_source to clouds_target
 * Assumes that corresponding points in clouds_source[i] and clouds_target[i] have same indices
 */
Eigen::Matrix4f findTransformation(	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_source,
									std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_target);


/* Convert chessboard corners found with OpenCV's findChessboardCorners to PCL point cloud.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr cornersToPointCloud(const std::vector<cv::Point2f> &corners, const cv::Mat &disp, const cv::Mat &Q);

/* Find chessboard corners from image.
 */
bool findChessboardCorners(cv::Mat &rgb, const cv::Mat &disp, const cv::Mat &Q, const cv::Size pattern_size, pcl::PointCloud<pcl::PointXYZ>::Ptr &out);

};
};

#endif  // HAVE_PCL
#endif  // _FTL_RECONSTRUCT_REGISTRATION_HPP_
