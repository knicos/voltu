#pragma once
#ifndef _FTL_CALIBRATION_HPP_
#define _FTL_CALIBRATION_HPP_

#include <vector>

#include <opencv2/core/core.hpp>

namespace ftl {
namespace calibration {

namespace transform {

/**
 * @brief Get rotation matrix and translation vector from transformation matrix.
 * @param T    transformation matrix (4x4) or (3x4)
 * @param R    (out) rotation matrix (3x3)
 * @param t    (out) translation vector (3x1)
 */
inline void getRotationAndTranslation(const cv::Mat &T, cv::Mat &R, cv::Mat &t) {
	T(cv::Rect(0, 0, 3, 3)).copyTo(R);
	T(cv::Rect(3, 0, 1, 3)).copyTo(t);
}

/**
 * @brief Inverse transform inplace
 * @param R   rotation matrix (3x3)
 * @param t   translation vector (3x1)
 */
inline void inverse(cv::Mat &R, cv::Mat &t) {
	cv::Mat t_(3, 1, CV_64FC1);

	t_.at<double>(0) = -R.col(0).dot(t);
	t_.at<double>(1) = -R.col(1).dot(t);
	t_.at<double>(2) = -R.col(2).dot(t);

	cv::swap(t_, t);
	R = R.t();
}

/**
 * @brief Inverse transform inplace
 * @param T   transformation matrix (4x4)
 */
inline void inverse(cv::Mat &T) {
	cv::Mat rmat;
	cv::Mat tvec;
	getRotationAndTranslation(T, rmat, tvec);
	T = cv::Mat::eye(4, 4, CV_64FC1);

	T(cv::Rect(3, 0, 1, 1)) = -rmat.col(0).dot(tvec);
	T(cv::Rect(3, 1, 1, 1)) = -rmat.col(1).dot(tvec);
	T(cv::Rect(3, 2, 1, 1)) = -rmat.col(2).dot(tvec);
	T(cv::Rect(0, 0, 3, 3)) = rmat.t();
}

inline cv::Point3d apply(const cv::Point3d& point, const cv::InputArray& R, const cv::InputArray& t) {
	cv::Mat R_ = R.getMat();
	cv::Mat t_ = t.getMat();
	cv::Mat p_new = R_ * cv::Mat(point) + t_;
	return cv::Point3d(p_new.at<double>(0), p_new.at<double>(1), p_new.at<double>(2));
}

inline cv::Point3d apply(const cv::Point3d& point, const cv::InputArray& T) {
	cv::Mat T_ = T.getMat();
	cv::Mat rmat;
	cv::Mat tvec;
	getRotationAndTranslation(T_, rmat, tvec);
	return apply(point, rmat, tvec);
}

} // namespace transform

/**
 * @brief Calculate MSE reprojection error
 */
double reprojectionError(const std::vector<cv::Point2d>& points_im,
	const std::vector<cv::Point3d>& points, const cv::Mat& K, const cv::Mat& D,
	const cv::Mat& R, const cv::Mat& t);

/**
 * Same as OpenCV's recoverPose(), but does not assume same intrinsic paramters
 * for both cameras.
 *
 * @todo Write unit tests to check that intrinsic parameters work as expected.
 */
int recoverPose(const cv::Mat &E, const std::vector<cv::Point2d> &_points1,
	const std::vector<cv::Point2d> &_points2, const cv::Mat &_cameraMatrix1,
	const cv::Mat &_cameraMatrix2, cv::Mat &_R, cv::Mat &_t,
	double distanceThresh, cv::Mat &triangulatedPoints);

/**
 * Find camera rotation and translation from first to second camera. Uses
 * OpenCV's recoverPose() (with modifications) to estimate camera pose and
 * triangulate point locations. Scale is estimated from object_points. 8 point
 * algorithm (OpenCV) is used to estimate fundamental matrix at beginning.
 */
double computeExtrinsicParameters(const cv::Mat &K1, const cv::Mat &D1,
	const cv::Mat &K2, const cv::Mat &D2, const std::vector<cv::Point2d> &points1,
	const std::vector<cv::Point2d> &points2, const std::vector<cv::Point3d> &object_points,
	cv::Mat &R, cv::Mat &t, std::vector<cv::Point3d> &points_out);

}
}

#endif
