#pragma once
#ifndef _FTL_CALIBRATION_HPP_
#define _FTL_CALIBRATION_HPP_

#include <vector>

#include <opencv2/core/core.hpp>

namespace ftl {
namespace calibration {

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
