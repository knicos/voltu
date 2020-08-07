#pragma once

#include <opencv2/core.hpp>
#include <ftl/calibration/structures.hpp>

namespace ftl {
namespace calibration {

/** Stereo rectification parameters. Wrapper for cv::stereoRectify() */
struct StereoRectify {
	/** Calculate rectification parameters. c1 and c2 contain valid calibration
	 * (extrinsic parameters for translation from world to camera) */
	StereoRectify(const CalibrationData::Calibration &c1, const CalibrationData::Calibration& c2, const cv::Size size={0, 0}, double alpha=0.0, int flags=0);

	/** stereo pair baseline (same unit as extrinsic paramters) */
	double baseline() const;

	/** calculate maps (cv::remap()) for camera 1 */
	void map1(cv::Mat &m1, cv::Mat &m2, int format=CV_16SC2);
	/** calculate maps (cv::remap()) for camera 2 */
	void map2(cv::Mat &m1, cv::Mat &m2, int format=CV_16SC2);

	cv::Size size;
	/** unrectified params */
	cv::Mat K1;
	cv::Mat K2;
	cv::Mat distCoeffs1;
	cv::Mat distCoeffs2;

	/** 3x4 projection matrix for first camera */
	cv::Mat P1;
	/** 3x4 projection matrix for second camera */
	cv::Mat P2;
	/** rotation matrix for first camera (unrectified to rectified) */
	cv::Mat R1;
	/** rotation matrix for second camera (unrectified to rectified) */
	cv::Mat R2;
	/** disparity to depth matrix */
	cv::Mat Q;
	/** rotation from first camera to second camera (unrectified) */
	cv::Mat R;
	/** translation from first camera to second camera (unrectified) */
	cv::Mat t;
	/** largest ROI containing only valid pixels in rectified image for first camera */
	cv::Rect roi1;
	/** largest ROI containing only valid pixels in rectified image for second camera */
	cv::Rect roi2;
};

}
}