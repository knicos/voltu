#include <ftl/calibration/stereorectify.hpp>
#include <ftl/calibration/parameters.hpp>
#include <opencv2/calib3d.hpp>

namespace ftl {
namespace calibration {

// ==== StereoRectify ==========================================================

StereoRectify::StereoRectify(const CalibrationData::Calibration& c1, const CalibrationData::Calibration& c2, cv::Size sz, double alpha, int flags) {
	size = sz;

	if (size == cv::Size{0, 0}) {
		size = c1.intrinsic.resolution;
	}
	K1 = c1.intrinsic.matrix(size);
	K2 = c2.intrinsic.matrix(size);
	c1.intrinsic.distCoeffs.Mat().copyTo(distCoeffs1);
	c2.intrinsic.distCoeffs.Mat().copyTo(distCoeffs2);

	cv::Mat T1 = c1.extrinsic.matrix();
	cv::Mat T2 = c2.extrinsic.matrix();
	cv::Mat T = T2 * transform::inverse(T1);

	transform::getRotationAndTranslation(T, R, t);
	cv::stereoRectify(	K1, distCoeffs1, K2, distCoeffs2, size, R, t,
						R1, R2, P1, P2, Q, flags, alpha, size, &roi1, &roi2);
}

double StereoRectify::baseline() const {
	return cv::norm(t);
}

void StereoRectify::map1(cv::Mat &m1, cv::Mat &m2, int format) {
	cv::initUndistortRectifyMap(K1, distCoeffs1, R1, P1, size, format, m1, m2);
}

void StereoRectify::map2(cv::Mat &m1, cv::Mat &m2, int format) {
	cv::initUndistortRectifyMap(K2, distCoeffs2, R2, P2, size, format, m1, m2);
}

}
}