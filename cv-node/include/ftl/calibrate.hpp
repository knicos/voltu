#ifndef _FTL_CALIBRATION_HPP_
#define _FTL_CALIBRATION_HPP_

#include <opencv2/opencv.hpp>
#include <ftl/local.hpp>
#include <string>

namespace ftl {
class Calibrate {
	public:
	Calibrate(ftl::LocalSource *s, const std::string &cal);
	
	bool recalibrate(const std::string &conf);
	
	bool undistort(cv::Mat &l, cv::Mat &r);
	bool rectified(cv::Mat &l, cv::Mat &r);
	
	bool isCalibrated();
	
	private:
	bool runCalibration(cv::Mat &img, cv::Mat &cam);
	
	private:
	ftl::LocalSource *local_;
};
};

#endif // _FTL_CALIBRATION_HPP_

