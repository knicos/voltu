#ifndef _FTL_REGISTRATION_SFM_HPP_
#define _FTL_REGISTRATION_SFM_HPP_

#include <ftl/configurable.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <tuple>

namespace ftl {
namespace registration {

/**
 * @brief	Chessboard calibration pattern. Uses OpenCV's
 * 			findChessboardCornersSB function.
 * @todo	Parameters hardcoded in constructor
 *
 * All parameters:
 * 	- pattern size (inner corners)
 * 	- square size, millimeters (TODO: meters)
 * 	- image size, pixels
 * 	- flags, see ChessboardCornersSB documentation
 */
class CalibrationChessboard {
public:
	CalibrationChessboard(ftl::Configurable *root);
	void objectPoints(std::vector<cv::Vec3f> &out);
	bool findPoints(cv::Mat &in, std::vector<cv::Vec2f> &out);
	void drawPoints(cv::Mat &img, const std::vector<cv::Vec2f> &points);

private:
	int chessboard_flags_ = 0;
	float pattern_square_size_;
	cv::Size pattern_size_;
	cv::Size image_size_;
};

bool featuresSIFT(cv::Mat &frame1, cv::Mat &frame2, std::vector<std::tuple<int,int,int,int>> &points, int);

bool featuresChess(cv::Mat &frame1, cv::Mat &frame2, std::vector<std::tuple<int,int,int,int>> &points);

}
}

#endif  // _FTL_REGISTRATION_SFM_HPP_
