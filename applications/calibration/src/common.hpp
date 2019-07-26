#ifndef _FTL_CALIBRATION_COMMON_HPP_
#define _FTL_CALIBRATION_COMMON_HPP_

#include <map>
#include <string>

#include <opencv2/core.hpp>

namespace ftl {
namespace calibration {

std::string getOption(const std::map<std::string, std::string> &options, const std::string &opt);
bool hasOption(const std::map<std::string, std::string> &options, const std::string &opt);
int getOptionInt(const std::map<std::string, std::string> &options, const std::string &opt, int default_value);
double getOptionDouble(const std::map<std::string, std::string> &options, const std::string &opt, double default_value);
std::string getOptionString(const std::map<std::string, std::string> &options, const std::string &opt, std::string default_value);

bool loadIntrinsics(const std::string &ifile, std::vector<cv::Mat> &K, std::vector<cv::Mat> &D);
bool saveIntrinsics(const std::string &ofile, const std::vector<cv::Mat> &K, const std::vector<cv::Mat> &D);

// TODO loadExtrinsics()
bool saveExtrinsics(const std::string &ofile, cv::Mat &R, cv::Mat &T, cv::Mat &R1, cv::Mat &R2, cv::Mat &P1, cv::Mat &P2, cv::Mat &Q);

class Grid {
private:
	int rows_;
	int cols_;
	int width_;
	int height_;
	int cell_width_;
	int cell_height_;
	int offset_x_;
	int offset_y_;
	int visited_count_;

	std::vector<std::pair<cv::Point, cv::Point>> corners_;
	std::vector<bool> visited_;

public:
	Grid(int rows, int cols, int width, int height, int offset_x, int offset_y);
	void drawGrid(cv::Mat &rgb);
	int checkGrid(cv::Point p1, cv::Point p2);
	void updateGrid(int i);
	bool isVisited(int i);
	bool isComplete();
	void reset(); 
};

/**
 * @brief	Wrapper for OpenCV's calibration methods. Paramters depend on
 * 			implementation (different types of patterns).
 *
 * Calibration objects may store state; eg. from previous views of calibration
 * images.
 */
class Calibration {
public:
	/**
	 * @brief	Calculate reference points for given pattern
	 * @param	Output parameter
	 */
	void objectPoints(std::vector<cv::Vec3f> &out);

	/**
	 * @brief	Try to find calibration pattern in input image
	 * @param	Input image
	 * @param	Output parameter for found point image coordinates
	 * @returns	true if pattern found, otherwise false
	 */
	bool findPoints(cv::Mat &in, std::vector<cv::Vec2f> &out);

	/**
	 * @brief	Draw points to image
	 * @param	Image to draw to
	 * @param	Pattern points (in image coordinates)
	 */
	void drawPoints(cv::Mat &img, const std::vector<cv::Vec2f> &points);
};

/**
 * @brief	Chessboard calibration pattern. Uses OpenCV's
 * 			findChessboardCornersSB function.
 * @todo	Parameters hardcoded in constructor
 *
 * All parameters (command line parameters):
 * 	- rows, cols: pattern size (inner corners)
 * 	- square_size: millimeters (TODO: meters)
 * 	- width, height: image size, pixels
 * 	- flags: see ChessboardCornersSB documentation (TODO: not implemented)
 */
class CalibrationChessboard : Calibration {
public:
	CalibrationChessboard(const std::map<std::string, std::string> &opt);
	void objectPoints(std::vector<cv::Vec3f> &out);
	bool findPoints(cv::Mat &in, std::vector<cv::Vec2f> &out);
	void drawPoints(cv::Mat &img, const std::vector<cv::Vec2f> &points);

private:
	int chessboard_flags_ = 0;
	float pattern_square_size_;
	cv::Size pattern_size_;
	cv::Size image_size_;
};

// TODO other patterns, circles ...

}
}

#endif // _FTL_CALIBRATION_COMMON_HPP_
