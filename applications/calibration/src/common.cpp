#include <loguru.hpp>
#include <ftl/config.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "common.hpp"

using std::vector;
using std::map;
using std::string;

using cv::Mat;
using cv::Vec2f, cv::Vec3f;
using cv::Size;

using cv::stereoCalibrate;

namespace ftl {
namespace calibration {

// Options
string getOption(const map<string, string> &options, const string &opt) {
	const string str = options.at(opt);
	return str.substr(1, str.size() - 2);
}

bool hasOption(const map<string, string> &options, const string &opt) {
	return options.find(opt) != options.end();
}

int getOptionInt(const map<string, string> &options, const string &opt, int default_value) {
	if (!hasOption(options, opt)) return default_value;
	return std::stoi(options.at(opt));
}

double getOptionDouble(const map<string, string> &options, const string &opt, double default_value) {
	if (!hasOption(options, opt)) return default_value;
	return std::stod(options.at(opt));
}

string getOptionString(const map<string, string> &options, const string &opt, string default_value) {
	if (!hasOption(options, opt)) return default_value;
	return getOption(options, opt);
}

// Save/load files

bool saveExtrinsics(const string &ofile, Mat &R, Mat &T, Mat &R1, Mat &R2, Mat &P1, Mat &P2, Mat &Q) {
	cv::FileStorage fs;
	fs.open(ofile, cv::FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1"
			<< P1 << "P2" << P2 << "Q" << Q;
		fs.release();
		return true;
	} else {
		LOG(ERROR) << "Error: can not save the extrinsic parameters";
	}
	return false;
}

bool saveIntrinsics(const string &ofile, const Mat &M, const Mat& D) {
	cv::FileStorage fs(ofile, cv::FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "M" << M << "D" << D;
		fs.release();
		return true;
	}
	else {
		LOG(ERROR) << "Error: can not save the intrinsic parameters to '" << ofile << "'";
	}
	return false;
}

bool loadIntrinsics(const string &ifile, Mat &M1, Mat &D1) {
	using namespace cv;

	FileStorage fs;

	// reading intrinsic parameters
	fs.open((ifile).c_str(), FileStorage::READ);
	if (!fs.isOpened()) {
		LOG(WARNING) << "Could not open intrinsics file : " << ifile;
		return false;
	}
	
	LOG(INFO) << "Intrinsics from: " << ifile;

	fs["M"] >> M1;
	fs["D"] >> D1;

	return true;
}

Grid::Grid(int rows, int cols, int width, int height, 
		   int offset_x, int offset_y) {
	rows_ = rows;
	cols_ = cols;
	width_ = width;
	height_ = height;
	offset_x_ = offset_x;
	offset_y_ = offset_y;
	cell_width_ = width_ / cols_;
	cell_height_ = height_ / rows_;
	reset();

	corners_ = vector<std::pair<cv::Point, cv::Point>>();

	for (int r = 0; r < rows_; r++) {
	for (int c = 0; c < cols_; c++) {
		int x1 = offset_x_ + c * cell_width_;
		int y1 = offset_y_ + r * cell_height_;
		int x2 = offset_x_ + (c + 1) * cell_width_ - 1;
		int y2 = offset_y_ + (r + 1) * cell_height_ - 1;
		corners_.push_back(std::pair(cv::Point(x1, y1), cv::Point(x2, y2)));
	}}
}

void Grid::drawGrid(Mat &rgb) {
	for (int i = 0; i < rows_ * cols_; ++i) {	
		bool visited = visited_[i];
		cv::Scalar color = visited ? cv::Scalar(24, 255, 24) : cv::Scalar(24, 24, 255);
		cv::rectangle(rgb, corners_[i].first, corners_[i].second, color, 2);
	}
}

int Grid::checkGrid(cv::Point p1, cv::Point p2) {
	// TODO calculate directly

	for (int i = 0; i < rows_ * cols_; ++i) {
		auto &corners = corners_[i];
		if (p1.x >= corners.first.x &&
			p1.x <= corners.second.x &&
			p1.y >= corners.first.y &&
			p1.y <= corners.second.y && 
			p2.x >= corners.first.x &&
			p2.x <= corners.second.x &&
			p2.y >= corners.first.y &&
			p2.y <= corners.second.y) {
			return i;
		}
	}

	return -1;
}

void Grid::updateGrid(int i) {
	if (i >= 0 && i < static_cast<int>(visited_.size()) && !visited_[i]) {
		visited_[i] = true;
		visited_count_ += 1;
	}
}

bool Grid::isVisited(int i) {
	if (i >= 0 && i < static_cast<int>(visited_.size())) {
		return visited_[i];
	}
	return false;
}

bool Grid::isComplete() {
	return visited_count_ == static_cast<int>(visited_.size());
}

void Grid::reset() {
	visited_count_ = 0;
	visited_ = vector<bool>(rows_ * cols_, false);
	// reset visited
}

// Calibration classes for different patterns

CalibrationChessboard::CalibrationChessboard(const map<string, string> &opt) {
	pattern_size_ = Size(	getOptionInt(opt, "cols", 9),
							getOptionInt(opt, "rows", 6));
	image_size_ = Size(	getOptionInt(opt, "width", 1280),
						getOptionInt(opt, "height", 720));
	pattern_square_size_ = getOptionDouble(opt, "square_size", 36.0);

	LOG(INFO) << "Chessboard calibration parameters";
	LOG(INFO) << "         rows: " << pattern_size_.height;
	LOG(INFO) << "         cols: " << pattern_size_.width;
	LOG(INFO) << "        width: " << image_size_.width;
	LOG(INFO) << "       height: " << image_size_.height;
	LOG(INFO) << "  square_size: " << pattern_square_size_;
	LOG(INFO) << "-----------------------------------";

	// CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_EXHAUSTIVE | CALIB_CB_ACCURACY 
	chessboard_flags_ = cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ACCURACY;
}

void CalibrationChessboard::objectPoints(vector<Vec3f> &out) {
	out.reserve(pattern_size_.width * pattern_size_.height);
	for (int row = 0; row < pattern_size_.height; ++row) {
	for (int col = 0; col < pattern_size_.width; ++col) {
		out.push_back(Vec3f(col * pattern_square_size_, row * pattern_square_size_, 0));
	}}
}

bool CalibrationChessboard::findPoints(Mat &img, vector<Vec2f> &points) {
	return cv::findChessboardCornersSB(img, pattern_size_, points, chessboard_flags_);
}

void CalibrationChessboard::drawPoints(Mat &img, const vector<Vec2f> &points) {
	cv::drawChessboardCorners(img, pattern_size_, points, true);
}

}
}
