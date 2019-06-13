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

string getOption(map<string, string> &options, const string &opt) {
	auto str = options[opt];
	return str.substr(1,str.size()-2);
}

bool hasOption(const map<string, string> &options, const string &opt) {
	return options.find(opt) != options.end();
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

// Calibration classes for different patterns

CalibrationChessboard::CalibrationChessboard(const map<string, string> &opt) {
	pattern_size_ = Size(9, 6);
	image_size_ = Size(1280, 720);
	pattern_square_size_ = 36.0;//0.036;
	// CALIB_CB_NORMALIZE_IMAfE | CALIB_CB_EXHAUSTIVE | CALIB_CB_ACCURACY 
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
