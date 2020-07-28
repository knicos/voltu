#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/aruco.hpp>

/** Calibration objects */

namespace ftl
{
namespace calibration
{

class CalibrationObject {
public:
	virtual int detect(cv::InputArray, std::vector<cv::Point2d>&, const cv::Mat& K=cv::Mat(), const cv::Mat& D=cv::Mat()) = 0;
	virtual std::vector<cv::Point3d> object() = 0;
};

class ChessboardObject : public CalibrationObject {
public:
	ChessboardObject(int rows=18, int cols=25, double square_size=0.015);
	virtual int detect(cv::InputArray, std::vector<cv::Point2d>&, const cv::Mat& K=cv::Mat(), const cv::Mat& D=cv::Mat());
	std::vector<cv::Point3d> object() override;

	cv::Size chessboardSize();
	double squareSize();

private:
	void init();
	cv::Size chessboard_size_;
	double square_size_;
	int flags_;
	std::vector<cv::Point3d> object_points_;
};

class ArUCoObject : public CalibrationObject {
public:
	ArUCoObject(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary = cv::aruco::DICT_6X6_100,	float baseline = 0.25f, float tag_size = 0.15, int id1=0, int id2=1);
	virtual int detect(cv::InputArray, std::vector<cv::Point2d>&, const cv::Mat& K=cv::Mat(), const cv::Mat& D=cv::Mat());
	std::vector<cv::Point3d> object() override;

private:
	cv::Ptr<cv::aruco::Dictionary> dict_;
	cv::Ptr<cv::aruco::DetectorParameters> params_;
	float baseline_;
	float tag_size_;
	int id1_;
	int id2_;
};

} // namespace calibration
} // namespace ft
