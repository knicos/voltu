#pragma once

#include <opencv2/core/mat.hpp>

struct MiddEvalResult {
	int n;
	int bad;
	int invalid;
	float err_bad;
	float err_invalid;
	float err_total;
	float mse_bad;
	float mse_good;
	float mse_total;
	float avgerr;
	float threshold;
};

struct MiddEvalCalib {
	float f;
	float cx;
	float cy;
	float baseline;
	float doffs;
	int width;
	int height;
	int ndisp;
	int vmin;
	int vmax;
};

MiddEvalCalib read_calibration(const std::string &filename);

MiddEvalResult evaluate(const cv::Mat &disp, const cv::Mat &gt, const cv::Mat &mask, float threshold=1.0f);

cv::Mat read_pfm(const std::string &filename);

struct MiddleburyData {
	const std::string name;
	const cv::Mat imL;
	const cv::Mat imR;
	const cv::Mat gtL;
	const cv::Mat maskL;
	const MiddEvalCalib calib;
};

/** Load one middlebury dataset image
 * @param path path to image directory
 */
MiddleburyData load_input(const std::string &path);

/** Load all middlebury dataset images
 * @param path path to middlebury datase directory
 */
std::vector<MiddleburyData> load_directory(const std::string &path);

/**
 * Add gaussian noise to image.
 *
 * @param	im		CV_8UC3
 * @param	stddev	standar deviation
 * @param	mean	mean for noise
 */
void add_noise(cv::Mat &im, double stddev, double mean=0.0);
