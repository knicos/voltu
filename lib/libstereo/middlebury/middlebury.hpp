#pragma once

#include <opencv2/core/mat.hpp>

struct MiddEvalResult {
	int n;
	int bad;
	int invalid;
	float err_bad;
	float err_invalid;
	float err_total;
	float rms_bad;
	float rms_good;
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

/**
 * Add gaussian noise to image.
 *
 * @param	im		CV_8UC3
 * @param	stddev	standar deviation
 * @param	mean	mean for noise
 */
void add_noise(cv::Mat &im, double stddev, double mean=0.0);
