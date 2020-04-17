#pragma once

#include <opencv2/core/mat.hpp>
#include <stereo_types.hpp>

class StereoADCensusSgm {
public:
	StereoADCensusSgm();
	~StereoADCensusSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 5;
		unsigned short P2 = 25;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1; // subpixel interpolation method
		int paths = AggregationDirections::HORIZONTAL |
					AggregationDirections::VERTICAL |
					AggregationDirections::DIAGONAL;
		bool debug = false;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};

class StereoADSgm {
public:
	StereoADSgm();
	~StereoADSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 5;
		unsigned short P2 = 25;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1; // subpixel interpolation method
		int paths = AggregationDirections::HORIZONTAL |
					AggregationDirections::VERTICAL |
					AggregationDirections::DIAGONAL;
		bool debug = false;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};

class StereoCensusSgm {
public:
	StereoCensusSgm();
	~StereoCensusSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 5;
		unsigned short P2 = 25;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1; // subpixel interpolation method
		int paths = AggregationDirections::HORIZONTAL |
					AggregationDirections::VERTICAL |
					AggregationDirections::DIAGONAL;
		bool debug = false;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};

class StereoMiSgm {
public:
	StereoMiSgm();
	~StereoMiSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(const cv::Mat &disp);

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 2;
		unsigned short P2 = 8;
		float uniqueness = std::numeric_limits<short>::max();
		int subpixel = 1; // subpixel interpolation method
		int paths = AggregationDirections::HORIZONTAL |
					AggregationDirections::VERTICAL |
					AggregationDirections::DIAGONAL;
		bool debug = false;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};


/**
 * Census + SGM + prior
 *
class StereoCensusSgmP {
public:
	StereoCensusSgmP();
	~StereoCensusSgmP();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(const cv::Mat &prior);

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		int range = 10;
		unsigned short P1 = 5;
		unsigned short P2 = 25;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1; // subpixel interpolation method
		int paths = AggregationDirections::HORIZONTAL |
					AggregationDirections::VERTICAL |
					AggregationDirections::DIAGONAL;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};*/

class StereoGradientStree {
public:
	StereoGradientStree();
	~StereoGradientStree();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		float P1 = 5;
		float P2 = 25;
		float P3 = 64;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1;
		bool debug = false;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};
