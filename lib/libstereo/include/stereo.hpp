#pragma once

#include <opencv2/core/mat.hpp>
#include <stereo_types.hpp>

class StereoGCensusSgm {
public:
	StereoGCensusSgm();
	~StereoGCensusSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};
	void setEdges();

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		float P1 = 0.0645;
		float P2 = 1.2903;
		float uniqueness = std::numeric_limits<float>::max();
		int subpixel = 1; // subpixel interpolation method
		bool lr_consistency = true;
		int paths = AggregationDirections::HORIZONTAL |
					AggregationDirections::VERTICAL |
					AggregationDirections::DIAGONAL;
		bool debug = false;
	};
	Parameters params;

	enum Pattern {
		DENSE,
		SPARSE,
		RANDOM,
		GCT,
	};

	/**
	 * Set pattern.
	 *
	 * 		DENSE: size required, param ignored
	 * 		SPARSE: size and parama required, param is step (number of skipped pixels)
	 * 		RANDOM: size and param required, param is number of edges
	 * 		GCT: param required, size ignored, param is pattern type (number of edges), see the paper for description
	 */
	void setPattern(Pattern type, cv::Size size, int param=-1);
	/**
	 * Set custom pattern.
	 */
	void setPattern(const std::vector<std::pair<cv::Point2i, cv::Point2i>> &edges);

private:
	struct Impl;
	Impl *impl_;
	std::vector<std::pair<cv::Point2i, cv::Point2i>> pattern_;
};


class StereoADCensusSgm {
public:
	StereoADCensusSgm();
	~StereoADCensusSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};

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
	void setPrior(cv::InputArray disp) {};

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
	void setPrior(cv::InputArray disp) {};

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 5;
		unsigned short P2 = 25;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1; // subpixel interpolation method
		bool lr_consistency = true;
		int paths = AggregationDirections::HORIZONTAL |
					AggregationDirections::VERTICAL |
					AggregationDirections::DIAGONAL;
		bool debug = false;
		CensusPattern pattern = CensusPattern::STANDARD;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};

class StereoExCensusSgm {
public:
	StereoExCensusSgm();
	~StereoExCensusSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 5;
		unsigned short P2 = 25;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1; // subpixel interpolation method
		bool lr_consistency = true;
		int paths = AggregationDirections::HORIZONTAL |
					AggregationDirections::VERTICAL |
					AggregationDirections::DIAGONAL;
		bool debug = false;
		/** normalization of variance to range [alpha, beta] */
		float alpha = 0.2;
		float beta = 1.0;
		/** variance window size */
		int var_window = 5;
		CensusPattern pattern = CensusPattern::STANDARD;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};

/**
 * Mean reference pixel. Doesn't work since detail is lost form reference
 *
 * @see "Froba, B., Ernst, A. "Face detection with the modified census transform," In: Sixth IEEE International Conference on Automatic Face and Gesture Recognition. IEEE Computer Society Press, Los Alamitos (2004)."
 * @see "A robust local census-based stereo matching insensitive to illumination changes" (2012)
 * @see "A stereo matching algorithm based on four-moded census and relative confidence plane fitting" (2015)
 * @see "Stereo matching based on improved census transform" (2018)
 */
class StereoMeanCensusSgm {
public:
	StereoMeanCensusSgm();
	~StereoMeanCensusSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 5;
		unsigned short P2 = 25;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1; // subpixel interpolation method
		bool lr_consistency = true;
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
 * Bilateral filtering for reference pixel. Doesn't make things better, but
 * still better than mean value for reference.
 */
class StereoBRefCensusSgm {
public:
	StereoBRefCensusSgm();
	~StereoBRefCensusSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 5;
		unsigned short P2 = 25;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1; // subpixel interpolation method
		bool lr_consistency = true;
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

/* Clustered Salient Features with focal point */
class StereoCSF {
public:
	StereoCSF();
	~StereoCSF();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};

	struct Parameters {
		bool debug = false;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};

/**
 * STABLE Binary descriptor. This is a general implementation.
 *
 * @see K. Valentín, R. Huber-Mörk, and S. Štolc, “Binary descriptor-based dense
 *      line-scan stereo matching,” J. Electron. Imaging, vol. 26, no. 1, 2017.
 */
class StereoStableSgm {
public:
	StereoStableSgm();
	~StereoStableSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 5;
		unsigned short P2 = 25;
		short wsize = 17;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1; // subpixel interpolation method
		bool lr_consistency = true;
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
 * Three resolutions, fine, medium and coarse, are combined. In each case the
 * reference pixel always comes from the fine resolution. Each resolution is
 * variance weighted for that resolution to increase and decrease influence,
 * allowing textured high res to dominate when possible.
 */
class StereoHStableSgm {
public:
	StereoHStableSgm();
	~StereoHStableSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 5;
		unsigned short P2 = 25;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1; // subpixel interpolation method
		bool lr_consistency = true;
		int paths = AggregationDirections::HORIZONTAL |
					AggregationDirections::VERTICAL |
					AggregationDirections::DIAGONAL;
		/** normalization of variance to range [alpha, beta] */
		float alpha = 0.2;
		float beta = 1.0;
		/** variance window size */
		int var_window = 9;
		bool debug = false;
		short wsize = 17;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};

/**
 * Three resolutions, fine, medium and coarse, are combined. In each case the
 * reference pixel always comes from the fine resolution. Each resolution is
 * variance weighted for that resolution to increase and decrease influence,
 * allowing textured high res to dominate when possible.
 */
class StereoHierCensusSgm {
public:
	StereoHierCensusSgm();
	~StereoHierCensusSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 5;
		unsigned short P2 = 25;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1; // subpixel interpolation method
		bool lr_consistency = true;
		int paths = AggregationDirections::HORIZONTAL |
					AggregationDirections::VERTICAL |
					AggregationDirections::DIAGONAL;
		/** normalization of variance to range [alpha, beta] */
		float alpha = 0.2;
		float beta = 1.0;
		/** variance window size */
		int var_window = 9;
		bool debug = false;
		CensusPattern pattern = CensusPattern::STANDARD;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};

/**
 * Three resolutions, fine, medium and coarse, are combined. In each case the
 * reference pixel always comes from the fine resolution. Each resolution is
 * variance weighted for that resolution to increase and decrease influence,
 * allowing textured high res to dominate when possible.
 */
class StereoHierWeightCensusSgm {
public:
	StereoHierWeightCensusSgm();
	~StereoHierWeightCensusSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 5;
		unsigned short P2 = 25;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1; // subpixel interpolation method
		bool lr_consistency = true;
		int paths = AggregationDirections::HORIZONTAL |
					AggregationDirections::VERTICAL |
					AggregationDirections::DIAGONAL;
		/** normalization of variance to range [alpha, beta] */
		float alpha = 0.2;
		float beta = 1.0;
		/** variance window size */
		int var_window = 7;
		bool debug = false;
		CensusPattern pattern = CensusPattern::STANDARD;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};

class StereoCPCensusSgm {
public:
	StereoCPCensusSgm();
	~StereoCPCensusSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 5;
		unsigned short P2 = 25;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1; // subpixel interpolation method
		bool lr_consistency = true;
		int paths = AggregationDirections::HORIZONTAL |
					AggregationDirections::VERTICAL |
					AggregationDirections::DIAGONAL;
		bool debug = false;
		bool filtercosts = true;
		/** normalization of variance to range [alpha, beta] */
		float alpha = 0.5;
		float beta = 1.0;
		/** variance window size */
		int var_window = 7;
		CensusPattern pattern = CensusPattern::STANDARD;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};

class StereoWCensusSgm {
public:
	StereoWCensusSgm();
	~StereoWCensusSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 5;
		unsigned short P2 = 25;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1; // subpixel interpolation method
		bool lr_consistency = true;
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
 * Ternary census, or 3 moded, where there is a noise threshold and where
 * pixels can be identified as no luminance change in addition to above or
 * below.
 *
 * @see "TEXTURE-AWARE DENSE IMAGE MATCHING USING TERNARY CENSUS TRANSFORM" (2016)
 * @see "Local disparity estimation with three-moded cross census and advanced support weight" (2013)
 */
class StereoTCensusSgm {
public:
	StereoTCensusSgm();
	~StereoTCensusSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		uchar t = 8;
		unsigned short P1 = 5;
		unsigned short P2 = 25;
		float uniqueness = std::numeric_limits<unsigned short>::max();
		int subpixel = 1; // subpixel interpolation method
		bool lr_consistency = true;
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

class StereoCensusAdaptive {
public:
	StereoCensusAdaptive();
	~StereoCensusAdaptive();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 5;
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
	void setPrior(cv::InputArray disp);

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
 * Work in progress
 *
 * Mutual Information and Census cost with SGM. Cost calculation uses intensity
 * variance to weight MI and Census costs (low weight for census cost on
 * low variance areas). Window size for local variance based on census mask size.
 */
class StereoMiSgm2 {
public:
	StereoMiSgm2();
	~StereoMiSgm2();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp);

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		unsigned short P1 = 1;
		unsigned short P2 = 16;
		float uniqueness = std::numeric_limits<short>::max();
		int subpixel = 1; // subpixel interpolation method
		int paths = AggregationDirections::HORIZONTAL |
					AggregationDirections::VERTICAL |
					AggregationDirections::DIAGONAL;

		float alpha = 0.2; /** alpha: minimum weight for census cost */
		float beta = 0.7; /** 1-beta: minimum weight for MI cost */
		int var_window = 9;
		bool debug = false;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};

class StereoWADCensus {
public:
	StereoWADCensus();
	~StereoWADCensus();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		float P1 = 1;
		float P2 = 16;
		float l1 = 1.0;
		float l2 = 1.0;
		int wsize = 31;
		float uniqueness = std::numeric_limits<short>::max();
		int subpixel = 1; // subpixel interpolation method
		int paths = AggregationDirections::HORIZONTAL |
					AggregationDirections::VERTICAL |
					AggregationDirections::DIAGONAL;

		float alpha = 0.2; /** alpha: minimum weight for census cost */
		float beta = 0.7; /** 1-beta: minimum weight for MI cost */
		int var_window = 9;
		bool debug = false;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};

class StereoVarCensus {
public:
	StereoVarCensus();
	~StereoVarCensus();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp) {};

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		float P1 = 1;
		float P2 = 16;
		int wsize = 31;
		float uniqueness = std::numeric_limits<short>::max();
		int subpixel = 1; /** subpixel interpolation method */
		bool lr_consistency = true;
		int paths = AggregationDirections::HORIZONTAL |
					AggregationDirections::VERTICAL |
					AggregationDirections::DIAGONAL;

		/** normalization of variance to range [alpha, beta] */
		float alpha = 0.2;
		float beta = 1.0;
		/** variance window size */
		int var_window = 9;
		bool debug = false;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};

class StereoSad {
public:
	StereoSad();
	~StereoSad();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp);

	struct Parameters {
		int wsize = 63;
		int d_min = 0;
		int d_max = 0;
		int subpixel = 1; // subpixel interpolation method
		bool debug = false;
	};
	Parameters params;

private:
	struct Impl;
	Impl *impl_;
};


/**
 * SGM aggregation on ground truth cost
 */
class StereoGtSgm {
public:
	StereoGtSgm();
	~StereoGtSgm();

	void compute(cv::InputArray l, cv::InputArray r, cv::OutputArray disparity);
	void setPrior(cv::InputArray disp);

	struct Parameters {
		int d_min = 0;
		int d_max = 0;
		float P1 = 0.1f;
		float P2 = 1.0f;
		float uniqueness = std::numeric_limits<float>::max();
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
	void setPrior(cv::InputArray disp) {};

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
