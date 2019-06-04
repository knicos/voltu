/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_ALGORITHMS_RTCENSUS_HPP_
#define _FTL_ALGORITHMS_RTCENSUS_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "../disparity.hpp"
#include <nlohmann/json.hpp>

#include <ftl/config.h>
#include <ftl/configuration.hpp>

#if defined HAVE_CUDA
#include <opencv2/core/cuda.hpp>
#endif

namespace ftl {
namespace algorithms {

/**
 * Real-time Sparse Census disparity algorithm.
 */
class RTCensus : public ftl::Disparity {
	public:
	explicit RTCensus(nlohmann::json &config);
	
	void setGamma(float gamma) { gamma_ = gamma; }
	void setTau(float tau) { tau_ = tau; }
	
	void compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp);

	static inline Disparity *create(ftl::Configurable *p, const std::string &name) {
		return ftl::create<RTCensus>(p, name);
	}
	
	private:
	float gamma_;
	float tau_;
	bool use_cuda_;
	bool alternate_;
	
	#if defined HAVE_CUDA
	cv::cuda::GpuMat disp_;
	cv::cuda::GpuMat filtered_;
	cv::cuda::GpuMat left_;
	cv::cuda::GpuMat left2_;
	cv::cuda::GpuMat right_;
	#endif
	
	private:
	void computeCPU(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp);
	
	#if defined HAVE_CUDA
	void computeCUDA(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp);
	#endif
};
};
};

#endif  // _FTL_ALGORITHMS_RTCENSUS_HPP_

