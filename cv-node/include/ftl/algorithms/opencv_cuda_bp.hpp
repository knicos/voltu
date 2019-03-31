/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_ALGORITHMS_OPENCV_CUDA_BP_HPP_
#define _FTL_ALGORITHMS_OPENCV_CUDA_BP_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudastereo.hpp>
#include <ftl/disparity.hpp>

namespace ftl {
namespace algorithms {

/**
 * OpenCV CUDA Belief Propagation algorithm.
 */
class OpenCVCudaBP : public ftl::Disparity {
	public:
	explicit OpenCVCudaBP(nlohmann::json &config);
	
	void compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp);

	static inline Disparity *create(nlohmann::json &config) {
		return new OpenCVCudaBP(config);
	}
	
	private:
	cv::Ptr<cv::cuda::StereoBeliefPropagation> matcher_;
	cv::cuda::GpuMat disp_;
	cv::cuda::GpuMat left_;
	cv::cuda::GpuMat right_;
};
};
};

#endif  // _FTL_ALGORITHMS_OPENCV_CUDA_BP_HPP_

