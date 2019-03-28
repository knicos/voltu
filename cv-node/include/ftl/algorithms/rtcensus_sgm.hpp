#ifndef _FTL_ALGORITHMS_RTCENSUSSGM_HPP_
#define _FTL_ALGORITHMS_RTCENSUSSGM_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ftl/disparity.hpp>
#include <nlohmann/json.hpp>

#if defined HAVE_CUDA
#include <opencv2/core/cuda.hpp>
#endif

namespace ftl {
namespace algorithms {
class RTCensusSGM : public ftl::Disparity {
	public:
	RTCensusSGM(nlohmann::json &config);
	
	void setGamma(float gamma) { gamma_ = gamma; }
	void setTau(float tau) { tau_ = tau; }
	
	void compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp);

	static inline Disparity *create(nlohmann::json &config) { return new RTCensusSGM(config); }
	
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
	#if defined HAVE_CUDA
	void computeCUDA(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp);
	#endif
};
};
};

#endif // _FTL_ALGORITHMS_RTCENSUSSGM_HPP_

