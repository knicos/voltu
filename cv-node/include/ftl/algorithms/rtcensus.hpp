#ifndef _FTL_ALGORITHMS_RTCENSUS_HPP_
#define _FTL_ALGORITHMS_RTCENSUS_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ftl/disparity.hpp>
#include <nlohmann/json.hpp>

namespace ftl {
namespace algorithms {
class RTCensus : public ftl::Disparity {
	public:
	RTCensus(nlohmann::json &config);
	
	void setGamma(float gamma) { gamma_ = gamma; }
	void setTau(float tau) { tau_ = tau; }
	
	void compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp);

	static inline Disparity *create(nlohmann::json &config) { return new RTCensus(config); }
	
	private:
	float gamma_;
	float tau_;
};
};
};

#endif // _FTL_ALGORITHMS_RTCENSUS_HPP_

