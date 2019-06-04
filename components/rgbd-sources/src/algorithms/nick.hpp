#ifndef _FTL_ALGORITHMS_NICK_HPP_
#define _FTL_ALGORITHMS_NICK_HPP_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudastereo.hpp>
#include "../disparity.hpp"
#include <ftl/configuration.hpp>

namespace ftl {
namespace algorithms {
class NickCuda : public ftl::Disparity {
	public:
	NickCuda(nlohmann::json &config);
	
	void compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp);

	static inline Disparity *create(ftl::Configurable *p, const std::string &name) {
		return ftl::create<NickCuda>(p, name);
	}
	
	private:
	cv::cuda::GpuMat disp_;
	//cv::cuda::GpuMat filtered_;
	cv::cuda::GpuMat left_;
	cv::cuda::GpuMat right_;
};
};
};

#endif // _FTL_ALGORITHMS_NICK_HPP_

