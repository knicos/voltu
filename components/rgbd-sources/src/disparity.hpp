/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_DISPARITY_HPP_
#define _FTL_DISPARITY_HPP_

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <ftl/configurable.hpp>
#include <ftl/rgbd/frame.hpp>

namespace ftl {
namespace rgbd {
namespace detail {

/**
 * Virtual base class for disparity algorithms. An automatic factory is used
 * to construct instances of specific algorithms that implement this
 * interface, for this to work a static instance of the Register class must
 * be created in the algorithms cpp file.
 */
class Disparity : public ftl::Configurable {
	public:
	explicit Disparity(nlohmann::json &config);
	
	virtual void setMinDisparity(size_t min) { min_disp_ = min; }
	virtual void setMaxDisparity(size_t max) { max_disp_ = max; }
	
	virtual void setMask(cv::Mat &mask) { mask_l_ = cv::cuda::GpuMat(mask); }
	virtual void setMask(cv::cuda::GpuMat &mask) { mask_l_ = mask; }
	
	void scaleInput(const cv::cuda::GpuMat& left_in,
					const cv::cuda::GpuMat& right_in,
					cv::cuda::GpuMat& left_out,
					cv::cuda::GpuMat& right_out,
					cv::cuda::Stream &stream);
	
	void scaleDisparity(const cv::Size &new_size,
						cv::cuda::GpuMat& in,
						cv::cuda::GpuMat& out,
						cv::cuda::Stream &stream);

	/**
	 * Pure virtual function representing the actual computation of
	 * disparity from left and right images to be implemented.
	 */
	virtual void compute(Frame &frame, cv::cuda::Stream &stream)=0;
	virtual void compute(cv::cuda::GpuMat &l, cv::cuda::GpuMat &r, cv::cuda::GpuMat &disp, cv::cuda::Stream &stream)
	{
		// FIXME: What were these for?
		//ftl::rgbd::Frame frame;
		//frame.create<cv::cuda::GpuMat>(ftl::rgbd::Channel::Left) = l;
		//frame.create<cv::cuda::GpuMat>(ftl::rgbd::Channel::Right) = r;
		//frame.create<cv::cuda::GpuMat>(ftl::rgbd::Channel::Disparity) = disp;
	}

	/**
	 * Factory registration class.
	 */
	class Register {
		public:
		// cppcheck-suppress *
		Register(const std::string &n, std::function<Disparity*(ftl::Configurable *, const std::string &)> f) {
			Disparity::_register(n,f);
		};
	};
	
	/**
	 * Factory instance creator where config contains an "algorithm" property
	 * used as the instance name to construct.
	 */
	static Disparity *create(ftl::Configurable *, const std::string &);
	
	protected:
	static void _register(const std::string &n, std::function<Disparity*(ftl::Configurable *, const std::string &)> f);
	
protected:
	int min_disp_;
	int max_disp_;
	cv::Size size_;
	
	cv::cuda::GpuMat left_scaled_;
	cv::cuda::GpuMat right_scaled_;
	cv::cuda::GpuMat dispt_scaled_;
	cv::cuda::GpuMat mask_l_;
	
	private:
	static std::map<std::string,std::function<Disparity*(ftl::Configurable *, const std::string &)>> *algorithms__;
};

}
}
}

#endif // _FTL_DISPARITY_HPP_
