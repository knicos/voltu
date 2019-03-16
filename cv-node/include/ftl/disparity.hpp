#ifndef _FTL_DISPARITY_HPP_
#define _FTL_DISPARITY_HPP_

#include <opencv2/opencv.hpp>

namespace ftl {
class Disparity {
	public:
	Disparity();
	
	virtual void setMinDisparity(size_t min) { min_disp_ = min; }
	virtual void setMaxDisparity(size_t max) { max_disp_ = max; }
	
	virtual void compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp)=0;
	
	class Register {
		public:
		Register(const std::string &n, std::function<Disparity*()> f) {
			Disparity::_register(n,f);
		};
	};
	
	static Disparity *create(const std::string &n);
	
	protected:
	static void _register(const std::string &n, std::function<Disparity*()> f);
	
	protected:
	size_t min_disp_;
	size_t max_disp_;
	
	private:
	static std::map<std::string,std::function<Disparity*()>> algorithms__;
};
};

#endif // _FTL_DISPARITY_HPP_

