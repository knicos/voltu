/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_DISPARITY_HPP_
#define _FTL_DISPARITY_HPP_

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <ftl/configurable.hpp>

namespace ftl {

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
	
	virtual void setMask(cv::Mat &mask) { mask_l_ = mask; }
	
	/**
	 * Pure virtual function representing the actual computation of
	 * disparity from left and right images to be implemented.
	 */
	virtual void compute(const cv::Mat &l, const cv::Mat &r, cv::Mat &disp)=0;

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
	//nlohmann::json &config_;
	int min_disp_;
	int max_disp_;
	cv::Mat mask_l_;
	
	private:
	static std::map<std::string,std::function<Disparity*(ftl::Configurable *, const std::string &)>> *algorithms__;
};
};

#endif // _FTL_DISPARITY_HPP_

