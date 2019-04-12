/*
 * Copyright 2019 Nicolas Pope
 */

#ifndef _FTL_DISPARITY_HPP_
#define _FTL_DISPARITY_HPP_

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

namespace ftl {

/**
 * Virtual base class for disparity algorithms. An automatic factory is used
 * to construct instances of specific algorithms that implement this
 * interface, for this to work a static instance of the Register class must
 * be created in the algorithms cpp file.
 */
class Disparity {
	public:
	explicit Disparity(nlohmann::json &config);
	
	virtual void setMinDisparity(size_t min) { min_disp_ = min; }
	virtual void setMaxDisparity(size_t max) { max_disp_ = max; }
	
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
		Register(const std::string &n, std::function<Disparity*(nlohmann::json&)> f) {
			Disparity::_register(n,f);
		};
	};
	
	/**
	 * Factory instance creator where config contains an "algorithm" property
	 * used as the instance name to construct.
	 */
	static Disparity *create(nlohmann::json &config);
	
	protected:
	static void _register(const std::string &n, std::function<Disparity*(nlohmann::json&)> f);
	
	protected:
	nlohmann::json &config_;
	size_t min_disp_;
	size_t max_disp_;
	
	private:
	static std::map<std::string,std::function<Disparity*(nlohmann::json&)>> algorithms__;
};
};

#endif // _FTL_DISPARITY_HPP_
