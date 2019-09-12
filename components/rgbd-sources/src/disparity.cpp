/*
 * Copyright 2019 Nicolas Pope
 */

#include "disparity.hpp"
#include <loguru.hpp>
#include <ftl/config.h>
#include <ftl/configuration.hpp>

using ftl::rgbd::detail::Disparity;

std::map<std::string, std::function<Disparity*(ftl::Configurable *, const std::string &)>>
		*Disparity::algorithms__ = nullptr;

Disparity::Disparity(nlohmann::json &config)
	: 	ftl::Configurable(config),
		min_disp_(value("minimum",0)),
		max_disp_(value("maximum", 256)),
		size_(value("width", 1280), value("height", 720))
	{

	}

Disparity *Disparity::create(ftl::Configurable *parent, const std::string &name) {
	nlohmann::json &config = ftl::config::resolve((!parent->getConfig()[name].is_null()) ? parent->getConfig()[name] : ftl::config::resolve(parent->getConfig())[name]); // ftl::config::resolve(parent->getConfig()[name]);

	//auto alg = parent->get<std::string>("algorithm");
	if (!config["algorithm"].is_string()) {
		return nullptr;
	}
	std::string alg = config["algorithm"].get<std::string>();

	if (algorithms__->count(alg) != 1) return nullptr;
	return (*algorithms__)[alg](parent, name);
}

void Disparity::_register(const std::string &n,
		std::function<Disparity*(ftl::Configurable *, const std::string &)> f) {
	if (!algorithms__) algorithms__ = new std::map<std::string, std::function<Disparity*(ftl::Configurable *, const std::string &)>>;
	//LOG(INFO) << "Register disparity algorithm: " << n;
	(*algorithms__)[n] = f;
}

void Disparity::scaleInput(	const cv::cuda::GpuMat& left_in,
							const cv::cuda::GpuMat& right_in,
							cv::cuda::GpuMat& left_out,
							cv::cuda::GpuMat& right_out,
							cv::cuda::Stream &stream)
{
	cv::cuda::resize(left_in, left_scaled_, size_, 0.0, 0.0, cv::INTER_CUBIC, stream);
	left_out = left_scaled_;
	cv::cuda::resize(right_in, right_scaled_, size_, 0.0, 0.0, cv::INTER_CUBIC, stream);
	right_out = right_scaled_;
}

void Disparity::scaleDisparity(	const cv::Size&		new_size,
								cv::cuda::GpuMat&	in,
								cv::cuda::GpuMat&	out,
								cv::cuda::Stream&	stream)
{
	cv::cuda::multiply(in, (double) new_size.width / (double) in.cols, in);
	cv::cuda::resize(in, dispt_scaled_, new_size, 0.0, 0.0, cv::INTER_NEAREST, stream);
	out = dispt_scaled_;
}

// TODO:(Nick) Add remaining algorithms
/*
#include "algorithms/rtcensus.hpp"
static ftl::rgbd::detail::Disparity::Register rtcensus("rtcensus", ftl::algorithms::RTCensus::create);
*/

#ifdef HAVE_LIBSGM
#include "algorithms/fixstars_sgm.hpp"
static ftl::rgbd::detail::Disparity::Register fixstarssgm("libsgm", ftl::algorithms::FixstarsSGM::create);
#endif  // HAVE_LIBSGM

