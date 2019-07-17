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
		max_disp_(value("maximum", 256)) {}

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

// TODO:(Nick) Add remaining algorithms
/*
#include "algorithms/rtcensus.hpp"
static ftl::rgbd::detail::Disparity::Register rtcensus("rtcensus", ftl::algorithms::RTCensus::create);
*/

#ifdef HAVE_LIBSGM
#include "algorithms/fixstars_sgm.hpp"
static ftl::rgbd::detail::Disparity::Register fixstarssgm("libsgm", ftl::algorithms::FixstarsSGM::create);
#endif  // HAVE_LIBSGM

