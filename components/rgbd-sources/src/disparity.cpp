/*
 * Copyright 2019 Nicolas Pope
 */

#include "disparity.hpp"
#include <glog/logging.h>
#include <ftl/config.h>

using ftl::Disparity;

std::map<std::string, std::function<Disparity*(nlohmann::json&)>>
		*Disparity::algorithms__ = nullptr;

Disparity::Disparity(nlohmann::json &config)
	: 	config_(config),
		min_disp_(config["minimum"]),
		max_disp_(config["maximum"]) {}

Disparity *Disparity::create(nlohmann::json &config) {
	if (algorithms__->count(config["algorithm"]) != 1) return nullptr;
	return (*algorithms__)[config["algorithm"]](config);
}

void Disparity::_register(const std::string &n,
		std::function<Disparity*(nlohmann::json&)> f) {
	if (!algorithms__) algorithms__ = new std::map<std::string, std::function<Disparity*(nlohmann::json&)>>;
	LOG(INFO) << "Register disparity algorithm: " << n;
	(*algorithms__)[n] = f;
}

// TODO(Nick) Add remaining algorithms

#include "algorithms/rtcensus.hpp"
static ftl::Disparity::Register rtcensus("rtcensus", ftl::algorithms::RTCensus::create);

#ifdef HAVE_LIBSGM
#include "algorithms/fixstars_sgm.hpp"
static ftl::Disparity::Register fixstarssgm("libsgm", ftl::algorithms::FixstarsSGM::create);
#endif  // HAVE_LIBSGM

