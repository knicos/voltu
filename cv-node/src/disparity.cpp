#include <ftl/disparity.hpp>

using ftl::Disparity;

std::map<std::string,std::function<Disparity*(nlohmann::json&)>> Disparity::algorithms__;

Disparity::Disparity(nlohmann::json &config)
	: 	config_(config),
		min_disp_(config["minimum"]),
		max_disp_(config["maximum"]) {}

Disparity *Disparity::create(nlohmann::json &config) {
	if (algorithms__.count(config["algorithm"]) != 1) return nullptr;
	return algorithms__[config["algorithm"]](config);
}

void Disparity::_register(const std::string &n, std::function<Disparity*(nlohmann::json&)> f) {
	algorithms__[n] = f;
}

