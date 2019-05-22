#pragma once
#ifndef _FTL_CONFIGURABLE_HPP_
#define _FTL_CONFIGURABLE_HPP_

#include <glog/logging.h>
#include <nlohmann/json.hpp>
#include <string>
#include <tuple>

#define REQUIRED(...) required(__func__, __VA_ARGS__)

namespace ftl {

class Configurable {
	public:
	Configurable() {}
	explicit Configurable(nlohmann::json &config) : config_(config) {

	}

	void required(const char *f, const std::vector<std::tuple<std::string, std::string, std::string>> &r) {
		bool diderror = false;
		for (auto i : r) {
			auto [name, desc, type] = i;
			if (config_[name].type_name() != type) {
				LOG(ERROR) << "Missing required option in \"" << f << "\": \"" << name << "\" - " << desc;
				LOG(ERROR) << "    Got type " << config_[name].type_name() << " but expected " << type;
				diderror = true;
			}
		}
		if (diderror) LOG(FATAL) << "Cannot continue without required option";
	}

	nlohmann::json &getConfig() { return config_; }

	protected:
	nlohmann::json config_;
};

}

#endif  // _FTL_CONFIGURABLE_HPP_
