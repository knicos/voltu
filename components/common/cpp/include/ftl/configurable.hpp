#pragma once
#ifndef _FTL_CONFIGURABLE_HPP_
#define _FTL_CONFIGURABLE_HPP_

#include <glog/logging.h>
#include <nlohmann/json.hpp>
#include <string>
#include <tuple>
#include <map>
#include <list>
#include <functional>
#include <optional>

#define REQUIRED(...) required(__func__, __VA_ARGS__)

namespace ftl {

/**
 * The Configurable class should be inherited by any entity that is to be
 * configured using json objects. Additionally, any such object can then be
 * reconfigured through changes to that underlying json object with event
 * callbacks being triggered when specific changes occur.
 * 
 * Configurables may also optionally have a URI that enables static methods
 * of getting and setting configuration options. These static methods are used
 * by network RPCs to enable remote reconfiguration.
 */
class Configurable {
	public:
	Configurable() {}
	explicit Configurable(nlohmann::json &config) : config_(config) {
		if (config["uri"].is_string()) __changeURI(config["uri"].get<std::string>(), this);
	}

	/**
	 * Force the JSON object to have specific properties with a specific type.
	 * If not, emit errors and terminate the application.
	 */
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

	/**
	 * Get a configuration property from the json object. Returns an optional
	 * result which will be empty if the property does not exist or is not of
	 * the requested type.
	 */
	template <typename T>
	std::optional<T> get(const std::string &name) {
		try {
			return config_[name].get<T>();
		} catch (...) {
			return {};
		}
	}

	/**
	 * Change a configuration property and trigger any listening event handlers
	 * for that property. Also triggers the global listeners.
	 */
	template <typename T>
	void set(const std::string &name, T value) {
		config_[name] = value;
		_trigger(name);
	}

	/**
	 * Add callback for whenever a specified property changes value.
	 * @param prop Name of property to watch
	 * @param callback A function object that will be called on change.
	 */
	void on(const std::string &prop, std::function<void(Configurable*, const std::string&)>);

	protected:
	nlohmann::json config_;

	private:
	std::map<std::string, std::list<std::function<void(Configurable*, const std::string&)>>> observers_; 

	void _trigger(const std::string &name);

	static void __changeURI(const std::string &uri, Configurable *cfg);
};

/*template <>
void Configurable::set<const std::string&>(const std::string &name, const std::string &value) {
	config_[name] = value;
	if (name == "uri") __changeURI(value, this);
	_trigger(name);
}*/

}

#endif  // _FTL_CONFIGURABLE_HPP_
