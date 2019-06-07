#pragma once
#ifndef _FTL_CONFIGURABLE_HPP_
#define _FTL_CONFIGURABLE_HPP_

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <tuple>
#include <map>
#include <list>
#include <functional>
#include <optional>

#define REQUIRED(...) required(__func__, __VA_ARGS__)

namespace ftl {

class Configurable;

namespace config {
struct Event {
	Configurable *entity;
	std::string name;
};
}

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
	Configurable();
	explicit Configurable(nlohmann::json &config);
	virtual ~Configurable() {}

	/**
	 * Force the JSON object to have specific properties with a specific type.
	 * If not, emit errors and terminate the application.
	 */
	void required(const char *f, const std::vector<std::tuple<std::string, std::string, std::string>> &r);

	/**
	 * Return raw JSON entity for this Configurable.
	 */
	nlohmann::json &getConfig() { return config_; }

	/**
	 * Get a configuration property from the json object. Returns an optional
	 * result which will be empty if the property does not exist or is not of
	 * the requested type.
	 */
	template <typename T>
	std::optional<T> get(const std::string &name);

	/**
	 * Get a configuration property, but return a default if not found.
	 */
	template <typename T>
	T value(const std::string &name, T def) {
		auto r = get<T>(name);
		return (r) ? *r : def;
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
	 * Create or find existing configurable object of given type from the
	 * given property name. Additional constructor arguments can also be
	 * provided. Any kind of failure results in a nullptr being returned.
	 */
	template <typename T, typename... ARGS>
	T *create(const std::string &name, ARGS ...args);

	/**
	 * Add callback for whenever a specified property changes value.
	 * @param prop Name of property to watch
	 * @param callback A function object that will be called on change.
	 */
	void on(const std::string &prop, std::function<void(const config::Event&)>);

	protected:
	nlohmann::json &config_;

	private:
	std::map<std::string, std::list<std::function<void(const config::Event&)>>> observers_; 

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

#include <ftl/configuration.hpp>

template <typename T>
std::optional<T> ftl::Configurable::get(const std::string &name) {
	if (!config_[name].is_null()) {
		try {
			return config_[name].get<T>();
		} catch (...) {
			return {};
		}
	} else if (config_["$ref"].is_string()) {
		// TODO(Nick) Add # if missing
		// TODO(Nick) Cache result of ref loopkup
		std::string res_uri = config_["$ref"].get<std::string>()+"/"+name;
		auto &r = ftl::config::resolve(res_uri);

		DLOG(2) << "GET: " << res_uri << " = " << r;

		try {
			return r.get<T>();
		} catch (...) {
			LOG(ERROR) << "Missing: " << config_["$id"].get<std::string>()+"/"+name;
			return {};
		}
	} else {
		return {};
	}
}

template <typename T, typename... ARGS>
T *ftl::Configurable::create(const std::string &name, ARGS ...args) {
	return ftl::config::create<T>(this, name, args...);
}

#endif  // _FTL_CONFIGURABLE_HPP_
