#pragma once
#ifndef _FTL_CONFIGURABLE_HPP_
#define _FTL_CONFIGURABLE_HPP_

//#define LOGURU_REPLACE_GLOG 1
//#include <loguru.hpp>
#include <ftl/exception.hpp>
#include <nlohmann/json_fwd.hpp>
#include <string>
#include <tuple>
#include <map>
#include <list>
#include <functional>
#include <optional>
#include <unordered_set>

#define REQUIRED(...) required(__func__, __VA_ARGS__)

// TODO: Find a better place for this
#define UNUSED(A) (void)(A)

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
	virtual ~Configurable();

	/**
	 * Force the JSON object to have specific properties with a specific type.
	 * If not, emit errors and terminate the application.
	 */
	void required(const char *f, const std::vector<std::tuple<std::string, std::string, std::string>> &r);

	/**
	 * Return raw JSON entity for this Configurable.
	 */
	nlohmann::json &getConfig() { return *config_; }

	/**
	 * Get a configuration property from the json object. Returns an optional
	 * result which will be empty if the property does not exist or is not of
	 * the requested type.
	 */
	template <typename T>
	std::optional<T> get(const std::string &name);

	std::string getID() { return *get<std::string>("$id"); }

	template <typename T>
	inline bool is(const std::string &name) { return false; }

	bool has(const std::string &name) const;

	/**
	 * Get a configuration property, but return a default if not found.
	 */
	template <typename T>
	T value(const std::string &name, const T &def);

	/**
	 * Change a configuration property and trigger any listening event handlers
	 * for that property. Also triggers the global listeners.
	 */
	template <typename T>
	void set(const std::string &name, T value);

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
	void on(const std::string &prop, std::function<void()>);

	/**
	 * Same callback for all properties in set.
	 */
	void onAny(const std::unordered_set<std::string> &props, std::function<void()>);

	template <typename T>
	void on(const std::string &prop, T &v) {
		on(prop, [&v,this,prop]() { v = *this->get<T>(prop); });
	}

	template <typename T>
	void on(const std::string &prop, T &v, const T &def) {
		v = this->value(prop, def);
		on(prop, [&v,this,prop]() { v = *this->get<T>(prop); });
	}

	void patchPtr(nlohmann::json &newcfg) { config_ = &newcfg; }

	/**
	 * Allow configurables to refresh their internal state, perhaps from a
	 * remote source.
	 */
	virtual void refresh();

	/**
	 * Restore configurable properties from session storage using this key.
	 * The key could be the same as configurable ID or perhaps uses another
	 * property such as URI. If restore is used it will also result in a save
	 * when the configurable is destroyed. The key should ideally be unique.
	 * 
	 * The allowed parameter specifies the set of properties that can be saved.
	 */
	void restore(const std::string &key, const std::unordered_set<std::string> &allowed);

	/**
	 * Load defaults from config file. The key represents type information and
	 * many configurables can load from the same key. If load defaults has been
	 * used by the configurable, then it is also called again when the
	 * configurable is reset.
	 */
	void loadDefaults(const std::string &key);

	virtual void reset() {};

	void save();

	protected:
	nlohmann::json *config_;

	virtual void inject(const std::string &name, nlohmann::json &value) {}

	private:
	std::string restore_;
	std::string defaults_;
	std::unordered_set<std::string> save_allowed_;

	typedef std::list<std::function<void()>> ObserverList;
	std::unordered_map<std::string,ObserverList> observers_; 

	void _trigger(const std::string &name);
};

/*template <>
void Configurable::set<const std::string&>(const std::string &name, const std::string &value) {
	config_[name] = value;
	if (name == "uri") __changeURI(value, this);
	_trigger(name);
}*/

}

#include <ftl/configuration.hpp>

extern template float ftl::Configurable::value<float>(const std::string &name, const float &def);
extern template bool ftl::Configurable::value<bool>(const std::string &name, const bool &def);
extern template int ftl::Configurable::value<int>(const std::string &name, const int &def);
extern template unsigned int ftl::Configurable::value<unsigned int>(const std::string &name, const unsigned int &def);
extern template double ftl::Configurable::value<double>(const std::string &name, const double &def);
extern template std::string ftl::Configurable::value<std::string>(const std::string &name, const std::string &def);

extern template std::optional<float> ftl::Configurable::get<float>(const std::string &name);
extern template std::optional<int> ftl::Configurable::get<int>(const std::string &name);
extern template std::optional<double> ftl::Configurable::get<double>(const std::string &name);
extern template std::optional<std::vector<double>> ftl::Configurable::get<std::vector<double>>(const std::string &name);
extern template std::optional<std::string> ftl::Configurable::get<std::string>(const std::string &name);
extern template std::optional<std::vector<std::string>> ftl::Configurable::get<std::vector<std::string>>(const std::string &name);

extern template void ftl::Configurable::set<float>(const std::string &name, float value);
extern template void ftl::Configurable::set<bool>(const std::string &name, bool value);
extern template void ftl::Configurable::set<int>(const std::string &name, int value);
extern template void ftl::Configurable::set<double>(const std::string &name, double value);
extern template void ftl::Configurable::set<const char*>(const std::string &name, const char *value);
extern template void ftl::Configurable::set<std::string>(const std::string &name, std::string value);
extern template void ftl::Configurable::set<std::vector<std::string>>(const std::string &name, std::vector<std::string> value);
extern template void ftl::Configurable::set<nlohmann::json>(const std::string &name, nlohmann::json value);

template <> bool ftl::Configurable::is<float>(const std::string &name);
template <> bool ftl::Configurable::is<int>(const std::string &name);
template <> bool ftl::Configurable::is<unsigned int>(const std::string &name);
template <> bool ftl::Configurable::is<std::string>(const std::string &name);
template <> bool ftl::Configurable::is<bool>(const std::string &name);
template <> bool ftl::Configurable::is<double>(const std::string &name);

extern template bool ftl::Configurable::is<float>(const std::string &name);
extern template bool ftl::Configurable::is<int>(const std::string &name);
extern template bool ftl::Configurable::is<unsigned int>(const std::string &name);
extern template bool ftl::Configurable::is<std::string>(const std::string &name);
extern template bool ftl::Configurable::is<bool>(const std::string &name);
extern template bool ftl::Configurable::is<double>(const std::string &name);

template <typename T, typename... ARGS>
T *ftl::Configurable::create(const std::string &name, ARGS ...args) {
	return ftl::config::create<T>(this, name, args...);
}

#endif  // _FTL_CONFIGURABLE_HPP_
