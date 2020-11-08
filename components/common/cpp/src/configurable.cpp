#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>
#include <ftl/configurable.hpp>

#include <nlohmann/json.hpp>

using ftl::Configurable;
using std::string;
using std::map;
using std::list;
using std::function;
using ftl::config::json_t;

extern nlohmann::json null_json;

Configurable::Configurable() : config_(&null_json) {
	ftl::config::registerConfigurable(this);
}

Configurable::Configurable(nlohmann::json &config) : config_(&config) {
	if (!config.is_object()) {
		LOG(FATAL) << "Configurable json is not an object: " << config;
	}

	/*if (!config.contains("$id")) {
		config["$id"] = "ftl://utu.fi";
	}*/

	ftl::config::registerConfigurable(this);
}

Configurable::~Configurable() {
	save();
	ftl::config::removeConfigurable(this);
}

void Configurable::save() {
	if (restore_.size() > 0) {
		auto &r = ftl::config::getRestore(restore_);
		for (auto &i : save_allowed_) {
			r[i] = (*config_)[i];
		}
	}
}

void Configurable::restore(const std::string &key, const std::unordered_set<std::string> &allowed) {
	save();
	
	auto &r = ftl::config::getRestore(key);
	if (r.is_object()) {
		config_->merge_patch(r);
		
	}
	restore_ = key;
	save_allowed_ = allowed;
}

template <typename T>
T ftl::Configurable::value(const std::string &name, const T &def) {
	auto r = get<T>(name);
	if (r) return *r;
	(*config_)[name] = def;
	return def;
}

template <typename T>
void ftl::Configurable::set(const std::string &name, T value) {
	(*config_)[name] = value;
	inject(name, (*config_)[name]);
	_trigger(name);
}

template <typename T>
std::optional<T> ftl::Configurable::get(const std::string &name) {
	if (!config_->is_object() && !config_->is_null()) throw FTL_Error("Config is not an object");
	if (!(*config_)[name].is_null()) {
		try {
			return (*config_)[name].get<T>();
		} catch (...) {
			return {};
		}
	} else if ((*config_)["$ref"].is_string()) {
		// FIXME:(Nick) Add # if missing
		// TODO:(Nick) Cache result of ref loopkup
		std::string res_uri = (*config_)["$ref"].get<std::string>()+"/"+name;
		auto &r = ftl::config::resolve(res_uri);

		//DLOG(2) << "GET: " << res_uri << " = " << r;

		try {
			return r.get<T>();
		} catch (...) {
			//throw FTL_Error("Missing: " << (*config_)["$id"].get<std::string>()+"/"+name);
			return {};
		}
	} else {
		return {};
	}
}

template float ftl::Configurable::value<float>(const std::string &name, const float &def);
template bool ftl::Configurable::value<bool>(const std::string &name, const bool &def);
template int ftl::Configurable::value<int>(const std::string &name, const int &def);
template unsigned int ftl::Configurable::value<unsigned int>(const std::string &name, const unsigned int &def);
template double ftl::Configurable::value<double>(const std::string &name, const double &def);
template std::string ftl::Configurable::value<std::string>(const std::string &name, const std::string &def);

template std::optional<float> ftl::Configurable::get<float>(const std::string &name);
template std::optional<int> ftl::Configurable::get<int>(const std::string &name);
template std::optional<double> ftl::Configurable::get<double>(const std::string &name);
template std::optional<std::vector<double>> ftl::Configurable::get<std::vector<double>>(const std::string &name);
template std::optional<std::string> ftl::Configurable::get<std::string>(const std::string &name);
template std::optional<std::vector<std::string>> ftl::Configurable::get<std::vector<std::string>>(const std::string &name);

template void ftl::Configurable::set<float>(const std::string &name, float value);
template void ftl::Configurable::set<bool>(const std::string &name, bool value);
template void ftl::Configurable::set<int>(const std::string &name, int value);
template void ftl::Configurable::set<double>(const std::string &name, double value);
template void ftl::Configurable::set<const char*>(const std::string &name, const char *value);
template void ftl::Configurable::set<std::string>(const std::string &name, std::string value);
template void ftl::Configurable::set<std::vector<std::string>>(const std::string &name, std::vector<std::string> value);
template void ftl::Configurable::set<nlohmann::json>(const std::string &name, nlohmann::json value);

template <>
bool ftl::Configurable::is<float>(const std::string &name) {
	return getConfig()[name].is_number_float();
}

template <>
bool ftl::Configurable::is<int>(const std::string &name) {
	return getConfig()[name].is_number_integer();
}

template <>
bool ftl::Configurable::is<unsigned int>(const std::string &name) {
	return getConfig()[name].is_number_unsigned();
}

template <>
bool ftl::Configurable::is<std::string>(const std::string &name) {
	return getConfig()[name].is_string();
}

template <>
bool ftl::Configurable::is<bool>(const std::string &name) {
	return getConfig()[name].is_boolean();
}

template <>
bool ftl::Configurable::is<double>(const std::string &name) {
	return getConfig()[name].is_number_float();
}

bool ftl::Configurable::has(const std::string &name) const {
	return (config_) ? config_->contains(name) : false;
}

void Configurable::required(const char *f, const std::vector<std::tuple<std::string, std::string, std::string>> &r) {
	bool diderror = false;
	for (auto i : r) {
		auto [name, desc, type] = i;
		auto ent = get<json_t>(name);
		if (!ent || (*ent).type_name() != type) {
			LOG(ERROR) << "Missing required option in \"" << f << "\": \"" << name << "\" - " << desc;
			//LOG(ERROR) << "    Got type " << (*ent).type_name() << " but expected " << type;
			diderror = true;
		}
	}
	if (diderror) LOG(FATAL) << "Cannot continue without required option";
}

void Configurable::_trigger(const string &name) {
	auto ix = observers_.find(name);
	if (ix != observers_.end()) {
		for (auto &f : (*ix).second) {
			try {
				f();
			} catch(...) {
				LOG(ERROR) << "Exception in event handler for '" << name << "'";
			}
		}
	}
}

void Configurable::onAny(const std::unordered_set<string> &props, function<void()> f) {
	for (const auto &p : props) {
		on(p, f);
	}
}

void Configurable::on(const string &prop, function<void()> f) {
	auto ix = observers_.find(prop);
	if (ix == observers_.end()) {
		observers_[prop] = {f};
	} else {
		(*ix).second.push_back(f);
	}
}

void Configurable::refresh() {
	// Do nothing by default
}
