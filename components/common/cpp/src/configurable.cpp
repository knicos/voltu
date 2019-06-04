#include <ftl/configurable.hpp>

using ftl::Configurable;
using std::string;
using std::map;
using std::list;
using std::function;
using ftl::config::json_t;

extern nlohmann::json null_json;

Configurable::Configurable() : config_(null_json) {}

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
				f({this, name});
			} catch(...) {
				LOG(ERROR) << "Exception in event handler for '" << name << "'";
			}
		}
	}
}

void Configurable::on(const string &prop, function<void(const ftl::config::Event&)> f) {
	auto ix = observers_.find(prop);
	if (ix == observers_.end()) {
		observers_[prop] = {f};
	} else {
		(*ix).second.push_back(f);
	}
}

void Configurable::__changeURI(const string &uri, Configurable *cfg) {

}