#include <ftl/configurable.hpp>

using ftl::Configurable;
using std::string;
using std::map;
using std::list;
using std::function;

void Configurable::_trigger(const string &name) {
	auto ix = observers_.find(name);
	if (ix != observers_.end()) {
		for (auto &f : (*ix).second) {
			try {
				f(this, name);
			} catch(...) {
				LOG(ERROR) << "Exception in event handler for '" << name << "'";
			}
		}
	}
}

void Configurable::on(const string &prop, function<void(Configurable*, const string&)> f) {
	auto ix = observers_.find(prop);
	if (ix == observers_.end()) {
		observers_[prop] = {f};
	} else {
		(*ix).second.push_back(f);
	}
}

void Configurable::__changeURI(const string &uri, Configurable *cfg) {

}