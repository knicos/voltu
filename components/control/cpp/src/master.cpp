#include <ftl/master.hpp>

using ftl::ctrl::Master;
using ftl::net::Universe;
using ftl::Configurable;
using std::string;
using ftl::config::json_t;
using std::vector;
using std::function;
using ftl::ctrl::LogEvent;

Master::Master(Configurable *root, Universe *net)
		: root_(root), net_(net) {
	net_->bind("log", [this](const std::string &pre, const std::string &msg) {
		for (auto f : log_handlers_) {
			f({pre,msg});
		}
	});
}

Master::~Master() {
	net_->unbind("log");
}

void Master::restart() {
	net_->broadcast("restart");
}

void Master::restart(const ftl::UUID &peer) {
	net_->send(peer, "restart");
}

void Master::shutdown() {
	net_->broadcast("shutdown");
}

void Master::shutdown(const ftl::UUID &peer) {
	net_->send(peer, "shutdown");
}

void Master::set(const string &uri, json_t &value) {
	net_->broadcast("update_cfg", uri, (string)value);
}

void Master::set(const ftl::UUID &peer, const string &uri, json_t &value) {
	net_->send(peer, "update_cfg", uri, (string)value);
}

vector<string> Master::getConfigurables() {
	return {};
}

vector<string> Master::getConfigurables(const ftl::UUID &peer) {
	return {};
}

vector<json_t> Master::get(const string &uri) {
	return {};
}

json_t Master::getOne(const string &uri) {
	return {};
}

json_t Master::get(const ftl::UUID &peer, const string &uri) {
	return {};
}

void Master::watch(const string &uri, function<void()> f) {

}

// Events

//void onError();
void Master::onLog(function<void(const LogEvent &)> h) {
	log_handlers_.push_back(h);
}