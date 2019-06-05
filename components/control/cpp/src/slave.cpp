#include <ftl/slave.hpp>
#include <loguru.hpp>

using ftl::Configurable;
using ftl::net::Universe;
using ftl::ctrl::Slave;

static void netLog(void* user_data, const loguru::Message& message) {
	Universe *net = (Universe*)user_data;
	net->publish("log", message.preamble, message.message);
}

Slave::Slave(Universe *net, ftl::Configurable *root) {
	net->bind("restart", []() {
		LOG(WARNING) << "Remote restart...";
		exit(1);
	});

	net->bind("shutdown", []() {
		LOG(WARNING) << "Remote shutdown...";
		exit(0);
	});

	net->bind("update_cfg", [](const std::string &uri, const std::string &value) {
		ftl::config::update(uri, nlohmann::json::parse(value));
	});

	net->bind("get_cfg", [](const std::string &uri) -> std::string {
		return ftl::config::resolve(uri);
	});

	loguru::add_callback("net_log", netLog, net, loguru::Verbosity_INFO);
}

Slave::~Slave() {

}
