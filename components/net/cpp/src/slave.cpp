#include <ftl/net/slave.hpp>
#include <loguru.hpp>

using ftl::Configurable;
using ftl::net::Universe;
using ftl::net::Slave;

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

	loguru::add_callback("net_log", netLog, net, loguru::Verbosity_INFO);
}

Slave::~Slave() {

}
