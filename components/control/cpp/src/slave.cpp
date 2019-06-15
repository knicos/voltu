#include <ftl/slave.hpp>

#include <ftl/threads.hpp>

using ftl::Configurable;
using ftl::net::Universe;
using ftl::ctrl::Slave;
using std::string;

static void netLog(void* user_data, const loguru::Message& message) {
	Slave *slave = static_cast<Slave*>(user_data);
	slave->sendLog(message);
}

Slave::Slave(Universe *net, ftl::Configurable *root) : net_(net), in_log_(false), active_(true) {
	net->bind("restart", []() {
		LOG(WARNING) << "Remote restart...";
		//exit(1);
		ftl::exit_code = 1;
		ftl::running = false;
	});

	net->bind("shutdown", []() {
		LOG(WARNING) << "Remote shutdown...";
		//exit(0);
		ftl::running = false;
	});

	net->bind("update_cfg", [](const std::string &uri, const std::string &value) {
		ftl::config::update(uri, nlohmann::json::parse(value));
	});

	net->bind("get_cfg", [](const std::string &uri) -> std::string {
		return ftl::config::resolve(uri);
	});

	net->bind("node_details", [net,root]() -> std::vector<std::string> {
		ftl::config::json_t json {
			{"id", net->id().to_string()},
			{"title", root->value("title", *root->get<string>("$id"))},
			{"kind", "slave"}
		};
		return {json.dump()};
	});

	net->bind("log_subscribe", [this](const ftl::UUID &peer) {
		UNIQUE_LOCK(mutex_, lk);
		log_peers_.push_back(peer);
	});

	net->bind("connect", [this](const std::string &url) {
		net_->connect(url);
	});

	//net->onConnect([this](ftl::net::Peer *peer) {
	//	net_->broadcast("new_peer", peer->id());
	//});

	//loguru::add_callback("net_log", netLog, this, loguru::Verbosity_INFO);
}

Slave::~Slave() {
	stop();
}

void Slave::stop() {
	if (!active_) return;
	active_ = false;
	loguru::remove_all_callbacks();
	net_->unbind("restart");
	net_->unbind("shutdown");
	net_->unbind("update_cfg");
	net_->unbind("get_cfg");
	net_->unbind("slave_details");
	net_->unbind("log_subscribe");
}

void Slave::sendLog(const loguru::Message& message) {
	UNIQUE_LOCK(mutex_, lk);
	if (in_log_) return;
	in_log_ = true;

	for (auto &p : log_peers_) {
		auto peer = net_->getPeer(p);
		if (!peer || !peer->isConnected()) continue;

		std::cout << "sending log to master..." << std::endl;
		if (!net_->send(p, "log", message.verbosity, message.preamble, message.message)) {
			// TODO(Nick) Remove peer from loggers list...
		}
	}

	in_log_ = false;
}
