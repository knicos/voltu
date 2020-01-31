#include <ftl/master.hpp>
#include <ftl/net_configurable.hpp>
#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#include <nlohmann/json.hpp>

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
	// Init system state
	state_.paused = false;

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

	net->bind("pause", [this]() {
		state_.paused = !state_.paused;
	});

	net->bind("update_cfg", [](const std::string &uri, const std::string &value) {
		ftl::config::update(uri, nlohmann::json::parse(value));
	});

	net->bind("get_cfg", [](const std::string &uri) -> std::string {
		return ftl::config::resolve(uri, false).dump();
	});

	net->bind("get_configurable", [](const std::string &uri) -> std::string {
		return ftl::config::find(uri)->getConfig().dump();
	});

	net->bind("list_configurables", []() {
		return ftl::config::list();
	});

	net->bind("log_subscribe", [this](const ftl::UUID &peer) {
		UNIQUE_LOCK(mutex_, lk);
		log_peers_.push_back(peer);
	});

	net->bind("connect", [this](const std::string &url) {
		net_->connect(url);
	});

	net->bind("log", [this](int v, const std::string &pre, const std::string &msg) {
		for (auto f : log_handlers_) {
			f({v,pre,msg});
		}
	});

	net->bind("node_details", [net,root]() -> std::vector<std::string> {
		ftl::config::json_t json {
			{"id", net->id().to_string()},
			{"title", root->value("title", *root->get<string>("$id"))}
		};
		return {json.dump()};
	});

	//net->broadcast("log_subscribe", net->id());

	net->onConnect([this](ftl::net::Peer *p) {
		//net_->broadcast("log_subscribe", net_->id());
		ftl::UUID peer = p->id();
		auto cs = getConfigurables(peer);
		for (auto c : cs) {
			//LOG(INFO) << "NET CONFIG: " << c;
			ftl::config::json_t *configuration = new ftl::config::json_t;
			*configuration = getConfigurable(peer, c);
			if (!configuration->empty()) {
				ftl::NetConfigurable *nc = new ftl::NetConfigurable(peer, c, *this, *configuration);
				peerConfigurables_[peer].push_back(nc);
			}
		}
	});

	net->onDisconnect([this](ftl::net::Peer *p) {
		ftl::UUID peer = p->id();
		for (ftl::NetConfigurable *nc : peerConfigurables_[peer]) {
			ftl::config::json_t *configuration = &(nc->getConfig());
			delete nc;
			delete configuration;
		}
	});
}

Master::~Master() {
	stop();
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

void Master::pause() {
	net_->broadcast("pause");
}

void Master::pause(const ftl::UUID &peer) {
	net_->send(peer, "pause");
}

void Master::set(const string &uri, json_t &value) {
	net_->broadcast("update_cfg", uri, (string)value);
}

void Master::set(const ftl::UUID &peer, const string &uri, const json_t &value) {
	LOG(INFO) << "CHANGE: " << uri;
	net_->send(peer, "update_cfg", uri, value.dump());
}

vector<json_t> Master::getControllers() {
	auto response = net_->findAll<string>("node_details");
	vector<json_t> result;
	for (auto &r : response) {
		result.push_back(json_t::parse(r));
		LOG(INFO) << "Node details: " << result[result.size()-1];
	}
	return result;
}

vector<string> Master::getConfigurables() {
	return {};
}

vector<string> Master::getConfigurables(const ftl::UUID &peer) {
	try {
		LOG(INFO) << "LISTING CONFIGS";
		return net_->call<vector<string>>(peer, "list_configurables");
	} catch (...) {
		return {};
	}
}

vector<json_t> Master::get(const string &uri) {
	return {};
}

json_t Master::getOne(const string &uri) {
	return {};
}

json_t Master::get(const ftl::UUID &peer, const string &uri) {
	return json_t::parse(net_->call<string>(peer, "get_cfg", uri));
}

json_t Master::getConfigurable(const ftl::UUID &peer, const string &uri) {
	return json_t::parse(net_->call<string>(peer, "get_configurable", uri));
}

void Master::getConfigurable(json_t &cfg, const ftl::UUID &peer, const string &uri) {
	cfg = json_t::parse(net_->call<string>(peer, "get_configurable", uri));
}

void Master::watch(const string &uri, function<void()> f) {

}

Eigen::Matrix4d Master::getPose(const std::string &uri) {
	auto r = net_->findOne<vector<unsigned char>>("get_pose", uri);
	if (r) {
		Eigen::Matrix4d pose;
		memcpy(pose.data(), (*r).data(), (*r).size());
		return pose;
	} else {
		LOG(WARNING) << "No pose found for " << uri;
		Eigen::Matrix4d pose;
		pose.setIdentity();
		return pose;
	}
}

void Master::setPose(const std::string &uri, const Eigen::Matrix4d &pose) {
	vector<unsigned char> vec((unsigned char*)pose.data(), (unsigned char*)(pose.data()+(pose.size())));
	net_->broadcast("set_pose", uri, vec);
}

// Events

//void onError();
void Master::onLog(function<void(const LogEvent &)> h) {
	log_handlers_.push_back(h);
}

void Master::stop() {
	if (!active_) return;
	active_ = false;
	loguru::remove_all_callbacks();
	net_->unbind("restart");
	net_->unbind("shutdown");
	net_->unbind("update_cfg");
	net_->unbind("get_cfg");
	net_->unbind("slave_details"); // TODO: Remove
	net_->unbind("log_subscribe");
	net_->unbind("log");
}

/*void Master::sendLog(const loguru::Message& message) {
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
}*/