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
	net->bind("log", [this](int v, const std::string &pre, const std::string &msg) {
		for (auto f : log_handlers_) {
			f({v,pre,msg});
		}
	});

	net->bind("node_details", [net,root]() -> std::vector<std::string> {
		ftl::config::json_t json {
			{"id", net->id().to_string()},
			{"title", root->value("title", *root->get<string>("$id"))},
			{"kind", "master"}
		};
		return {json.dump()};
	});

	//net->broadcast("log_subscribe", net->id());

	net->onConnect([this](ftl::net::Peer*) {
		//net_->broadcast("log_subscribe", net_->id());
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

vector<json_t> Master::getSlaves() {
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