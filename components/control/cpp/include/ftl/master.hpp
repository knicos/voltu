#ifndef _FTL_CTRL_MASTER_HPP_
#define _FTL_CTRL_MASTER_HPP_

#include <ftl/net/universe.hpp>
#include <ftl/configurable.hpp>
#include <ftl/uuid.hpp>
#include <functional>
#include <string>
#include <vector>
#include <Eigen/Eigen>

namespace ftl {

class NetConfigurable;

namespace ctrl {

struct SystemState {
	bool paused;
};

struct LogEvent {
	int verbosity;
	std::string preamble;
	std::string message;
};

class Master {
	public:
	Master(ftl::Configurable *root, ftl::net::Universe *net);
	~Master();

	void restart();

	void restart(const ftl::UUID &peer);

	void shutdown();

	void shutdown(const ftl::UUID &peer);

	void pause();

	void pause(const ftl::UUID &peer);

	void set(const std::string &uri, ftl::config::json_t &value);

	void set(const ftl::UUID &peer, const std::string &uri, const ftl::config::json_t &value);

	std::vector<std::string> getConfigurables();

	std::vector<std::string> getConfigurables(const ftl::UUID &peer);

	std::vector<ftl::config::json_t> getControllers();

	std::vector<ftl::config::json_t> get(const std::string &uri);

	ftl::config::json_t getOne(const std::string &uri);

	ftl::config::json_t get(const ftl::UUID &peer, const std::string &uri);

	ftl::config::json_t getConfigurable(const ftl::UUID &peer, const std::string &uri);

	void watch(const std::string &uri, std::function<void()> f);

	Eigen::Matrix4d getPose(const std::string &uri);

	void setPose(const std::string &uri, const Eigen::Matrix4d &pose);

	/**
	 * Clean up to remove log and status forwarding over the network.
	 */
	void stop();

	/**
	 * Do not call! Automatically called from logging subsystem.
	 */
	void sendLog(const loguru::Message& message);

	bool isPaused() const { return state_.paused; }

	// Events

	//void onError();
	void onLog(std::function<void(const LogEvent &)>);
	//void onFailure();
	//void onStatus();
	// void onChange();

	ftl::net::Universe *getNet() { return net_; }
	ftl::Configurable *getRoot() { return root_; }

	private:
	std::vector<std::function<void(const LogEvent&)>> log_handlers_;
	ftl::Configurable *root_;
	ftl::net::Universe *net_;
	std::map<ftl::UUID, std::vector<ftl::NetConfigurable*>> peerConfigurables_;
	std::vector<ftl::UUID> log_peers_;
	RECURSIVE_MUTEX mutex_;
	bool in_log_;
	bool active_;
	SystemState state_;
};

}
}

#endif  // _FTL_CTRL_MASTER_HPP_
