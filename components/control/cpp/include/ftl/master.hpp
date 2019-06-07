#ifndef _FTL_CTRL_MASTER_HPP_
#define _FTL_CTRL_MASTER_HPP_

#include <ftl/net/universe.hpp>
#include <ftl/configurable.hpp>
#include <ftl/uuid.hpp>
#include <functional>
#include <string>
#include <vector>

namespace ftl {
namespace ctrl {

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

	void set(const std::string &uri, ftl::config::json_t &value);

	void set(const ftl::UUID &peer, const std::string &uri, ftl::config::json_t &value);

	std::vector<std::string> getConfigurables();

	std::vector<std::string> getConfigurables(const ftl::UUID &peer);

	std::vector<ftl::config::json_t> getSlaves();

	std::vector<ftl::config::json_t> get(const std::string &uri);

	ftl::config::json_t getOne(const std::string &uri);

	ftl::config::json_t get(const ftl::UUID &peer, const std::string &uri);

	void watch(const std::string &uri, std::function<void()> f);

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
};

}
}

#endif  // _FTL_CTRL_MASTER_HPP_
