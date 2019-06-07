#ifndef _FTL_CTRL_SLAVE_HPP_
#define _FTL_CTRL_SLAVE_HPP_

#include <ftl/net/universe.hpp>
#include <ftl/configurable.hpp>
#include <loguru.hpp>
#include <mutex>

namespace ftl {
namespace ctrl {

/**
 * Allows a node to be remote controlled and observed over the network. All
 * such nodes should create a single instance of this class, but must call
 * "stop()" before terminating the network.
 */
class Slave {
	public:
	Slave(ftl::net::Universe *, ftl::Configurable *);
	~Slave();

	/**
	 * Clean up to remove log and status forwarding over the network.
	 */
	void stop();

	/**
	 * Do not call! Automatically called from logging subsystem.
	 */
	void sendLog(const loguru::Message& message);

	private:
	std::vector<ftl::UUID> log_peers_;
	ftl::net::Universe *net_;
	std::recursive_mutex mutex_;
	bool in_log_;
	bool active_;
};

}
}

#endif  // _FTL_CTRL_SLAVE_HPP_
