#ifndef _FTL_NET_UNIVERSE_HPP_
#define _FTL_NET_UNIVERSE_HPP_

#include <ftl/net/peer.hpp>
#include <ftl/net/listener.hpp>
#include <vector>
#include <string>
#include <thread>

namespace ftl {
namespace net {

class Universe {
	public:
	explicit Universe(const std::string &base);
	~Universe();
	
	bool listen(const std::string &addr);
	bool connect(const std::string &addr);
	
	private:
	void _run();
	int _setDescriptors();
	void _installBindings(Peer *);
	
	static void __start(Universe *u);
	
	private:
	bool active_;
	std::string base_;
	std::thread thread_;
	fd_set sfderror_;
	fd_set sfdread_;
	std::vector<ftl::net::Listener*> listeners_;
	std::vector<ftl::net::Peer*> peers_;
};

};  // namespace net
};  // namespace ftl

#endif  // _FTL_NET_UNIVERSE_HPP_
