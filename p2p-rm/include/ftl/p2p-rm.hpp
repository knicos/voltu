#ifndef _FTL_P2P_RM_HPP_
#define _FTL_P2P_RM_HPP_

#include <ftl/p2p-rm/cluster.hpp>
#include <memory>

namespace ftl {
namespace net {
class Listener;
};

namespace rm {

	std::shared_ptr<Cluster> cluster(const char *uri, std::shared_ptr<ftl::net::Listener> l);
	
}
}

#endif // _FTL_P2P_RM_HPP_

