#ifndef _FTL_P2P_RM_HPP_
#define _FTL_P2P_RM_HPP_

#include <ftl/p2p-rm/cluster.hpp>

namespace ftl {
namespace rm {

	std::shared_ptr<Cluster> cluster(const char *uri);
	
}
}

#endif // _FTL_P2P_RM_HPP_

