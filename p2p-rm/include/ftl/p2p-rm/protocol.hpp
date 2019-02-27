#ifndef _FTL_P2P_RM_PROTOCOL_HPP_
#define _FTL_P2P_RM_PROTOCOL_HPP_

/* To get the service space for p2p */
#include <ftl/net/protocol.hpp>

#define P2P_SYNC				(FTL_PROTOCOL_FREE + 1)
#define P2P_REQUESTOWNERSHIP	(FTL_PROTOCOL_FREE + 2)
#define P2P_FINDOWNER			(FTL_PROTOCOL_FREE + 3)
#define P2P_NOTIFYOWNERSHIP		(FTL_PROTOCOL_FREE + 4)
#define P2P_URISEARCH			(FTL_PROTOCOL_FREE + 5)
#define P2P_PEERSEARCH			(FTL_PROTOCOL_FREE + 6)
#define P2P_RPC_CALL			(FTL_PROTOCOL_FREE + 7)

namespace ftl {
namespace rm {
	struct P2PQuery {
		char guid[16];
		uint8_t ttl;
	};
	
	struct MemOwner {
		char peer[16];
		uint64_t age;
	};
};
};

#endif // _FTL_P2P_RM_PROTOCOL_HPP_

