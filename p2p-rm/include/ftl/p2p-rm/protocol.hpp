#ifndef _FTL_P2P_RM_PROTOCOL_HPP_
#define _FTL_P2P_RM_PROTOCOL_HPP_

/* To get the service space for p2p */
#include <ftl/net/protocol.hpp>

#define P2P_SYNC				(FTL_PROTOCOL_P2P + 1)
#define P2P_REQUESTOWNERSHIP	(FTL_PROTOCOL_P2P + 2)
#define P2P_FINDOWNER			(FTL_PROTOCOL_P2P + 3)
#define P2P_NOTIFYOWNERSHIP		(FTL_PROTOCOL_P2P + 4)
#define P2P_URISEARCH			(FTL_PROTOCOL_P2P + 5)
#define P2P_PEERSEARCH			(FTL_PROTOCOL_P2P + 6)

#endif // _FTL_P2P_RM_PROTOCOL_HPP_

