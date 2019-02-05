#include "catch.hpp"
#include <ftl/p2p-ra.hpp>

// ---- MOCK THE SOCKET --------------------------------------------------------
namespace ftl {
namespace net {
namespace raw {
	class Socket {
		public:
		int close();

		int send(uint32_t service, std::string &data);
		//int send(uint32_t service, std::ostringstream &data);
		//int send(uint32_t service, void *data, int length);

		bool isConnected() { return true; };

		void onMessage(sockdatahandler_t handler) { m_handler = handler; }
		//void onError(sockerrorhandler_t handler) {}
		//void onConnect(sockconnecthandler_t handler) {}
		//void onDisconnect(sockdisconnecthandler_t handler) {}
	};
}
}
}
// -----------------------------------------------------------------------------

SCENARIO( "Can get a remote array object", "[array]" ) {
	GIVEN( "a valid uri" ) {
		Array a = ftl::p2p::get("ftl://utu.fi/array/test1");
		REQUIRE( a.isValid() );
	}
}

