#include "catch.hpp"
#include <iostream>
#include <memory>
//#include <map>
#include <tuple>

#include <ftl/net/peer.hpp>
#include <ftl/net/protocol.hpp>
#include <ftl/config.h>

/* Allow socket functions to be mocked */
#define TEST_MOCKS
#include "../src/net_internal.hpp"

using std::tuple;
using std::get;
using ftl::net::Peer;

#ifdef WIN32
#pragma comment(lib, "Ws2_32.lib")
#endif

// --- Mock --------------------------------------------------------------------

class MockPeer : public Peer {
	public:
	MockPeer() : Peer(0) {}
	void mock_data() { data(); }
};

// --- Support -----------------------------------------------------------------

static std::map<int, std::string> fakedata;

#ifdef WIN32
int ftl::net::internal::recv(SOCKET sd, char *buf, int n, int f) {
#else
ssize_t ftl::net::internal::recv(int sd, void *buf, size_t n, int f) {
#endif
	if (fakedata.count(sd) == 0) {
		std::cout << "Unrecognised socket" << std::endl;
		return 0;
	}
	
	int l = fakedata[sd].size();
	
	std::memcpy(buf, fakedata[sd].c_str(), l);
	fakedata.erase(sd);
	return l;
}

#ifdef WIN32
int ftl::net::internal::send(SOCKET sd, const char *v, int cnt, int flags) {
	int len = cnt;
	// TODO(nick) merge multiple sends
	fakedata[sd] = std::string(v, len);
	return len;
}
#else
ssize_t ftl::net::internal::writev(int sd, const struct iovec *v, int cnt) {
	size_t len = 0; //v[0].iov_len+v[1].iov_len;
	char buf[1000];
	char *bufp = &buf[0];
	
	for (auto i=0; i<cnt; i++) {
		std::memcpy(bufp,v[i].iov_base,v[i].iov_len);
		len += v[i].iov_len;
		bufp += v[i].iov_len;
	}
	
	fakedata[sd] = std::string(&buf[0], len);
	return len;
}
#endif


static std::function<void()> waithandler;

namespace ftl {
namespace net {
bool wait() {
	if (waithandler) waithandler();
	//waithandler = nullptr;
	return true;
}
};
};

/*void fake_send(int sd, uint32_t  service, ARGS) {
	//std::cout << "HEX SEND: " << hexStr(data) << std::endl;
	char buf[8+1024];
	assert(data.size() < 1024);
	ftl::net::Header *h = (ftl::net::Header*)&buf;
	h->size = data.size()+4;
	h->service = service;
	std::memcpy(&buf[8],data.data(),data.size());
	fakedata[sd] = std::string(&buf[0], 8+data.size());
	
	//std::cout << "HEX SEND2: " << hexStr(fakedata[sd]) << std::endl;
}*/

void send_handshake(Peer &p) {
	ftl::UUID id;
	p.send("__handshake__", ftl::net::kMagic, ((8 << 16) + (5 << 8) + 2), id);
}

template <typename T>
tuple<std::string, T> readResponse(int s) {
	msgpack::object_handle msg = msgpack::unpack(fakedata[s].data(), fakedata[s].size());
	tuple<uint8_t, std::string, T> req;
	msg.get().convert(req);
	return std::make_tuple(get<1>(req), get<2>(req));
}

// --- Files to test -----------------------------------------------------------

#include "../src/peer.cpp"

// --- Tests -------------------------------------------------------------------

TEST_CASE("Peer(int)", "[]") {
	SECTION("initiates a valid handshake") {
		MockPeer s;

		auto [name, hs] = readResponse<ftl::net::Handshake>(0);
		
		REQUIRE( name == "__handshake__" );
		
		// 1) Sends magic (64bits)
		REQUIRE( get<0>(hs) == ftl::net::kMagic );
		 
		// 2) Sends FTL Version
		REQUIRE( get<1>(hs) == (FTL_VERSION_MAJOR << 16) + (FTL_VERSION_MINOR << 8) + FTL_VERSION_PATCH );
		
		// 3) Sends peer UUID
		
		
		REQUIRE( s.status() == Peer::kConnecting );
	}
	
	SECTION("completes on full handshake") {
		MockPeer s;
		
		// Send handshake response
		send_handshake(s);
		s.mock_data();
		
		REQUIRE( s.status() == Peer::kConnected );
	}
	
	SECTION("has correct version on full handshake") {
		MockPeer s;
		
		// Send handshake response
		send_handshake(s);
		s.mock_data();
		
		REQUIRE( s.getFTLVersion() ==  (8 << 16) + (5 << 8) + 2 );
	}
	
	SECTION("has correct peer id on full handshake") {
		MockPeer s;
		
		// Send handshake response
		
		//REQUIRE( s.id() ==   );
	}
}

/*TEST_CASE("Peer::call()", "[rpc]") {
	MockPeer s;
	
	SECTION("no argument call") {
		waithandler = [&]() {
			// Read fakedata sent
			// TODO Validate data
			
			// Do a fake send
			auto res_obj = std::make_tuple(1,0,msgpack::object(),66);
			std::stringstream buf;
			msgpack::pack(buf, res_obj);
		
			fake_send(0, FTL_PROTOCOL_RPCRETURN, buf.str());
			s.mock_data();
		};
		
		int res = s.call<int>("test1");
		
		REQUIRE( (res == 66) );
	}
	
	SECTION("one argument call") {
		waithandler = [&]() {
			// Read fakedata sent
			// TODO Validate data
			
			// Do a fake send
			auto res_obj = std::make_tuple(1,1,msgpack::object(),43);
			std::stringstream buf;
			msgpack::pack(buf, res_obj);
		
			fake_send(0, FTL_PROTOCOL_RPCRETURN, buf.str());
			s.mock_data();
		};
		
		int res = s.call<int>("test1", 78);
		
		REQUIRE( (res == 43) );
	}
	
	waithandler = nullptr;
}*/

TEST_CASE("Peer::bind()", "[rpc]") {
	MockPeer s;
	
	SECTION("no argument call") {
		bool done = false;
		
		s.bind("hello", [&]() {
			done = true;
		});

		send_handshake(s);
		s.mock_data();
		s.send("hello");
		s.mock_data(); // Force it to read the fake send...
		
		REQUIRE( done );
	}
	
	SECTION("one argument call") {		
		int done = 0;
		
		s.bind("hello", [&](int a) {
			done = a;
		});
		
		send_handshake(s);	
		s.mock_data();
		s.send("hello", 55);
		s.mock_data(); // Force it to read the fake send...
		
		REQUIRE( (done == 55) );
	}
	
	SECTION("two argument call") {		
		std::string done;
		
		s.bind("hello", [&](int a, std::string b) {
			done = b;
		});
		
		send_handshake(s);	
		s.mock_data();
		s.send("hello", 55, "world");
		s.mock_data(); // Force it to read the fake send...
		
		REQUIRE( (done == "world") );
	}
}

TEST_CASE("Socket::send()", "[io]") {
	MockPeer s;
	
	SECTION("send an int") {
		int i = 607;
		s.send("dummy",i);
		
		auto [name, value] = readResponse<tuple<int>>(0);
		
		REQUIRE( (name == "dummy") );
		REQUIRE( (get<0>(value) == 607) );
	}
	
	SECTION("send a string") {
		std::string str("hello world");
		s.send("dummy",str);
		
		auto [name, value] = readResponse<tuple<std::string>>(0);
		
		REQUIRE( (name == "dummy") );
		REQUIRE( (get<0>(value) == "hello world") );
	}
	
	SECTION("send const char* string") {
		s.send("dummy","hello world");
		
		auto [name, value] = readResponse<tuple<std::string>>(0);
		
		REQUIRE( (name == "dummy") );
		REQUIRE( (get<0>(value) == "hello world") );
	}
	
	/*SECTION("send const char* array") {
		s.send(100,ftl::net::array{"hello world",10});
		
		REQUIRE( (get_service(0) == 100) );
		REQUIRE( (get_size(0) == 10) );
		REQUIRE( (get_value<std::string>(0) == "hello worl") );
	}*/
	
	SECTION("send a tuple") {
		auto tup = std::make_tuple(55,66,true,6.7);
		s.send("dummy",tup);
		
		auto [name, value] = readResponse<tuple<decltype(tup)>>(0);
		
		REQUIRE( (name == "dummy") );
		REQUIRE( (get<1>(get<0>(value)) == 66) );
	}
	
	SECTION("send multiple strings") {
		std::string str("hello ");
		std::string str2("world");
		s.send("dummy2",str,str2);
		
		auto [name, value] = readResponse<tuple<std::string,std::string>>(0);
		
		REQUIRE( (name == "dummy2") );
		REQUIRE( (get<0>(value) == "hello") );
		REQUIRE( (get<1>(value) == "world") );
	}
}

/*TEST_CASE("Socket::read()", "[io]") {
	MockSocket s;
	
	SECTION("read an int") {
		int i = 99;
		fake_send(0, 100, std::string((char*)&i,4));
		
		i = 0;
		s.mock_data(); // Force a message read, but no protocol...
		
		REQUIRE( (s.size() == sizeof(int)) );
		REQUIRE( (s.read(i) == sizeof(int)) );
		REQUIRE( (i == 99) );
	}
	
	SECTION("read two ints") {
		int i[2];
		i[0] = 99;
		i[1] = 101;
		fake_send(0, 100, std::string((char*)&i,2*sizeof(int)));
		
		i[0] = 0;
		i[1] = 0;
		s.mock_data(); // Force a message read, but no protocol...
		
		REQUIRE( (s.size() == 2*sizeof(int)) );
		REQUIRE( (s.read(&i,2) == 2*sizeof(int)) );
		REQUIRE( (i[0] == 99) );
		REQUIRE( (i[1] == 101) );
	}
	
	SECTION("multiple reads") {
		int i[2];
		i[0] = 99;
		i[1] = 101;
		fake_send(0, 100, std::string((char*)&i,2*sizeof(int)));
		
		i[0] = 0;
		i[1] = 0;
		s.mock_data(); // Force a message read, but no protocol...
		
		REQUIRE( (s.read(&i[0],1) == sizeof(int)) );
		REQUIRE( (i[0] == 99) );
		REQUIRE( (s.read(&i[1],1) == sizeof(int)) );
		REQUIRE( (i[1] == 101) );
	}
	
	SECTION("read a string") {
		std::string str;
		fake_send(0, 100, std::string("hello world"));
		
		s.mock_data(); // Force a message read, but no protocol...
		
		REQUIRE( (s.size() == 11) );
		REQUIRE( (s.read(str) == 11) );
		REQUIRE( (str == "hello world") );
	}
	
	SECTION("read into existing string") {
		std::string str;
		str.reserve(11);
		void *ptr = str.data();
		fake_send(0, 100, std::string("hello world"));
		
		s.mock_data(); // Force a message read, but no protocol...
		
		REQUIRE( (s.size() == 11) );
		REQUIRE( (s.read(str) == 11) );
		REQUIRE( (str == "hello world") );
		REQUIRE( (str.data() == ptr) );
	}
	
	SECTION("read too much data") {
		int i = 99;
		fake_send(0, 100, std::string((char*)&i,4));
		
		i = 0;
		s.mock_data(); // Force a message read, but no protocol...
		
		REQUIRE( (s.size() == sizeof(int)) );
		REQUIRE( (s.read(&i,2) == sizeof(int)) );
		REQUIRE( (i == 99) );
	}
}*/

