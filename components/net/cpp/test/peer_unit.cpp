#include "catch.hpp"
#include <iostream>
#include <memory>
//#include <map>
#include <vector>
#include <tuple>
#include <thread>
#include <chrono>

#include <ftl/net/common.hpp>
#include <ftl/net/peer.hpp>
#include <ftl/net/protocol.hpp>
#include <ftl/config.h>

#ifdef WIN32
#pragma comment(lib, "Rpcrt4.lib")
#endif

/* Allow socket functions to be mocked */
#define TEST_MOCKS
#include "../src/net_internal.hpp"

using std::tuple;
using std::get;
using std::vector;
using ftl::net::Peer;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;

#ifdef WIN32
#pragma comment(lib, "Ws2_32.lib")
#endif

// --- Mock --------------------------------------------------------------------

class MockPeer : public Peer {
	public:
	MockPeer() : Peer((SOCKET)0) {}
	void mock_data() { data(); }
};

// --- Support -----------------------------------------------------------------

static std::map<SOCKET, std::string> fakedata;

#ifdef WIN32
int ftl::net::internal::recv(SOCKET sd, char *buf, int n, int f) {
#else
ssize_t ftl::net::internal::recv(SOCKET sd, void *buf, size_t n, int f) {
#endif
	if (fakedata.count(sd) == 0) {
		std::cout << "Unrecognised socket" << std::endl;
		return 0;
	}
	
	size_t l = fakedata[sd].size();
	
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

template <typename T>
tuple<uint32_t, T> readRPC(int s) {
	msgpack::object_handle msg = msgpack::unpack(fakedata[s].data(), fakedata[s].size());
	tuple<uint8_t, uint32_t, std::string, T> req;
	msg.get().convert(req);
	return std::make_tuple(get<1>(req), get<3>(req));
}

template <typename T>
T readRPCReturn(int s) {
	msgpack::object_handle msg = msgpack::unpack(fakedata[s].data(), fakedata[s].size());
	tuple<uint8_t, uint32_t, std::string, T> req;
	msg.get().convert(req);
	return get<3>(req);
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
		
		sleep_for(milliseconds(50));
		
		REQUIRE( s.status() == Peer::kConnected );
	}
	
	SECTION("has correct version on full handshake") {
		MockPeer s;
		
		// Send handshake response
		send_handshake(s);
		s.mock_data();
		
		sleep_for(milliseconds(50));
		
		REQUIRE( (s.getFTLVersion() ==  (8 << 16) + (5 << 8) + 2) );
	}
	
	SECTION("has correct peer id on full handshake") {
		MockPeer s;
		
		// Send handshake response
		
		//REQUIRE( s.id() ==   );
	}
}

TEST_CASE("Peer::call()", "[rpc]") {
	MockPeer s;
	send_handshake(s);
	s.mock_data();
	sleep_for(milliseconds(50));
	
	SECTION("one argument call") {
		REQUIRE( s.isConnected() );
		
		fakedata[0] = "";
		
		// Thread to provide response to otherwise blocking call
		std::thread thr([&s]() {
			while (fakedata[0].size() == 0) std::this_thread::sleep_for(std::chrono::milliseconds(20));
			
			auto [id,value] = readRPC<tuple<int>>(0);
			auto res_obj = std::make_tuple(1,id,"__return__",get<0>(value)+22);
			std::stringstream buf;
			msgpack::pack(buf, res_obj);
			fakedata[0] = buf.str();
			s.mock_data();
			sleep_for(milliseconds(50));
		});
		
		int res = s.call<int>("test1", 44);
		
		thr.join();
		
		REQUIRE( (res == 66) );
	}
	
	SECTION("no argument call") {
		REQUIRE( s.isConnected() );
		
		fakedata[0] = "";
		
		// Thread to provide response to otherwise blocking call
		std::thread thr([&s]() {
			while (fakedata[0].size() == 0) std::this_thread::sleep_for(std::chrono::milliseconds(20));
			
			auto [id,value] = readRPC<tuple<>>(0);
			auto res_obj = std::make_tuple(1,id,"__return__",77);
			std::stringstream buf;
			msgpack::pack(buf, res_obj);
			fakedata[0] = buf.str();
			s.mock_data();
			sleep_for(milliseconds(50));
		});
		
		int res = s.call<int>("test1");
		
		thr.join();
		
		REQUIRE( (res == 77) );
	}

	SECTION("vector return from call") {
		REQUIRE( s.isConnected() );
		
		fakedata[0] = "";
		
		// Thread to provide response to otherwise blocking call
		std::thread thr([&s]() {
			while (fakedata[0].size() == 0) std::this_thread::sleep_for(std::chrono::milliseconds(20));
			
			auto [id,value] = readRPC<tuple<>>(0);
			vector<int> data = {44,55,66};
			auto res_obj = std::make_tuple(1,id,"__return__",data);
			std::stringstream buf;
			msgpack::pack(buf, res_obj);
			fakedata[0] = buf.str();
			s.mock_data();
			sleep_for(milliseconds(50));
		});
		
		vector<int> res = s.call<vector<int>>("test1");
		
		thr.join();
		
		REQUIRE( (res[0] == 44) );
		REQUIRE( (res[2] == 66) );
	}
}

TEST_CASE("Peer::bind()", "[rpc]") {
	MockPeer s;
	send_handshake(s);	
	s.mock_data();
	sleep_for(milliseconds(50));
	
	SECTION("no argument call") {
		bool done = false;
		
		s.bind("hello", [&]() {
			done = true;
		});

		s.send("hello");
		s.mock_data(); // Force it to read the fake send...
		sleep_for(milliseconds(50));
		
		REQUIRE( done );
	}
	
	SECTION("one argument call") {		
		int done = 0;
		
		s.bind("hello", [&](int a) {
			done = a;
		});
		
		s.send("hello", 55);
		s.mock_data(); // Force it to read the fake send...
		sleep_for(milliseconds(50));
		
		REQUIRE( (done == 55) );
	}
	
	SECTION("two argument call") {		
		std::string done;
		
		s.bind("hello", [&](int a, std::string b) {
			done = b;
		});

		s.send("hello", 55, "world");
		s.mock_data(); // Force it to read the fake send...
		sleep_for(milliseconds(50));
		
		REQUIRE( (done == "world") );
	}

	SECTION("int return value") {		
		int done = 0;
		
		s.bind("hello", [&](int a) -> int {
			done = a;
			return a;
		});
		
		s.asyncCall<int>("hello", [](int a){}, 55);
		s.mock_data(); // Force it to read the fake send...
		sleep_for(milliseconds(50));
		
		REQUIRE( (done == 55) );
		REQUIRE( (readRPCReturn<int>(0) == 55) );
	}

	SECTION("vector return value") {		
		int done = 0;
		
		s.bind("hello", [&](int a) -> vector<int> {
			done = a;
			vector<int> b = {a,45};
			return b;
		});
		
		s.asyncCall<int>("hello", [](int a){}, 55);
		s.mock_data(); // Force it to read the fake send...
		sleep_for(milliseconds(50));
		
		REQUIRE( (done == 55) );

		auto res = readRPCReturn<vector<int>>(0);
		REQUIRE( (res[1] == 45) );
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
		REQUIRE( (get<0>(value) == "hello ") );
		REQUIRE( (get<1>(value) == "world") );
	}
}

