#include "catch.hpp"
#include <iostream>
#include <memory>
#include <map>

#include <ftl/net/protocol.hpp>
#include <ftl/net/socket.hpp>

using ftl::net::Socket;

// --- Mock --------------------------------------------------------------------

class MockSocket : public Socket {
	public:
	MockSocket() : Socket(0) {}
	void mock_data() { data(); }
};

static std::string last_rpc;
void ftl::net::Protocol::dispatchRPC(Socket &s, const std::string &d) {
	last_rpc = d;
	
	// This should send a return value
}

void ftl::net::Protocol::dispatchRaw(uint32_t service, Socket &s) {

}

ftl::net::Protocol::Protocol(const std::string &id) {
}

ftl::net::Protocol::~Protocol() {
}

ftl::net::Protocol *ftl::net::Protocol::find(const std::string &p) {
	return NULL;
}

// --- Support -----------------------------------------------------------------

static std::map<int, std::string> fakedata;

void fake_send(int sd, uint32_t  service, const std::string &data) {
	//std::cout << "HEX SEND: " << hexStr(data) << std::endl;
	char buf[8+data.size()];
	ftl::net::Header *h = (ftl::net::Header*)&buf;
	h->size = data.size()+4;
	h->service = service;
	std::memcpy(&buf[8],data.data(),data.size());
	fakedata[sd] = std::string(&buf[0], 8+data.size());
	
	//std::cout << "HEX SEND2: " << hexStr(fakedata[sd]) << std::endl;
}

extern ssize_t recv(int sd, void *buf, size_t n, int f) {
	if (fakedata.count(sd) == 0) {
		std::cout << "Unrecognised socket" << std::endl;
		return 0;
	}
	
	int l = fakedata[sd].size();
	
	std::memcpy(buf, fakedata[sd].c_str(), l);
	fakedata.erase(sd);
	return l;
}

extern ssize_t writev(int sd, const struct iovec *v, int cnt) {
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

uint32_t get_service(int sd) {
	auto h = (ftl::net::Header*)fakedata[sd].data();
	return h->service;
}

size_t get_size(int sd) {
	auto h = (ftl::net::Header*)fakedata[sd].data();
	return h->size-4;
}

template <typename T>
T get_value(int sd) {
	auto h = (T*)(fakedata[sd].data()+sizeof(ftl::net::Header));
	return *h;
}

template <>
std::string get_value(int sd) {
	return std::string((char*)(fakedata[sd].data()+sizeof(ftl::net::Header)),get_size(sd));
}

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

// --- Files to test -----------------------------------------------------------

#include "../src/socket.cpp"

// --- Tests -------------------------------------------------------------------

using ftl::net::Protocol;

TEST_CASE("Socket::call()", "[rpc]") {
	MockSocket s;
	
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
}

TEST_CASE("Socket receive RPC", "[rpc]") {
	MockSocket s;
	auto p = new Protocol("ftl://utu.fi");
	s.setProtocol(p);
	
	SECTION("no argument call") {		
		// Do a fake send
		auto args_obj = std::make_tuple();
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
	
		fake_send(0, FTL_PROTOCOL_RPC, buf.str());
		s.mock_data(); // Force it to read the fake send...
		
		REQUIRE( (last_rpc == buf.str()) );
	}
	
	SECTION("one argument call") {		
		// Do a fake send
		auto args_obj = std::make_tuple(55);
		auto call_obj = std::make_tuple(0,0,"test2",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
	
		fake_send(0, FTL_PROTOCOL_RPC, buf.str());
		s.mock_data(); // Force it to read the fake send...
		
		REQUIRE( (last_rpc == buf.str()) );
	}
}

TEST_CASE("Socket::operator>>()", "[io]") {
	MockSocket s;
	
	SECTION("stream ints") {
		int i[2];
		i[0] = 99;
		i[1] = 101;
		fake_send(0, 100, std::string((char*)&i,2*sizeof(int)));
		
		i[0] = 0;
		i[1] = 0;
		s.mock_data(); // Force a message read, but no protocol...
		
		REQUIRE( (s.size() == 2*sizeof(int)) );
		s >> i;
		REQUIRE( (i[0] == 99) );
		REQUIRE( (i[1] == 101) );
	}
}

TEST_CASE("Socket::send()", "[io]") {
	MockSocket s;
	
	SECTION("send an int") {
		int i = 607;
		s.send(100,i);
		
		REQUIRE( (get_service(0) == 100) );
		REQUIRE( (get_size(0) == sizeof(int)) );
		REQUIRE( (get_value<int>(0) == 607) );
	}
	
	SECTION("send a string") {
		std::string str("hello world");
		s.send(100,str);
		
		REQUIRE( (get_service(0) == 100) );
		REQUIRE( (get_size(0) == str.size()) );
		REQUIRE( (get_value<std::string>(0) == "hello world") );
	}
	
	SECTION("send const char* string") {
		s.send(100,"hello world");
		
		REQUIRE( (get_service(0) == 100) );
		REQUIRE( (get_size(0) == 11) );
		REQUIRE( (get_value<std::string>(0) == "hello world") );
	}
	
	SECTION("send const char* array") {
		s.send(100,ftl::net::array{"hello world",10});
		
		REQUIRE( (get_service(0) == 100) );
		REQUIRE( (get_size(0) == 10) );
		REQUIRE( (get_value<std::string>(0) == "hello worl") );
	}
	
	SECTION("send a tuple") {
		auto tup = std::make_tuple(55,66,true,6.7);
		s.send(100,tup);
		
		REQUIRE( (get_service(0) == 100) );
		REQUIRE( (get_size(0) == sizeof(tup)) );
		REQUIRE( (get_value<decltype(tup)>(0) == tup) );
	}
	
	SECTION("send multiple strings") {
		std::string str("hello ");
		std::string str2("world");
		s.send(100,str,str2);
		
		REQUIRE( (get_service(0) == 100) );
		REQUIRE( (get_size(0) == str.size()+str2.size()) );
		REQUIRE( (get_value<std::string>(0) == "hello world") );
	}
}

TEST_CASE("Socket::read()", "[io]") {
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
}

