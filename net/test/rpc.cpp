#include "catch.hpp"
#include <ftl/net/socket.hpp>
#include <iostream>
#include <memory>
#include <map>

/*struct FakeHeader {
	uint32_t size;
	uint32_t service;
};*/

static std::map<int, std::string> fakedata;

/*static std::string hexStr(const std::string &s)
{
	const char *data = s.data();
	int len = s.size();
    std::stringstream ss;
    ss << std::hex;
    for(int i=0;i<len;++i)
        ss << std::setw(2) << std::setfill('0') << (int)data[i];
    return ss.str();
}*/

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
	// TODO Use count incase more sources exist...
	size_t len = v[0].iov_len+v[1].iov_len;
	char buf[len];
	std::memcpy(&buf[0],v[0].iov_base,v[0].iov_len);
	std::memcpy(&buf[v[0].iov_len],v[1].iov_base,v[1].iov_len);
	fakedata[sd] = std::string(&buf[0], len);
	return 0;
}

static std::function<void()> waithandler;

bool ftl::net::wait() {
	if (waithandler) waithandler();
	//waithandler = nullptr;
	return true;
}

TEST_CASE("Socket::bind()", "[rpc]") {
	SECTION("no argument bind fake dispatch") {
		auto s = new ftl::net::Socket(0);
		bool called = false;
		
		s->bind("test1", [&]() {
			called = true;
		});
		
		auto args_obj = std::make_tuple();
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		s->dispatchRPC(buf.str());
		REQUIRE( called );
	}
	
	SECTION("one argument bind fake dispatch") {
		auto s = new ftl::net::Socket(0);
		bool called = false;
		
		s->bind("test1", [&](int a) {
			called = true;
			REQUIRE( a == 5 );
		});
		
		auto args_obj = std::make_tuple(5);
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		s->dispatchRPC(buf.str());
		REQUIRE( called );
	}
	
	SECTION("two argument bind fake dispatch") {
		auto s = new ftl::net::Socket(0);
		bool called = false;
		
		s->bind("test1", [&](int a, float b) {
			called = true;
			REQUIRE( a == 5 );
			REQUIRE( b == 5.4f ); // Danger
		});
		
		auto args_obj = std::make_tuple(5, 5.4f);
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		s->dispatchRPC(buf.str());
		REQUIRE( called );
	}
	
	SECTION("no argument bind fake data") {
		auto s = new ftl::net::Socket(0);
		bool called = false;
		
		s->bind("test1", [&]() {
			called = true;
		});
		
		auto args_obj = std::make_tuple();
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		fake_send(0, FTL_PROTOCOL_RPC, buf.str());
		REQUIRE( s->data() );
		REQUIRE( called );
	}
	
	SECTION("non-void bind fake data") {
		auto s = new ftl::net::Socket(0);
		bool called = false;
		
		s->bind("test1", [&]() -> int {
			called = true;
			return 55;
		});
		
		auto args_obj = std::make_tuple();
		auto call_obj = std::make_tuple(0,0,"test1",args_obj);
		std::stringstream buf;
		msgpack::pack(buf, call_obj);
		
		fake_send(0, FTL_PROTOCOL_RPC, buf.str());
		REQUIRE( s->data() );
		REQUIRE( called );
		
		// TODO Require that a writev occurred with result value
	}
}

TEST_CASE("Socket::call()", "[rpc]") {
	SECTION("no argument call") {
		auto s = new ftl::net::Socket(0);
		
		waithandler = [&]() {
			// Read fakedata sent
			// TODO Validate data
			
			// Do a fake send
			auto res_obj = std::make_tuple(1,0,msgpack::object(),66);
			std::stringstream buf;
			msgpack::pack(buf, res_obj);
		
			fake_send(0, FTL_PROTOCOL_RPCRETURN, buf.str());
			s->data();
		};
		
		int res = s->call<int>("test1");
		
		REQUIRE( res == 66 );
	}
	
	SECTION("one argument call") {
		auto s = new ftl::net::Socket(0);
		
		waithandler = [&]() {
			// Read fakedata sent
			// TODO Validate data
			
			// Do a fake send
			auto res_obj = std::make_tuple(1,1,msgpack::object(),43);
			std::stringstream buf;
			msgpack::pack(buf, res_obj);
		
			fake_send(0, FTL_PROTOCOL_RPCRETURN, buf.str());
			s->data();
		};
		
		int res = s->call<int>("test1", 78);
		
		REQUIRE( res == 43 );
	}
	
	waithandler = nullptr;
}

TEST_CASE("Socket::call+bind loop", "[rpc]") {
	auto s = new ftl::net::Socket(0);

	// Just loop the send back to the recv
	waithandler = [&]() {
		s->data();
	};
	
	SECTION( "Loop a numeric result" ) {
		s->bind("test1", [](int a) -> int {
			std::cout << "Bind test1 called" << std::endl;
			return a*2;
		});
		
		int res = s->call<int>("test1", 5);
		REQUIRE( res == 10 );
	}
	
	SECTION( "Loop a string result" ) {
		s->bind("test1", [](std::string a) -> std::string {
			std::cout << "Test1 = " << a << std::endl;
			return a + " world";
		});
		
		auto res = s->call<std::string>("test1", "hello");
		
		REQUIRE( res == "hello world" );
	}
}

