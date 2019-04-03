#include "catch.hpp"
#include <ftl/net.hpp>
#include <ftl/net/socket.hpp>
#include <ftl/net/listener.hpp>

#include <memory>
#include <iostream>

using ftl::net::Socket;
using ftl::net::Protocol;
using std::shared_ptr;

// --- Support -----------------------------------------------------------------

#ifndef WIN32
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#endif

#ifdef WIN32
#include <winsock2.h>
#include <Ws2tcpip.h>
#include <windows.h>

#pragma comment(lib, "Ws2_32.lib")
#endif

static int ssock = INVALID_SOCKET;
static sockaddr_in slocalAddr;

void fin_server() {
	//int t = 1;
	//setsockopt(ssock,SOL_SOCKET,SO_REUSEADDR,&t,sizeof(int));

	#ifndef WIN32
	if (ssock != INVALID_SOCKET) close(ssock);
	#else
	if (ssock != INVALID_SOCKET) closesocket(ssock);
	#endif

	ssock = INVALID_SOCKET;
}

void init_server() {
	//fin_server();
	int port = 7077;
	
	#ifdef WIN32
	WSAData wsaData;
	//If Win32 then load winsock
	if (WSAStartup(MAKEWORD(1,1), &wsaData) != 0) {
		std::cerr << "Socket error\n";
		return;
	}
	#endif

	ssock = socket(AF_INET, SOCK_STREAM, 0);
	if (ssock == INVALID_SOCKET) {
		std::cerr << "Socket error 1\n";
		return;
	}

	int enable = 1;
	if (setsockopt(ssock, SOL_SOCKET, SO_REUSEADDR, (char*)&enable, sizeof(int)) < 0)
    	std::cerr << "setsockopt(SO_REUSEADDR) failed" << std::endl;

	//Specify listen port and address
	//memset(&s_localAddr, 0, sizeof(s_localAddr));
	slocalAddr.sin_family = AF_INET;
	slocalAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	slocalAddr.sin_port = htons(port);
	
	int rc = bind(ssock, (struct sockaddr*)&slocalAddr, sizeof(slocalAddr));
	
	if (rc == SOCKET_ERROR) {
		std::cerr << "Socket error 2\n";
		
		#ifndef WIN32
		close(ssock);
		#else
		closesocket(ssock);
		#endif
		ssock = INVALID_SOCKET;
		return;
	}

	//Attempt to start listening for connection requests.
	rc = ::listen(ssock, 1);

	if (rc == SOCKET_ERROR) {
		std::cerr << "Socket error 3\n";

		#ifndef WIN32
		close(ssock);
		#else
		closesocket(ssock);
		#endif
		ssock = INVALID_SOCKET;
		return;
	}
}

// --- Tests -------------------------------------------------------------------

TEST_CASE("net::connect()", "[net]") {
	init_server();
	shared_ptr<Socket> sock = nullptr;

	SECTION("valid tcp connection using ipv4") {
		sock = ftl::net::connect("tcp://127.0.0.1:7077");
		REQUIRE(sock != nullptr);
		REQUIRE(sock->isValid());
	}

	SECTION("valid tcp connection using hostname") {
		sock = ftl::net::connect("tcp://localhost:7077");
		REQUIRE(sock->isValid());
	}

	SECTION("invalid protocol") {
		sock = ftl::net::connect("http://127.0.0.1:7077");
		REQUIRE(!sock->isValid());
	}

	SECTION("empty uri") {
		sock = ftl::net::connect("");
		REQUIRE(!sock->isValid());
	}

	SECTION("null uri") {
		sock = ftl::net::connect(NULL);
		REQUIRE(!sock->isValid());
	}

	// Disabled due to long timeout
	/*SECTION("incorrect ipv4 address") {
		sock = ftl::net::raw::connect("tcp://192.0.1.1:7077");
		REQUIRE(sock != NULL);
		REQUIRE(sock->isConnected() == false);
		sock = NULL;
	}*/

	// Removed as too slow
	/*SECTION("incorrect dns address") {
		sock = ftl::net::connect("tcp://xryyrrgrtgddgr.com:7077");
		REQUIRE(!sock->isValid());
	}*/
	
	fin_server();
}

TEST_CASE("net::listen()", "[net]") {

	SECTION("tcp any interface") {
		REQUIRE( ftl::net::listen("tcp://localhost:9001")->isListening() );

		SECTION("can connect to listening socket") {
			auto sock = ftl::net::connect("tcp://127.0.0.1:9001");
			REQUIRE(sock->isValid());
			ftl::net::wait([&sock]() { return sock->isConnected(); });
			REQUIRE(sock->isConnected());

			// TODO Need way of knowing about connection
		}

		ftl::net::stop();
	}
	
	SECTION("on connection event") {
		auto l = ftl::net::listen("tcp://localhost:9002");
		REQUIRE( l->isListening() );
		
		bool connected = false;
		
		l->onConnection([&](shared_ptr<Socket> s) {
			ftl::net::wait([&s]() { return s->isConnected(); });
			REQUIRE( s->isConnected() );
			connected = true;
		});
		
		auto sock = ftl::net::connect("tcp://127.0.0.1:9002");
		ftl::net::wait();
		REQUIRE( connected );
		ftl::net::stop();
	}
}

TEST_CASE("Net Integration", "[integrate]") {
	std::string data;
	
	Protocol p("ftl://utu.fi");
	
	p.bind("add", [](int a, int b) {
		return a + b;
	});
	
	p.bind(100, [&data](uint32_t m, Socket &s) {
		s.read(data);
	});
	
	auto l = ftl::net::listen("tcp://localhost:9000");
	REQUIRE( l->isListening() );
	l->setProtocol(&p);
	
	shared_ptr<Socket> s1;
	l->onConnection([&s1](auto &s) { s1 = s; });
	
	shared_ptr<Socket> s2 = ftl::net::connect("tcp://localhost:9000");
	s2->setProtocol(&p);
	
	REQUIRE( s2 != nullptr );
	ftl::net::wait([&s2]() { return s2->isConnected(); });
	REQUIRE( s1 != nullptr );	

	REQUIRE( s1->isConnected() );
	REQUIRE( s2->isConnected() );
	
	REQUIRE( s1->call<int>("add", 5, 6) == 11 );
	REQUIRE( s2->call<int>("add", 10, 5) == 15);
	
	s1->send(100, "hello world");
	ftl::net::wait();
	// TODO s2->wait(100);
	
	REQUIRE( data == "hello world" );
}

