#include "catch.hpp"
#include <string.h>
#include <ftl/net.hpp>
#include <ftl/net/socket.hpp>
#include <ftl/net/listener.hpp>
#include <iostream>
#include <memory>

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
#include <windows.h>
#include <winsock.h>
typedef int socklen_t;
#define MSG_WAITALL 0
#endif


// ---- MOCK Server Code -------------------------------------------------------

static bool server = false;
static bool running = false;
static int csock = INVALID_SOCKET;
static int ssock = INVALID_SOCKET;
static fd_set sfdread;
static fd_set sfderror;
static sockaddr_in slocalAddr;

using ftl::net::Socket;
using std::shared_ptr;

void fin_server() {
	if (!server) return;

	//int t = 1;
	//setsockopt(ssock,SOL_SOCKET,SO_REUSEADDR,&t,sizeof(int));

	#ifndef WIN32
	if (csock != INVALID_SOCKET) close(csock);
	if (ssock != INVALID_SOCKET) close(ssock);
	#else
	if (csock != INVALID_SOCKET) closesocket(csock);
	if (ssock != INVALID_SOCKET) closesocket(ssock);
	#endif

	csock = INVALID_SOCKET;
	ssock = INVALID_SOCKET;
	server = false;
}

void init_server() {
	fin_server();
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
	if (setsockopt(ssock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
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
		csock = INVALID_SOCKET;
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
		csock = INVALID_SOCKET;
		ssock = INVALID_SOCKET;
		return;
	}
	
	running = true;
	server = true;
}

void send_json(int service, const char *json) {
	int flen = strlen(json) + 2 * sizeof(int);
	char *buf = new char[flen];
	int *idat = (int*)buf;
	idat[0] = strlen(json)+sizeof(int);
	idat[1] = service;
	strcpy((char*)&idat[2], json);
	std::cerr << "SENDING " << flen << std::endl;
	::send(csock, buf, flen, MSG_DONTWAIT);
	//delete buf;
}

int setDescriptors() {
	//Reset all file descriptors
	FD_ZERO(&sfdread);
	FD_ZERO(&sfderror);

	int n = 0;

	//Set file descriptor for the listening socket.
	if (ssock != INVALID_SOCKET) {
		FD_SET(ssock, &sfdread);
		FD_SET(ssock, &sfderror);
		n = ssock;
	}

	//Set the file descriptors for each client
	if (csock != INVALID_SOCKET) {
		FD_SET(csock, &sfdread);
		FD_SET(csock, &sfderror);
		n = csock;
	}

	return n;
}

void accept_connection() {
	int n = setDescriptors();

	//Wait for a network event or timeout in 3 seconds
	timeval block;
	block.tv_sec = 1;
	block.tv_usec = 0;
	int selres = select(n+1, &sfdread, 0, &sfderror, &block);

	if (selres > 0 && FD_ISSET(ssock, &sfdread)) {
		int rsize = sizeof(sockaddr_storage);
		sockaddr_storage addr;
		
		std::cout << "Accepted connection!" << std::endl;

		//Finally accept this client connection.
		csock = accept(ssock, (sockaddr*)&addr, (socklen_t*)&rsize);
	} else {

	}
}

// ---- END MOCK ---------------------------------------------------------------

TEST_CASE("net::connect()", "[net]") {
	init_server();
	REQUIRE(ssock != INVALID_SOCKET);

	shared_ptr<Socket> sock = nullptr;

	SECTION("valid tcp connection using ipv4") {
		sock = ftl::net::connect("tcp://127.0.0.1:7077");
		REQUIRE(sock != nullptr);
		accept_connection();
	}

	SECTION("valid tcp connection using hostname") {
		sock = ftl::net::connect("tcp://localhost:7077");
		REQUIRE(sock->isValid());
		accept_connection();
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

	SECTION("incorrect dns address") {
		sock = ftl::net::connect("tcp://xryyrrgrtgddgr.com:7077");
		REQUIRE(sock->isValid());
		REQUIRE(sock->isConnected() == false);
		sock = nullptr;
	}

	if (sock && sock->isValid()) {
		REQUIRE(sock->isConnected());
		REQUIRE(csock != INVALID_SOCKET);
		sock->close();
	}
	fin_server();
}

TEST_CASE("net::listen()", "[net]") {

	SECTION("tcp any interface") {
		REQUIRE( ftl::net::listen("tcp://*:7078")->isListening() );

		SECTION("can connect to listening socket") {
			shared_ptr<Socket> sock = ftl::net::connect("tcp://127.0.0.1:7078");
			REQUIRE(sock->isValid());
			REQUIRE(sock->isConnected());
			ftl::net::wait();

			// TODO Need way of knowing about connection
		}

		ftl::net::stop();
	}
}

TEST_CASE("Socket.onMessage()", "[net]") {
	// Need a fake server...
	init_server();
	shared_ptr<Socket> sock = ftl::net::connect("tcp://127.0.0.1:7077");
	REQUIRE(sock->isValid());
	REQUIRE(sock->isConnected());
	accept_connection();

	SECTION("small valid message") {
		send_json(1, "{message: \"Hello\"}");

		bool msg = false;

		sock->onMessage([&](int service, std::string &data) {
			REQUIRE(service == 1);
			REQUIRE(data == "{message: \"Hello\"}");
			msg = true;	
		});

		ftl::net::wait();
		REQUIRE(msg);
	}

	SECTION("empty message") {
		send_json(1, "");

		bool msg = false;

		sock->onMessage([&](int service, std::string &data) {
			REQUIRE(service == 1);
			REQUIRE(data == "");
			msg = true;	
		});

		ftl::net::wait();
		REQUIRE(msg);
	}

	SECTION("multiple valid messages") {
		send_json(1, "{message: \"Hello\"}");
		send_json(1, "{test: \"world\"}");

		int msg = 0;

		sock->onMessage([&](int service, std::string &data) {
			REQUIRE(service == 1);
			if (msg == 0) REQUIRE(data == "{message: \"Hello\"}");
			else REQUIRE(data == "{test: \"world\"}");
			msg++;	
		});

		ftl::net::wait();
		REQUIRE(msg == 2);
	}

	SECTION("disconnected does not get message") {
		send_json(1, "world");

		bool msg = false;

		sock->onMessage([&](int service, std::string &data) {
			msg = true;	
		});

		sock->close();

		ftl::net::wait();
		REQUIRE(!msg);
	}

	fin_server();
}

