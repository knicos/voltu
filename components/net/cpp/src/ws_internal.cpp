/*
 * Copyright 2019 Nicolas Pope.
 */

//#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <loguru.hpp>

#include <cstring>
#include <ftl/net/ws_internal.hpp>
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
#include <winsock2.h>
#include <Ws2tcpip.h>
#include <windows.h>
#endif

#include <string>
#include <iostream>

using std::size_t;
using ftl::net::wsheader_type;
using ftl::URI;
using std::string;

/* Taken from easywsclient. */
int ftl::net::ws_dispatch(const char *data, size_t len, std::function<void(const wsheader_type&,const char*,size_t)> d) {
	wsheader_type ws;
	if (len < 2) return -1;
	
	ws.fin = (data[0] & 0x80) == 0x80;
	ws.opcode = (wsheader_type::opcode_type) (data[0] & 0x0f);
	ws.mask = (data[1] & 0x80) == 0x80;
	ws.N0 = (data[1] & 0x7f);
	ws.header_size = 2 + (ws.N0 == 126? 2 : 0) + (ws.N0 == 127? 8 : 0) + (ws.mask? 4 : 0);

	if (len < ws.header_size) return -1;

	int i = 0;
	if (ws.N0 < 126) {
		ws.N = ws.N0;
		i = 2;
	} else if (ws.N0 == 126) {
		ws.N = 0;
		ws.N |= ((uint64_t) data[2]) << 8;
		ws.N |= ((uint64_t) data[3]) << 0;
		i = 4;
	} else if (ws.N0 == 127) {
		ws.N = 0;
		ws.N |= ((uint64_t) data[2]) << 56;
		ws.N |= ((uint64_t) data[3]) << 48;
		ws.N |= ((uint64_t) data[4]) << 40;
		ws.N |= ((uint64_t) data[5]) << 32;
		ws.N |= ((uint64_t) data[6]) << 24;
		ws.N |= ((uint64_t) data[7]) << 16;
		ws.N |= ((uint64_t) data[8]) << 8;
		ws.N |= ((uint64_t) data[9]) << 0;
		i = 10;
	}
	
	if (ws.mask) {
		ws.masking_key[0] = ((uint8_t) data[i+0]) << 0;
		ws.masking_key[1] = ((uint8_t) data[i+1]) << 0;
		ws.masking_key[2] = ((uint8_t) data[i+2]) << 0;
		ws.masking_key[3] = ((uint8_t) data[i+3]) << 0;
	} else {
		ws.masking_key[0] = 0;
		ws.masking_key[1] = 0;
		ws.masking_key[2] = 0;
		ws.masking_key[3] = 0;
	}
	
	if (len < ws.header_size+ws.N) return -1;

	// Perform dispatch
	d(ws, &data[ws.header_size], ws.N);
	return ws.header_size+ws.N;
}

int ftl::net::ws_prepare(wsheader_type::opcode_type op, bool useMask, size_t len, char *data, size_t maxlen) {
	// TODO:
	// Masking key should (must) be derived from a high quality random
	// number generator, to mitigate attacks on non-WebSocket friendly
	// middleware:
	const uint8_t masking_key[4] = { 0x12, 0x34, 0x56, 0x78 };

	char *header = data;
	size_t header_size = 2 + (len >= 126 ? 2 : 0) + (len >= 65536 ? 6 : 0) + (useMask ? 4 : 0);
	if (header_size > maxlen) return -1;

	memset(header, 0, header_size);
	header[0] = 0x80 | op;
	if (false) { }
	else if (len < 126) {
		header[1] = (len & 0xff) | (useMask ? 0x80 : 0);
		if (useMask) {
		    header[2] = masking_key[0];
		    header[3] = masking_key[1];
		    header[4] = masking_key[2];
		    header[5] = masking_key[3];
		}
	} else if (len < 65536) {
		header[1] = 126 | (useMask ? 0x80 : 0);
		header[2] = (len >> 8) & 0xff;
		header[3] = (len >> 0) & 0xff;
		if (useMask) {
		    header[4] = masking_key[0];
		    header[5] = masking_key[1];
		    header[6] = masking_key[2];
		    header[7] = masking_key[3];
		}
	} else {
		header[1] = 127 | (useMask ? 0x80 : 0);
		header[2] = (len >> 56) & 0xff;
		header[3] = (len >> 48) & 0xff;
		header[4] = (len >> 40) & 0xff;
		header[5] = (len >> 32) & 0xff;
		header[6] = (len >> 24) & 0xff;
		header[7] = (len >> 16) & 0xff;
		header[8] = (len >>  8) & 0xff;
		header[9] = (len >>  0) & 0xff;
		if (useMask) {
		    header[10] = masking_key[0];
		    header[11] = masking_key[1];
		    header[12] = masking_key[2];
		    header[13] = masking_key[3];
		}
	}
	
	return header_size;
}

bool ftl::net::ws_connect(int sockfd, const URI &uri) {
	string http = "";
	int status;
	int i;
	char line[256];
	
	http += "GET "+uri.getPath()+" HTTP/1.1\r\n";
	if (uri.getPort() == 80) {
		http += "Host: "+uri.getHost()+"\r\n";
	} else {
		http += "Host: "+uri.getHost()+":"+std::to_string(uri.getPort())+"\r\n";
	}
	http += "Upgrade: websocket\r\n";
	http += "Connection: Upgrade\r\n";
	http += "Sec-WebSocket-Key: x3JJHMbDL1EzLkh9GBhXDw==\r\n";
	http += "Sec-WebSocket-Version: 13\r\n";
	http += "\r\n";
	int rc = ::send(sockfd, http.c_str(), http.length(), 0);
	if (rc != (int)http.length()) {
		LOG(ERROR) << "Could not send Websocket http request...";
		std::cout << http;
		return false;
	}
	
	for (i = 0; i < 2 || (i < 255 && line[i-2] != '\r' && line[i-1] != '\n'); ++i) { if (recv(sockfd, line+i, 1, 0) == 0) { return false; } }
	line[i] = 0;
	if (i == 255) { fprintf(stderr, "ERROR: Got invalid status line connecting to: %s\n", uri.getHost().c_str()); return false; }
	if (sscanf(line, "HTTP/1.1 %d", &status) != 1 || status != 101) { fprintf(stderr, "ERROR: Got bad status connecting to %s: %s", uri.getHost().c_str(), line); return false; }
	// TODO: verify response headers,
	while (true) {
		for (i = 0; i < 2 || (i < 255 && line[i-2] != '\r' && line[i-1] != '\n'); ++i) { if (recv(sockfd, line+i, 1, 0) == 0) { return false; } }
		if (line[0] == '\r' && line[1] == '\n') { break; }
	}
	return true;
}

