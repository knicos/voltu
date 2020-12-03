/**
 * @file ws_internal.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_NET_WS_INTERNAL_HPP_
#define _FTL_NET_WS_INTERNAL_HPP_

#include <stdint.h>
#include <cstddef>
#include <functional>
#include <ftl/uri.hpp>

#include <ftl/utility/msgpack.hpp>
#include <ftl/net/common.hpp>

using std::size_t;

namespace ftl {
namespace net {

/* Taken from easywsclient */
struct wsheader_type {
	unsigned header_size;
	bool fin;
	bool mask;
	enum opcode_type {
		CONTINUATION = 0x0,
		TEXT_FRAME = 0x1,
		BINARY_FRAME = 0x2,
		CLOSE = 8,
		PING = 9,
		PONG = 0xa,
	} opcode;
	int N0;
	uint64_t N;
	uint8_t masking_key[4];
};

struct ws_options {
	std::string userinfo = "";
};

/**
 * Websocket dispatch parser. Given a raw socket buffer and its length, this
 * function parses the websocket header and if valid and containing enough data
 * in the buffer, it calls the passed function. If not enough data available
 * to complete the dispatch, -1 is returned. Otherwise the total amount read
 * from the buffer is returned.
 */
int ws_dispatch(const char *data, size_t len, std::function<void(const wsheader_type&,const char*,size_t)> d);

int ws_parse(msgpack::unpacker &buf, wsheader_type &ws);

/**
 * Websocket header constructor. Fills a buffer with the correct websocket
 * header for a given opcode, mask setting and message length.
 */
int ws_prepare(wsheader_type::opcode_type, bool useMask, size_t len, char *buffer, size_t maxlen);

bool ws_connect(SOCKET sockfd, const ftl::URI &uri, const ws_options &options=ws_options());

};
};

#endif  // _FTL_NET_WS_INTERNAL_HPP_

