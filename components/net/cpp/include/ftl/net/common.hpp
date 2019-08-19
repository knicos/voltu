#ifndef _FTL_NET_COMMON_HPP_
#define _FTL_NET_COMMON_HPP_

#ifndef WIN32
#include <unistd.h>
#include <sys/poll.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <arpa/inet.h>
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define SOCKET int
#endif

#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
typedef int socklen_t;
#endif

#endif  // _FTL_NET_COMMON_HPP_
