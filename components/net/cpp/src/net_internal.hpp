#ifndef _FTL_NET_INTERNAL_HPP_
#define _FTL_NET_INTERNAL_HPP_

#if defined _DEBUG && DEBUG_NET
#include <loguru.hpp>
#include <chrono>
#endif

namespace ftl { namespace net { namespace internal {
#ifdef TEST_MOCKS
#ifdef WIN32
	int recv(SOCKET sd, char *buf, int n, int f);
	int send(SOCKET sd, const char *v, int cnt, int flags);
#else
	ssize_t recv(int sd, void *buf, size_t n, int f);
	ssize_t writev(int sd, const struct iovec *v, int cnt);
#endif
#else
#ifdef WIN32
	inline int recv(SOCKET sd, char *buf, int n, int f) { return ::recv(sd,buf,n,f); }
	inline int send(SOCKET sd, const char *v, int cnt, int flags) { return ::send(sd,v,cnt,flags); }
#else
#if defined _DEBUG && DEBUG_NET
	inline ssize_t recv(int sd, void *buf, size_t n, int f) {
		return ::recv(sd,buf,n,f);
	}
	inline ssize_t writev(int sd, const struct iovec *v, int cnt) {
		auto start = std::chrono::high_resolution_clock::now();
		return ::writev(sd,v,cnt);
		std::chrono::duration<double> elapsed =
				std::chrono::high_resolution_clock::now() - start;
		LOG(INFO) << "WRITEV in " << elapsed.count() << "s";
	}
#else
	inline ssize_t recv(int sd, void *buf, size_t n, int f) { return ::recv(sd,buf,n,f); }
	inline ssize_t writev(int sd, const struct iovec *v, int cnt) { return ::writev(sd,v,cnt); }
#endif  // DEBUG
#endif  // WIN32
#endif  // TEST_MOCKS
}}}

#endif  // _FTL_NET_INTERNAL_HPP_

