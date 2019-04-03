#ifndef _FTL_NET_INTERNAL_HPP_
#define _FTL_NET_INTERNAL_HPP_

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
	inline ssize_t recv(int sd, void *buf, size_t n, int f) { return ::recv(sd,buf,n,f); }
	inline ssize_t writev(int sd, const struct iovec *v, int cnt) { return ::writev(sd,v,cnt); }
#endif
#endif
}}}

#endif  // _FTL_NET_INTERNAL_HPP_

