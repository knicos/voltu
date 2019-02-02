//#define _GNU_SOURCE
#include <ftl/net/ice.hpp>
#include <ftl/net/stun.hpp>
#include <ftl/uri.hpp>

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <linux/if_link.h>

#define MAXLINE 200

using ftl::URI;

struct Candidate {
	Candidate() : port(0) { addr[0] = 0; }
	char addr[15];
	uint16_t port;
};

int stun_internal(std::string &c, bool tcp, uint16_t lport, std::string &host, uint16_t port) {
	int sockfd;
	sockaddr_in servaddr;
	sockaddr_in localaddr;
	unsigned char bindingReq[20];
	unsigned char buf[MAXLINE] = {0};

	sockfd = socket(AF_INET, (tcp) ? SOCK_STREAM : SOCK_DGRAM, 0);

	hostent *hoste = gethostbyname(host.c_str());

	if (hoste == NULL) {
		std::cerr << "Host not found: " << host << std::endl;
		close(sockfd);
		return -4;
	}

	// STUN Server
	bzero(&servaddr, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = ((in_addr *)(hoste->h_addr))->s_addr;
	servaddr.sin_port = htons(port);

	// local
	bzero(&localaddr, sizeof(localaddr));
	localaddr.sin_family = AF_INET;
	localaddr.sin_port = htons(lport);

	int n = bind(sockfd,(sockaddr *)&localaddr,sizeof(localaddr));

	// Build message packet
	* (short *)(&bindingReq[0]) = htons(0x0001);    // stun_method
	* (short *)(&bindingReq[2]) = htons(0x0000);    // msg_length
	* (int *)(&bindingReq[4])   = htonl(0x2112A442);// magic cookie

	*(int *)(&bindingReq[8]) = htonl(0x63c7117e);   // transacation ID 
	*(int *)(&bindingReq[12])= htonl(0x0714278f);
	*(int *)(&bindingReq[16])= htonl(0x5ded3221);

	if (!tcp) {
		n = sendto(sockfd, bindingReq, sizeof(bindingReq),0,(sockaddr *)&servaddr, sizeof(servaddr)); 
	} else {
		// TODO TCP STUN
	}

	if (n < 0) {
		// Error
		std::cerr << "STUN send error : " << host << ":" << port << std::endl;
		close(sockfd);
		return -2;
	}

	// Sleep!!? Select?

	if (!tcp) {
		n = recvfrom(sockfd, buf, MAXLINE, 0, NULL,0); // recv UDP
	} else {
		// TODO TCP
	}

	if (n < 0) {
		// Error
		std::cerr << "STUN recv error : " << host << ":" << port << std::endl;
		close(sockfd);
		return -2;
	}

	if (*(short *)(&buf[0]) == htons(0x0101)) {
		std::cerr << "STUN Success " << n << std::endl;

		n = htons(*(short *)(&buf[2]));
		size_t i = 20; // Header is 20 bytes, so skip
		Candidate cc;

       	while(i<sizeof(buf)) {
			short attr_type = htons(*(short *)(&buf[i]));
			short attr_length = htons(*(short *)(&buf[i+2]));

			std::cerr << "  -- Attr type: " << std::hex << attr_type << std::dec << std::endl;

			if (attr_type == 0x0001) {
				// parse : port, IP 
				cc.port = ntohs(*(short *)(&buf[i+6]));
				sprintf(cc.addr,"%d.%d.%d.%d",buf[i+8],buf[i+9],buf[i+10],buf[i+11]);
				break;
			} else if (attr_type == 0x0020) {
				// parse : port, IP 
				cc.port = ntohs(*(short *)(&buf[i+6]));
				cc.port ^= 0x2112;
				sprintf(cc.addr,"%d.%d.%d.%d",buf[i+8]^0x21,buf[i+9]^0x12,buf[i+10]^0xA4,buf[i+11]^0x42);
				break;
			}

			i += (4  + attr_length);
       }

		c = ((tcp) ? "tcp://" : "udp://");
		c += cc.addr;
		c += ":";
		c += std::to_string(cc.port);

		close(sockfd);
		return 0;
	}

	close(sockfd);
	return -3;
}

int ftl::net::ice::stun(std::string &c, const char *stunuri, uint16_t lport) {
	URI uri(stunuri);

	if (uri.getProtocol() == URI::SCHEME_UDP) return stun_internal(c, false, lport, uri.getHost(), uri.getPort());
	else if (uri.getProtocol() == URI::SCHEME_TCP) return stun_internal(c, true, lport, uri.getHost(), uri.getPort());
	else return -1;
}

int ftl::net::ice::stun(std::string &c, uint16_t lport, bool tcp) {
	// Choose a STUN server
	std::string uristr = (tcp) ? "tcp://" : "udp://";
	uristr += "stun.l.google.com:19302";

	return ftl::net::ice::stun(c, uristr.c_str(), lport);
}

int local_interfaces(std::vector<std::string> &c, bool tcp, uint16_t lport) {
	ifaddrs *ifaddr, *ifa;
	int s, n;
	char host[NI_MAXHOST];

	if (getifaddrs(&ifaddr) == -1) {
		return -1;
	}

	for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++) {
		if (ifa->ifa_addr == NULL) continue;

		auto family = ifa->ifa_addr->sa_family;
		if (family == AF_INET) {  // || AF_INET6
			s = getnameinfo(ifa->ifa_addr,
					(family == AF_INET) ? sizeof(struct sockaddr_in) :
										 sizeof(struct sockaddr_in6),
					host, NI_MAXHOST,
					NULL, 0, NI_NUMERICHOST);

			if (s != 0) return -1;

			c.push_back(std::string((tcp) ? "tcp://" : "udp://") + host + ":" + std::to_string(lport));
		}
	}

	freeifaddrs(ifaddr);
	return 0;
}

int ftl::net::ice::candidates(std::vector<std::string> &c, uint16_t lport, bool tcp) {
	local_interfaces(c, tcp, lport);

	std::string stunstr;
	ftl::net::ice::stun(stunstr, lport, tcp);
	c.push_back(stunstr);

	// TURN or proxy

	return 0;
}


