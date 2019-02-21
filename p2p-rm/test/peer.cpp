#include "ftl/p2p-rm.hpp"
#include <ftl/net.hpp>

#include <gflags/gflags.h>

DEFINE_string(listen, "tcp://*:9000", "Listen URI");
DEFINE_string(peer, "", "Peer to connect to");

using namespace ftl;

int main(int argc, char *argv) {
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	
	auto net = net::listen(FLAGS_listen);
	auto cluster = rm::cluster("ftl://utu.fi", net);
	
	int data = 20;
	auto ptr = cluster->map<int>("ftl://utu.fi/memory/test1", &data);
	
	if (FLAGS_peer) {
		cluster->addPeer(FLAGS_peer);
		std::cout << "Value = " << *ptr << std::endl; // 25.
		std::cout << "Raw = " << data << std::endl; // 25.
		
		*ptr = 30;
	} else {
		*ptr = 25;
		
		ptr.onChange(()=> {
			std::cout << "Value changed = " << *ptr << std::endl; // 30
		});
	}
	
	while (net());
	return 0;
}

