#include <string>
#include <iostream>
#include <ftl/net/p2p.hpp>
#include <ftl/net/listener.hpp>
#include <ftl/net/socket.hpp>
#include <memory>
#include <thread>

#ifndef WIN32
#include <readline/readline.h>
#endif

#ifdef WIN32
#pragma comment(lib, "Ws2_32.lib")
#endif

using std::string;
using std::shared_ptr;
using ftl::net::P2P;
using ftl::net::Listener;
using ftl::net::Socket;

static P2P *p2p;
static shared_ptr<Listener> listener = nullptr;
static volatile bool stop = false;

void handle_options(const char ***argv, int *argc) {
	while (*argc > 0) {
		string cmd((*argv)[0]);
		if (cmd[0] != '-') break;
		
		if (cmd.find("--peer=") == 0) {
			cmd = cmd.substr(cmd.find("=")+1);
			//std::cout << "Peer added " << cmd.substr(cmd.find("=")+1) << std::endl;
			p2p->addPeer(cmd);
		} else if (cmd.find("--listen=") == 0) {
			cmd = cmd.substr(cmd.find("=")+1);
			listener = ftl::net::listen(cmd.c_str());
			if (listener) listener->setProtocol(p2p);
			listener->onConnection([](shared_ptr<Socket> &s) { p2p->addPeer(s); });
		}
		
		(*argc)--;
		(*argv)++;
	}
}

void run() {
	while (!stop) ftl::net::wait();
}

void handle_command(const char *l) {
	string cmd = string(l);
	
	if (cmd == "exit") {
		stop = true;
	} else if (cmd.find("peer ") == 0) {
		cmd = cmd.substr(cmd.find(" ")+1);
		p2p->addPeer(cmd);
	} else if (cmd.find("list ") == 0) {
		cmd = cmd.substr(cmd.find(" ")+1);
		if (cmd == "peers") {
			auto res = p2p->getPeers();
			for (auto r : res) std::cout << "  " << r->to_string() << std::endl;
		}
	}
}

int main(int argc, const char **argv) {
	argc--;
	argv++;
	
	p2p = new P2P("ftl://cli");
	
	// Process Arguments
	handle_options(&argv, &argc);
	
	std::thread nthread(run);
	
	while (!stop) {
		char *line = readline("> ");
		if (!line) break;
		
		handle_command(line);
		
		free(line);		
	}
	stop = true;
	
	nthread.join();
	
	delete p2p;
	return 0;
}

