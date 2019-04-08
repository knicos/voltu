#include <string>
#include <iostream>
#include <ftl/net.hpp>

#ifndef WIN32
#include <readline/readline.h>
#endif

#ifdef WIN32
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Rpcrt4.lib")
#endif

using std::string;
using ftl::net::Universe;

static Universe *universe;
static volatile bool stop = false;

void handle_options(const char ***argv, int *argc) {
	while (*argc > 0) {
		string cmd((*argv)[0]);
		if (cmd[0] != '-') break;
		
		if (cmd.find("--peer=") == 0) {
			cmd = cmd.substr(cmd.find("=")+1);
			//std::cout << "Peer added " << cmd.substr(cmd.find("=")+1) << std::endl;
			universe->connect(cmd);
		} else if (cmd.find("--listen=") == 0) {
			cmd = cmd.substr(cmd.find("=")+1);
			universe->listen(cmd);
		}
		
		(*argc)--;
		(*argv)++;
	}
}

void handle_command(const char *l) {
	string cmd = string(l);
	
	if (cmd == "exit") {
		stop = true;
	} else if (cmd.find("peer ") == 0) {
		cmd = cmd.substr(cmd.find(" ")+1);
		universe->connect(cmd);
	} else if (cmd.find("list ") == 0) {
		cmd = cmd.substr(cmd.find(" ")+1);
		if (cmd == "peers") {
			//auto res = p2p->getPeers();
			//for (auto r : res) std::cout << "  " << r->to_string() << std::endl;
		}
	}
}

int main(int argc, const char **argv) {
	argc--;
	argv++;
	
	universe = new Universe("ftl://cli");
	
	// Process Arguments
	handle_options(&argv, &argc);
	
	while (!stop) {
#ifndef WIN32
		char *line = readline("> ");
#else
		char line[300];
		fgets(line, 299, stdin);
#endif
		if (!line) break;
		
		handle_command(line);
		
#ifndef WIN32
		free(line);
#endif
	}
	stop = true;
	
	delete universe;
	return 0;
}

