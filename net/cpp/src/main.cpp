#include <string>
#include <iostream>
#include <map>
//#include <vector>
#include <fstream>
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
using json = nlohmann::json;
using std::ifstream;
using std::map;

static Universe *universe;
static volatile bool stop = false;

// Store loaded configuration
static json config;

/**
 * Find and load a JSON configuration file
 */
static bool findConfiguration(const string &file) {
	ifstream i;
	
	if (file != "") i.open(file);
	if (!i.is_open()) i.open("./config.json");
	if (!i.is_open()) i.open(FTL_LOCAL_CONFIG_ROOT "/config.json");
	if (!i.is_open()) i.open(FTL_GLOBAL_CONFIG_ROOT "/config.json");
	if (!i.is_open()) return false;
	i >> config;
	return true;
}

/**
 * Generate a map from command line option to value
 */
map<string, string> read_options(char ***argv, int *argc) {
	map<string, string> opts;

	while (*argc > 0) {
		string cmd((*argv)[0]);
		if (cmd[0] != '-') break;

		size_t p;
		if ((p = cmd.find("=")) == string::npos) {
			opts[cmd.substr(2)] = "true";
		} else {
			opts[cmd.substr(2, p-2)] = cmd.substr(p+1);
		}

		(*argc)--;
		(*argv)++;
	}

	return opts;
}

/**
 * Put command line options into json config. If config element does not exist
 * or is of a different type then report an error.
 */
static void process_options(const map<string, string> &opts) {
	for (auto opt : opts) {
		if (opt.first == "config") continue;

		if (opt.first == "version") {
			std::cout << "FTL Vision Node - v" << FTL_VERSION << std::endl;
			std::cout << FTL_VERSION_LONG << std::endl;
			exit(0);
		}

		try {
			auto ptr = json::json_pointer("/"+opt.first);
			// TODO(nick) Allow strings without quotes
			auto v = json::parse(opt.second);
			if (v.type() != config.at(ptr).type()) {
				LOG(ERROR) << "Incorrect type for argument " << opt.first;
				continue;
			}
			config.at(ptr) = v;
		} catch(...) {
			LOG(ERROR) << "Unrecognised option: " << opt.first;
		}
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

int main(int argc, char **argv) {
	argc--;
	argv++;

	// Process Arguments
	auto options = read_options(&argv, &argc);
	if (!findConfiguration(options["config"])) {
		LOG(FATAL) << "Could not find any configuration!";
	}
	process_options(options);
	
	universe = new Universe(config);
	
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

