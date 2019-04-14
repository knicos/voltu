#include <glog/logging.h>
#include <ftl/config.h>

#include <nlohmann/json.hpp>
#include <ftl/configuration.hpp>

#include <fstream>
#include <string>
#include <map>
#include <iostream>

using nlohmann::json;
using std::ifstream;
using std::string;
using std::map;
using std::vector;

// Store loaded configuration
namespace ftl {
json config;
};

using ftl::config;

/**
 * Find and load a JSON configuration file
 */
static bool findConfiguration(const string &file, const vector<string> &paths,
		const std::string &app) {
	ifstream i;
	
	if (file != "") i.open(file);
	if (!i.is_open()) i.open("./config.json");
	if (!i.is_open()) i.open(FTL_LOCAL_CONFIG_ROOT "/config.json");
	if (!i.is_open()) i.open(FTL_GLOBAL_CONFIG_ROOT "/config.json");
	if (!i.is_open()) return false;
	i >> config;
	config = config[app];
	return true;
}

/**
 * Generate a map from command line option to value
 */
static map<string, string> read_options(char ***argv, int *argc) {
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

vector<string> ftl::configure(int argc, char **argv, const std::string &app) {
	argc--;
	argv++;

	// Process Arguments
	auto options = read_options(&argv, &argc);
	
	vector<string> paths;
	while (argc-- > 0) {
		paths.push_back(argv[0]);
	}
	
	if (!findConfiguration(options["config"], paths, app)) {
		LOG(FATAL) << "Could not find any configuration!";
	}
	process_options(options);

	return paths;
}

