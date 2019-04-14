#include <glog/logging.h>
#include <ftl/config.h>

#ifdef WIN32
#include <windows.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

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
using std::optional;
using ftl::is_file;
using ftl::is_directory;

// Store loaded configuration
namespace ftl {
json config;
};

using ftl::config;

bool ftl::is_directory(const std::string &path) {
#ifdef WIN32
	DWORD attrib = GetFileAttributesA(path.c_str());
	if (attrib == INVALID_FILE_ATTRIBUTES) return false;
	else return (attrib & FILE_ATTRIBUTE_DIRECTORY);
#else
	struct stat s;
	if (::stat(path.c_str(), &s) == 0) {
		return S_ISDIR(s.st_mode);
	} else {
		return false;
	}
#endif
}

bool ftl::is_file(const std::string &path) {
#ifdef WIN32
	DWORD attrib = GetFileAttributesA(path.c_str());
	if (attrib == INVALID_FILE_ATTRIBUTES) return false;
	else return !(attrib & FILE_ATTRIBUTE_DIRECTORY);
#else
	struct stat s;
	if (::stat(path.c_str(), &s) == 0) {
		return S_ISREG(s.st_mode);
	} else {
		return false;
	}
#endif
}

bool ftl::create_directory(const std::string &path) {
#ifdef WIN32
	// TODO(nick)
#else
	if (!is_directory(path)) {
		int err = ::mkdir(path.c_str(), S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
		return err != -1;
	}
	return true;
#endif
}

optional<string> locateFile(const string &name, const vector<string> &paths) {	
	for (auto p : paths) {
		if (is_directory(p)) {
			if (is_file(p+"/"+name)) {
				return p+"/"+name;
			}
		} else if (p.size() >= name.size() && 
				p.compare(p.size() - name.size(), name.size(), name) == 0 &&
				is_file(p)) {
			return p;
		}
	}
	
	if (is_file("./"+name)) return "./"+name;
	if (is_file(string(FTL_LOCAL_CONFIG_ROOT) +"/"+ name)) return string(FTL_LOCAL_CONFIG_ROOT) +"/"+ name;
	if (is_file(string(FTL_GLOBAL_CONFIG_ROOT) +"/"+ name)) return string(FTL_GLOBAL_CONFIG_ROOT) +"/"+ name;
	return {};
}

/**
 * Find and load a JSON configuration file
 */
static bool findConfiguration(const string &file, const vector<string> &paths,
		const std::string &app) {
	ifstream i;
	
	if (file != "") i.open(file);

	if (!i.is_open()) {
		auto f = locateFile("config.json", paths);
		if (!f) return false;
		i.open(*f);
	}
	
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
			auto val = cmd.substr(p+1);
			if (std::isdigit(val[0]) || val == "true" || val == "false" || val == "null") {
				opts[cmd.substr(2, p-2)] = val;
			} else {
				if (val[0] == '\\') opts[cmd.substr(2, p-2)] = val;
				else opts[cmd.substr(2, p-2)] = "\""+val+"\"";
			}
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

