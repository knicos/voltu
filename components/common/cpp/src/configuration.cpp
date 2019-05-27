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

static bool endsWith(const string &s, const string &e) {
	return s.size() >= e.size() && 
				s.compare(s.size() - e.size(), e.size(), e) == 0;
}

bool ftl::is_video(const string &file) {
	return endsWith(file, ".mp4");
}

bool ftl::create_directory(const std::string &path) {
#ifdef WIN32
	// TODO(nick)
	return false;
#else
	if (!is_directory(path)) {
		int err = ::mkdir(path.c_str(), S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
		return err != -1;
	}
	return true;
#endif
}

optional<string> ftl::locateFile(const string &name) {
	auto paths = config["paths"];
	
	if (!paths.is_null()) {
		for (string p : paths) {
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
	}
	
	if (is_file("./"+name)) return "./"+name;
	if (is_file(string(FTL_LOCAL_CONFIG_ROOT) +"/"+ name)) return string(FTL_LOCAL_CONFIG_ROOT) +"/"+ name;
	if (is_file(string(FTL_GLOBAL_CONFIG_ROOT) +"/"+ name)) return string(FTL_GLOBAL_CONFIG_ROOT) +"/"+ name;
	return {};
}

/**
 * Combine one json config with another patch json config.
 */
static bool mergeConfig(const string &path) {
	ifstream i;
	i.open(path);
	if (i.is_open()) {
		try {
			json t;
			i >> t;
			config.merge_patch(t);
			return true;
		} catch (json::parse_error& e) {
			LOG(ERROR) << "Parse error in loading config: "  << e.what();
			return false;
		}
	} else {
		return false;
	}
}

/**
 * Find and load a JSON configuration file
 */
static bool findConfiguration(const string &file, const vector<string> &paths,
		const std::string &app) {
	bool f = false;
	bool found = false;
	
	f = mergeConfig(FTL_GLOBAL_CONFIG_ROOT "/config.json");
	found |= f;
	if (f) LOG(INFO) << "Loaded config: " << FTL_GLOBAL_CONFIG_ROOT "/config.json";
	f = mergeConfig(FTL_LOCAL_CONFIG_ROOT "/config.json");
	found |= f;
	if (f) LOG(INFO) << "Loaded config: " << FTL_LOCAL_CONFIG_ROOT "/config.json";
	f = mergeConfig("./config.json");
	found |= f;
	if (f) LOG(INFO) << "Loaded config: " << "./config.json";
	
	for (auto p : paths) {
		if (is_directory(p)) {
			f = mergeConfig(p+"/config.json");
			found |= f;
			if (f) LOG(INFO) << "Loaded config: " << p << "/config.json";
		}
	}
	
	if (file != "") {
		f = mergeConfig(file);
		found |= f;

		if (!f) {
			LOG(ERROR) << "Specific config file (" << file << ") was not found";
		} else {
			LOG(INFO) << "Loaded config: " << file;
		}
	}

	if (found) {
		config = config[app];
		return true;
	} else {
		return false;
	}
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
#ifdef WIN32
			if ((val[0] >= 48 && val[0] <= 57) || val == "true" || val == "false" || val == "null") {
#else
			if (std::isdigit(val[0]) || val == "true" || val == "false" || val == "null") {
#endif
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
			std::cout << "Future-Tech Lab - v" << FTL_VERSION << std::endl;
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

