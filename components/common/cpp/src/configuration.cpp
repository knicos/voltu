#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>
#include <ftl/config.h>

#ifdef WIN32
#include <windows.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#ifndef WIN32
#include <signal.h>
#endif

#include <nlohmann/json.hpp>
#include <ftl/configuration.hpp>
#include <ftl/configurable.hpp>
#include <ftl/uri.hpp>

#include <fstream>
#include <string>
#include <map>
#include <iostream>

using ftl::config::json_t;
using std::ifstream;
using std::string;
using std::map;
using std::vector;
using std::optional;
using ftl::is_file;
using ftl::is_directory;
using ftl::Configurable;

// Store loaded configuration
namespace ftl {
namespace config {
json_t config;
//json_t root_config;
};
};

using ftl::config::config;
//using ftl::config::root_config;

static Configurable *rootCFG = nullptr;

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

optional<string> ftl::config::locateFile(const string &name) {
	if (is_file(name)) return name;

	auto paths = rootCFG->getConfig()["paths"];
	
	if (paths.is_array()) {
		vector<string> vpaths = paths.get<vector<string>>();
		for (string p : vpaths) {
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
static bool mergeConfig(const string path) {
	ifstream i(path.c_str());
	//i.open(path);
	if (i.is_open()) {
		try {
			nlohmann::json t;
			i >> t;
			config.merge_patch(t);
			return true;
		} catch (nlohmann::json::parse_error& e) {
			LOG(ERROR) << "Parse error in loading config: "  << e.what();
			return false;
		} catch (...) {
			LOG(ERROR) << "Unknown error opening config file";
		}
	} else {
		return false;
	}
}

static std::map<std::string, json_t*> config_index;
static std::map<std::string, ftl::Configurable*> config_instance;

/*
 * Recursively URI index the JSON structure.
 */
static void _indexConfig(json_t &cfg) {
	if (cfg.is_object()) {
		auto id = cfg["$id"];
		if (id.is_string()) {
			LOG(INFO) << "Indexing: " << id.get<string>();
			config_index[id.get<string>()] = &cfg;
		}

		for (auto i : cfg.items()) {
			if (i.value().is_structured()) {
				_indexConfig(cfg[i.key()]);
			}
		}
	} // TODO(Nick) Arrays....
}

ftl::Configurable *ftl::config::find(const std::string &uri) {
	auto ix = config_instance.find(uri);
	if (ix == config_instance.end()) return nullptr;
	else return (*ix).second;
}

void ftl::config::registerConfigurable(ftl::Configurable *cfg) {
	auto uri = cfg->get<string>("$id");
	if (!uri) {
		LOG(FATAL) << "Configurable object is missing $id property";
		return;
	}
	auto ix = config_instance.find(*uri);
	if (ix == config_instance.end()) {
		LOG(FATAL) << "Attempting to create a duplicate object: " << *uri;
	} else {
		config_instance[*uri] = cfg;
	}
}

json_t null_json;

json_t &ftl::config::resolve(const std::string &puri) {
	string uri_str = puri;

	// TODO(Nick) Must support alternative root (last $id)
	if (uri_str.at(0) == '#') {
		string id_str = config["$id"].get<string>();
		if (id_str.find('#') != string::npos) {
			uri_str[0] = '/';
		} // else {
			uri_str = id_str + uri_str;
		//}
	}

	ftl::URI uri(uri_str);
	if (uri.isValid()) {
		std::string u = uri.getBaseURI();
		LOG(INFO) << "Resolve URI: " << u;
		auto ix = config_index.find(u);
		if (ix == config_index.end()) {
			LOG(FATAL) << "Cannot find resource: " << u;
		}

		auto ptr = nlohmann::json::json_pointer("/"+uri.getFragment());
		try {
			return resolve((*ix).second->at(ptr));
		} catch(...) {
			return null_json;
		}
	//}
/*

		int n = 0;
		while (u.size() != 0) {
			LOG(INFO) << "Resolve URI: " << u;
			auto ix = config_index.find(u);
			if (ix == config_index.end()) {
				u = uri.getBaseURI(--n);
				continue;
			}
			//LOG(INFO) << "FOUND URI " << *(*ix).second;

			if (n == 0) {
				return *(*ix).second;
			} else {
				// Must apply path segments again...
				nlohmann::json *c = (*ix).second;
				while (n < 0) {
					auto seg = uri.getPathSegment(n++);
					c = &(*c)[seg];
					
					// Recursive resolve...
					if ((*c).is_string()) {
						c = &resolve(*c);
					}
				}

				// Give it the correct URI
				if (!(*c)["$id"].is_string()) {
					(*c)["$id"] = uri.getBaseURI();
				}

				return *c;
			}
		}
		LOG(FATAL) << "Unresolvable configuration URI: " << uri.getBaseURI();
		return null_json;*/
	} else {
		return null_json;
	}
}

json_t &ftl::config::resolve(json_t &j) {
	if (j.is_object() && j["$ref"].is_string()) {
		return resolve(j["$ref"].get<string>());
	} else {
		return j;
	}
}

/**
 * Find and load a JSON configuration file
 */
static bool findConfiguration(const string &file, const vector<string> &paths) {
	bool f = false;
	bool found = false;
	
	if (file.length() > 0) {
		f = mergeConfig(file.substr(1,file.length()-2));
		found |= f;

		if (!f) {
			LOG(ERROR) << "Specific config file (" << file << ") was not found";
		} else {
			LOG(INFO) << "Loaded config: " << file;
		}
	} else {
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
	}

	if (found) {
		_indexConfig(config);
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
static void process_options(Configurable *root, const map<string, string> &opts) {
	for (auto opt : opts) {
		if (opt.first == "config") continue;
		if (opt.first == "root") continue;

		if (opt.first == "version") {
			std::cout << "Future-Tech Lab - v" << FTL_VERSION << std::endl;
			std::cout << FTL_VERSION_LONG << std::endl;
			exit(0);
		}

		try {
			//auto ptr = nlohmann::json::json_pointer("/"+opt.first);
			auto ptr = ftl::config::resolve(*root->get<string>("$id") + string("/") + opt.first);

			LOG(INFO) << "PARAM RES TO " << (*root->get<string>("$id") + string("/") + opt.first);

			auto v = nlohmann::json::parse(opt.second);
			std::string type = ptr.type_name();
			if (type != "null" && v.type_name() != type) {
				LOG(ERROR) << "Incorrect type for argument " << opt.first << " - expected '" << type << "', got '" << v.type_name() << "'";
				continue;
			}
			ptr.update(v);
		} catch(...) {
			LOG(ERROR) << "Unrecognised option: " << *root->get<string>("$id") << "#" << opt.first;
		}
	}
}

static void signalIntHandler( int signum ) {
   std::cout << "Closing...\n";

   // cleanup and close up stuff here  
   // terminate program  

   exit(0);
}

Configurable *ftl::config::configure(int argc, char **argv, const std::string &root) {
	loguru::init(argc, argv, "--verbosity");
	argc--;
	argv++;

	loguru::g_preamble_date = false;
	loguru::g_preamble_uptime = false;
	loguru::g_preamble_thread = false;

#ifndef WIN32
	signal(SIGINT,signalIntHandler);
#endif  // WIN32

	// Process Arguments
	auto options = read_options(&argv, &argc);
	
	vector<string> paths;
	while (argc-- > 0) {
		paths.push_back(argv[0]);
	}
	
	if (!findConfiguration(options["config"], paths)) {
		LOG(FATAL) << "Could not find any configuration!";
	}

	string root_str = (options.find("root") != options.end()) ? nlohmann::json::parse(options["root"]).get<string>() : root;

	Configurable *rootcfg = create<Configurable>(config);
	if (root_str.size() > 0) {
		LOG(INFO) << "Setting root to " << root_str;
		rootcfg = create<Configurable>(rootcfg, root_str);
	}

	//root_config = rootcfg->getConfig();
	rootCFG = rootcfg;
	rootcfg->set("paths", paths);
	process_options(rootcfg, options);
	return rootcfg;
}

