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
#include <ftl/threads.hpp>
#include <ftl/timer.hpp>
#include <ftl/cuda_common.hpp>

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

ctpl::thread_pool ftl::pool(std::thread::hardware_concurrency()*2);

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

bool ftl::running = true;
int ftl::exit_code = 0;
std::string ftl::branch_name = FTL_BRANCH;

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
	// TODO:(nick)
	return false;
#else
	if (!is_directory(path)) {
		int err = ::mkdir(path.c_str(), S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
		return err != -1;
	}
	return true;
#endif
}

void ftl::config::addPath(const std::string &path) {
	auto &paths = rootCFG->getConfig()["paths"];
	paths.push_back(path);
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
		return false;
	} else {
		return false;
	}
}

static std::map<std::string, json_t*> config_index;
static std::map<std::string, ftl::Configurable*> config_instance;
static std::map<std::string, std::vector<ftl::Configurable*>> tag_index;

/*
 * Recursively URI index the JSON structure.
 */
static void _indexConfig(json_t &cfg) {
	if (cfg.is_object()) {
		auto id = cfg["$id"];
		if (id.is_string()) {
			config_index[id.get<string>()] = &cfg;
		}

		for (auto i : cfg.items()) {
			if (i.value().is_structured()) {
				_indexConfig(cfg[i.key()]);
			}
		}
	} // TODO:(Nick) Arrays....
}

ftl::Configurable *ftl::config::find(const std::string &uri) {
	std::string actual_uri = uri;
	if (uri[0] == '/') {
		if (uri.size() == 1) {
			return rootCFG;
		} else {
			actual_uri = rootCFG->getID() + uri;
		}
	}
	
	auto ix = config_instance.find(actual_uri);
	if (ix == config_instance.end()) return nullptr;
	else return (*ix).second;
}

const std::vector<Configurable*> &ftl::config::findByTag(const std::string &tag) {
	return tag_index[tag];
}

std::vector<std::string> ftl::config::list() {
	vector<string> r;
	for (auto i : config_instance) {
		r.push_back(i.first);
	}
	return r;
}

const std::vector<Configurable *> ftl::config::getChildren(const string &uri) {
	std::vector<Configurable *> children;
	for (const auto &[curi, c] : config_instance) {
		auto mismatch = std::mismatch(uri.begin(), uri.end(), curi.begin());
		if (mismatch.first == uri.end()) {
			children.push_back(c);
		}
	}
	return children;
}

void ftl::config::registerConfigurable(ftl::Configurable *cfg) {
	auto uri = cfg->get<string>("$id");
	if (!uri) {
		LOG(ERROR) << "Configurable object is missing $id property: " << cfg->getConfig();
		return;
	}
	auto ix = config_instance.find(*uri);
	if (ix != config_instance.end()) {
		// FIXME: HACK NOTE TODO SHOULD BE FATAL
		LOG(ERROR) << "Attempting to create a duplicate object: " << *uri;
	} else {
		config_instance[*uri] = cfg;
		LOG(INFO) << "Registering instance: " << *uri;

		auto tags = cfg->get<vector<string>>("tags");
		if (tags) {
			for (auto &t : *tags) {
				//LOG(INFO) << "REGISTER TAG: " << t;
				tag_index[t].push_back(cfg);
			}
		}
	}
}

json_t null_json;

/* To allow for custom tag format */
static std::string preprocessURI(const std::string &uri) {
	if (uri[0] == '[') {
		size_t closing = uri.find_last_of(']');
		string tags = uri.substr(1, closing-1);

		// TODO: Allow for multiple tags

		const auto &cfgs = ftl::config::findByTag(tags);

		// FIXME: Check for more than one tag result
		if (cfgs.size() > 0) {
			//LOG(INFO) << "PREPROC URI " << cfgs[0]->getID() + uri.substr(closing+1);
			return cfgs[0]->getID() + uri.substr(closing+1);
		} else {
			return uri;
		}
	} else if (uri[0] == '/') {
		return rootCFG->getID() + uri;
	} else {
		return uri;
	}
}

bool ftl::config::update(const std::string &puri, const json_t &value) {
	// Remove last component of URI
	string tail = "";
	string head = "";
	string uri = preprocessURI(puri);
	size_t last_hash = uri.find_last_of('#');
	if (last_hash != string::npos) {
		size_t last = uri.find_last_of('/');
		if (last != string::npos && last > last_hash) {
			tail = uri.substr(last+1);
			head = uri.substr(0, last);
		} else {
			tail = uri.substr(last_hash+1);
			head = uri.substr(0, last_hash);
		}
	} else {
		LOG(WARNING) << "Expected a # in an update URI: " << uri;
		return false;
	}

	Configurable *cfg = find(head);

	if (cfg) {
		DLOG(1) << "Updating CFG: " << head << "[" << tail << "] = " << value;
		cfg->set<json_t>(tail, value);
		return true;
	} else {
		DLOG(1) << "Updating: " << head << "[" << tail << "] = " << value;
		auto &r = resolve(head, false);

		if (!r.is_structured()) {
			LOG(ERROR) << "Cannot update property '" << tail << "' of '" << head << "'";
			return false;
		}

		// If there is an ID it still may be a configurable so check!
		if (r["$id"].is_string()) {
			Configurable *cfg = find(r["$id"].get<string>());
			if (cfg) {
				cfg->set<json_t>(tail, value);
				return true;
			}
		}

		r[tail] = value;
		return true;
	}
}

json_t &ftl::config::get(const std::string &puri) {
	// Remove last component of URI
	string tail = "";
	string head = "";
	string uri = preprocessURI(puri);
	size_t last_hash = uri.find_last_of('#');
	if (last_hash != string::npos) {
		size_t last = uri.find_last_of('/');
		if (last != string::npos && last > last_hash) {
			tail = uri.substr(last+1);
			head = uri.substr(0, last);
		} else {
			tail = uri.substr(last_hash+1);
			head = uri.substr(0, last_hash);
		}
	} else {
		LOG(WARNING) << "Expected a # in an update URI: " << uri;
		return null_json;
	}

	Configurable *cfg = find(head);

	if (cfg) {
		return cfg->getConfig()[tail];
	} else {
		return null_json;
	}
}

json_t &ftl::config::resolve(const std::string &puri, bool eager) {
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
		auto ix = config_index.find(u);
		if (ix == config_index.end()) {
			LOG(WARNING) << "Cannot find resource: " << u;
			return null_json;
		}

		auto ptr = nlohmann::json::json_pointer("/"+uri.getFragment());
		try {
			return (eager) ? resolve((*ix).second->at(ptr)) : (*ix).second->at(ptr);
		} catch(...) {
			LOG(WARNING) << "Resolve failed for " << puri;
			return null_json;
		}
	} else {
		LOG(WARNING) << "Resolve failed for " << puri;
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

		f = mergeConfig(FTL_GLOBAL_CONFIG_ROOT "/config.jsonc");
		found |= f;
		if (f) LOG(INFO) << "Loaded config: " << FTL_GLOBAL_CONFIG_ROOT "/config.jsonc";

		f = mergeConfig(FTL_LOCAL_CONFIG_ROOT "/config.json");
		found |= f;
		if (f) LOG(INFO) << "Loaded config: " << FTL_LOCAL_CONFIG_ROOT "/config.json";

		f = mergeConfig(FTL_LOCAL_CONFIG_ROOT "/config.jsonc");
		found |= f;
		if (f) LOG(INFO) << "Loaded config: " << FTL_LOCAL_CONFIG_ROOT "/config.jsonc";

		f = mergeConfig("./config.json");
		found |= f;
		if (f) LOG(INFO) << "Loaded config: " << "./config.json";

		f = mergeConfig("./config.jsonc");
		found |= f;
		if (f) LOG(INFO) << "Loaded config: " << "./config.jsonc";
		
		for (auto p : paths) {
			if (is_directory(p)) {
				f = mergeConfig(p+"/config.json");
				found |= f;
				if (f) LOG(INFO) << "Loaded config: " << p << "/config.json";

				f = mergeConfig(p+"/config.jsonc");
				found |= f;
				if (f) LOG(INFO) << "Loaded config: " << p << "/config.jsonc";
			}
		}
	}

	if (found) {
		//_indexConfig(config);
		return true;
	} else {
		return false;
	}
}

/**
 * Generate a map from command line option to value
 */
map<string, string> ftl::config::read_options(char ***argv, int *argc) {
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
			/* //auto ptr = nlohmann::json::json_pointer("/"+opt.first);
			auto ptr = ftl::config::resolve(*root->get<string>("$id") + string("/") + opt.first);

			LOG(INFO) << "PARAM RES TO " << (*root->get<string>("$id") + string("/") + opt.first);

			auto v = nlohmann::json::parse(opt.second);
			std::string type = ptr.type_name();
			if (type != "null" && v.type_name() != type) {
				LOG(ERROR) << "Incorrect type for argument " << opt.first << " - expected '" << type << "', got '" << v.type_name() << "'";
				continue;
			}
			ptr.swap(v);*/

			auto v = nlohmann::json::parse(opt.second);
			ftl::config::update(*root->get<string>("$id") + string("/") + opt.first, v);
		} catch(...) {
			LOG(ERROR) << "Unrecognised option: " << *root->get<string>("$id") << "#" << opt.first;
		}
	}
}

static bool sig_int_called = false;

static void signalIntHandler( int signum ) {
   std::cout << "Closing...\n";

   if (sig_int_called) quick_exit(-1);
   sig_int_called = true;

   // cleanup and close up stuff here  
   // terminate program  

   ftl::running = false;
}

Configurable *ftl::config::configure(ftl::config::json_t &cfg) {
	loguru::g_preamble_date = false;
	loguru::g_preamble_uptime = false;
	loguru::g_preamble_thread = false;
	int argc = 1;
	const char *argv[]{"d",0};
	loguru::init(argc, const_cast<char**>(argv), "--verbosity");

	config_index.clear();
	config_instance.clear();

	config = cfg;
	_indexConfig(config);
	Configurable *rootcfg = create<Configurable>(config);
	return rootcfg;
}

static bool doing_cleanup = false;
void ftl::config::cleanup() {
	if (doing_cleanup) return;
	doing_cleanup = true;
	for (auto f : config_instance) {
		delete f.second;
	}
	config_instance.clear();
	doing_cleanup = false;
}

void ftl::config::removeConfigurable(Configurable *cfg) {
	if (doing_cleanup) return;

	auto i = config_instance.find(cfg->getID());
	if (i != config_instance.end()) {
		config_instance.erase(i);
	}
}

std::string ftl::config::_getID(nlohmann::json &link) {
	if (!link["$id"].is_string()) {
		throw FTL_Error("Entity does not have $id or parent: " << link);
	}
	return link["$id"].get<std::string>();
}

std::vector<nlohmann::json*> ftl::config::_createArray(ftl::Configurable *parent, const std::string &name) {
	nlohmann::json &base = (!parent->getConfig()[name].is_null())
			? parent->getConfig()[name]
			: ftl::config::resolve(parent->getConfig())[name];

	std::vector<nlohmann::json*> result;

	if (base.is_array()) {
		int i=0;
		for (auto &entity : base) {
			if (entity.is_object()) {
				if (!entity["$id"].is_string()) {
					std::string id_str = *parent->get<std::string>("$id");
					if (id_str.find('#') != std::string::npos) {
						entity["$id"] = id_str + std::string("/") + name + std::string("/") + std::to_string(i);
					} else {
						entity["$id"] = id_str + std::string("#") + name + std::string("/") + std::to_string(i);
					}
				}

				result.push_back(&entity);
			} else if (entity.is_null()) {
				// Must create the object from scratch...
				std::string id_str = *parent->get<std::string>("$id");
				if (id_str.find('#') != std::string::npos) {
					id_str = id_str + std::string("/") + name + std::string("/") + std::to_string(i);
				} else {
					id_str = id_str + std::string("#") + name + std::string("/") + std::to_string(i);
				}
				parent->getConfig()[name] = {
					// cppcheck-suppress constStatement
					{"$id", id_str}
				};

				nlohmann::json &entity2 = parent->getConfig()[name];
				result.push_back(&entity2);
			}
			i++;
		}
	} else {
		//LOG(WARNING) << "Expected an array for '" << name << "' in " << parent->getID();
	}

	return std::move(result);
}

nlohmann::json &ftl::config::_create(ftl::Configurable *parent, const std::string &name) {
	nlohmann::json &entity = (!parent->getConfig()[name].is_null())
			? parent->getConfig()[name]
			: ftl::config::resolve(parent->getConfig())[name];

	if (entity.is_object()) {
		if (!entity["$id"].is_string()) {
			std::string id_str = *parent->get<std::string>("$id");
			if (id_str.find('#') != std::string::npos) {
				entity["$id"] = id_str + std::string("/") + name;
			} else {
				entity["$id"] = id_str + std::string("#") + name;
			}
		}

		return entity;
	} else if (entity.is_null()) {
		// Must create the object from scratch...
		std::string id_str = *parent->get<std::string>("$id");
		if (id_str.find('#') != std::string::npos) {
			id_str = id_str + std::string("/") + name;
		} else {
			id_str = id_str + std::string("#") + name;
		}
		parent->getConfig()[name] = {
			// cppcheck-suppress constStatement
			{"$id", id_str}
		};

		nlohmann::json &entity2 = parent->getConfig()[name];
		return entity2;
	}

	throw FTL_Error("Unable to create Configurable entity '" << name << "'");
}

nlohmann::json *ftl::config::createJSON() {
	return new nlohmann::json;
}

void ftl::config::destroyJSON(nlohmann::json *j) {
	delete j;
}

void ftl::config::copyJSON(nlohmann::json *dst, nlohmann::json *src) {
	*dst = *src;
}

void ftl::config::parseJSON(nlohmann::json &dst, const std::string &src) {
	dst = nlohmann::json::parse(src);
}

std::string ftl::config::dumpJSON(const nlohmann::json &json) {
	return json.dump();
}

template <typename T>
std::optional<T> ftl::config::getJSON(nlohmann::json *config, const std::string &name) {
	try {
		return (*config)[name].get<T>();
	} catch (...) {
		return {};
	}
}

template <typename T>
void ftl::config::setJSON(nlohmann::json *config, const std::string &name, T value) {
	(*config)[name] = value;
}

template std::optional<float> ftl::config::getJSON<float>(nlohmann::json *config, const std::string &name);
template std::optional<int> ftl::config::getJSON<int>(nlohmann::json *config, const std::string &name);
template std::optional<std::string> ftl::config::getJSON<std::string>(nlohmann::json *config, const std::string &name);

template void ftl::config::setJSON<float>(nlohmann::json *config, const std::string &name, float value);
template void ftl::config::setJSON<int>(nlohmann::json *config, const std::string &name, int value);
template void ftl::config::setJSON<std::string>(nlohmann::json *config, const std::string &name, std::string value);

Configurable *ftl::config::configure(int argc, char **argv, const std::string &root) {
	loguru::g_preamble_date = false;
	loguru::g_preamble_uptime = false;
	loguru::g_preamble_thread = false;
	loguru::init(argc, argv, "--verbosity");
	argc--;
	argv++;

#ifndef WIN32
	signal(SIGINT,signalIntHandler);
#endif  // WIN32

	// Process Arguments
	auto options = ftl::config::read_options(&argv, &argc);
	
	vector<string> paths(argc);
	while (argc-- > 0) {
		paths.push_back(argv[0]);
		argv++;
	}

	if (!findConfiguration(options["config"], paths)) {
		LOG(FATAL) << "Could not find any configuration!";
	}

	string root_str = (options.find("root") != options.end()) ? nlohmann::json::parse(options["root"]).get<string>() : root;

	if (options.find("id") != options.end()) config["$id"] = nlohmann::json::parse(options["id"]).get<string>();
	_indexConfig(config);

	Configurable *rootcfg = create<Configurable>(config);
	if (root_str.size() > 0) {
		LOG(INFO) << "Setting root to " << root_str;
		rootcfg = create<Configurable>(rootcfg, root_str);
	}

	//root_config = rootcfg->getConfig();
	rootCFG = rootcfg;
	rootcfg->set("paths", paths);
	process_options(rootcfg, options);

	if (rootcfg->get<std::string>("branch")) {
		ftl::branch_name = *rootcfg->get<std::string>("branch");
	}
	rootcfg->on("branch", [](const ftl::config::Event &e) {
		if (e.entity->get<std::string>("branch")) {
			ftl::branch_name = *e.entity->get<std::string>("branch");
		}
	});

	// Some global settings
	ftl::timer::setInterval(1000 / rootcfg->value("fps",20));

	// Check CUDA
	ftl::cuda::initialise();

	int pool_size = rootcfg->value("thread_pool_factor", 2.0f)*std::thread::hardware_concurrency();
	if (pool_size != ftl::pool.size()) ftl::pool.resize(pool_size);


	//LOG(INFO) << "CONFIG: " << config["vision_default"];
	//CHECK_EQ( &config, config_index["ftl://utu.fi"] );

	return rootcfg;
}

