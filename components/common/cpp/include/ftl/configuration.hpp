/**
 * @file configuration.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#pragma once
#ifndef _FTL_COMMON_CONFIGURATION_HPP_
#define _FTL_COMMON_CONFIGURATION_HPP_

#include <nlohmann/json_fwd.hpp>
#include <string>
#include <vector>
#include <optional>
#include <unordered_set>

namespace ftl {

extern bool running;				///< Set to false to bring the system down
extern int exit_code;				///< Specify exit code to eventually use
extern std::string branch_name;		///< Print out desired git branch name to use at restart

class Configurable;

bool is_directory(const std::string &path);
bool create_directory(const std::string &path);
bool is_video(const std::string &file);
std::vector<std::string> directory_listing(const std::string &path);

nlohmann::json loadJSON(const std::string &path);

bool saveJSON(const std::string &path, nlohmann::json &json);

namespace config {

typedef nlohmann::json json_t;

void addPath(const std::string &path);

std::optional<std::string> locateFile(const std::string &name);

std::map<std::string, std::string> read_options(char ***argv, int *argc);

/**
 * Called first to set up the entire system.
 * 
 * @param root The initial key in the config file to use as root config.
 */
Configurable *configure(int argc, char **argv, const std::string &root, const std::unordered_set<std::string> &restoreable={});

Configurable *configure(json_t &);

nlohmann::json &getRestore(const std::string &key);
nlohmann::json &getDefault(const std::string &key);

void cleanup();

void save();

void removeConfigurable(Configurable *cfg);

/**
 * Change a configuration value based upon a URI. Return true if changed,
 * false if it was not able to change.
 */
bool update(const std::string &puri, const json_t &value);

json_t &get(const std::string &puri);

/**
 * Resolve a JSON schema reference, but do not wait for a remote reference
 * if it is not available. A null entity is returned if not resolved.
 */
json_t &resolve(const std::string &, bool eager=true);

/**
 * Resolve a reference object, or if not a reference object it simply returns
 * the original object. A reference object with additional properties other
 * than $ref will result in a new merged object.
 */
json_t &resolve(json_t &ref);

/**
 * Resolve a JSON schema reference and block until such a reference can be
 * resolved correctly.
 */
json_t &resolveWait(const std::string &);

/**
 * Using a JSON schema reference, find an existing instance of a Configurable
 * object for that reference. Or return nullptr if not found.
 */
Configurable *find(const std::string &uri);

/**
 * Add an alternative URI for a configurable.
 */
void alias(const std::string &uri, Configurable *cfg);

/**
 * Get all configurables that contain a specified tag. Tags are given under the
 * "tags" property as an array of strings, but only during configurable
 * construction.
 */
const std::vector<Configurable *> &findByTag(const std::string &tag);

std::vector<std::string> list();

/**
 * Recursively get all children of a configurable. The given configurable is
 * also included in the vector, unless it is null,
 * in which case an empty vector is returned.
 */
const std::vector<Configurable *> getChildren(const std::string &uri);

/**
 * Adds a Configurable instance to the database of instances so that it can
 * then be resolved using find().
 */
void registerConfigurable(Configurable *cfg);

/**
 * Create a new configurable directly from a raw object. This should not be used.
 */
template <typename T, typename... ARGS>
T *create(json_t &link, ARGS ...args);

/**
 * Create a configurable from an attribute of a parent configurable.
 */
template <typename T, typename... ARGS>
T *create(ftl::Configurable *parent, const std::string &name, ARGS ...args);

nlohmann::json &_create(ftl::Configurable *parent, const std::string &name);
std::vector<nlohmann::json*> _createArray(ftl::Configurable *parent, const std::string &name);

std::string _getID(nlohmann::json &);

nlohmann::json *createJSON();
void destroyJSON(nlohmann::json*);
void copyJSON(nlohmann::json *dst, nlohmann::json *src);
void parseJSON(nlohmann::json &dst, const std::string &src);
std::string dumpJSON(const nlohmann::json &json);

template <typename T>
std::optional<T> getJSON(nlohmann::json *config, const std::string &name);

template <typename T>
void setJSON(nlohmann::json *config, const std::string &name, T value);

/**
 * Create a configurable rooted on a parent but with a specific object
 * that is not directly a child of the parent. Used by RGB-D Factory.
 */
//template <typename T, typename... ARGS>
//T *create(ftl::Configurable *parent, json_t &obj, const std::string &name, ARGS ...args);

template <typename T, typename... ARGS>
std::vector<T*> createArray(ftl::Configurable *parent, const std::string &name, ARGS ...args);

void destroy(ftl::Configurable *);

void set(const std::string &uri, const nlohmann::json &);

}  // namespace config

// Deprecated
using config::create;
using config::createArray;
using config::locateFile;
using config::configure;

}  // namespace ftl

#include <ftl/configurable.hpp>

// ==== Impl ===================================================================

extern template std::optional<float> ftl::config::getJSON<float>(nlohmann::json *config, const std::string &name);
extern template std::optional<int> ftl::config::getJSON<int>(nlohmann::json *config, const std::string &name);
extern template std::optional<std::string> ftl::config::getJSON<std::string>(nlohmann::json *config, const std::string &name);

extern template void ftl::config::setJSON<float>(nlohmann::json *config, const std::string &name, float value);
extern template void ftl::config::setJSON<int>(nlohmann::json *config, const std::string &name, int value);
extern template void ftl::config::setJSON<std::string>(nlohmann::json *config, const std::string &name, std::string value);

template <typename T, typename... ARGS>
T *ftl::config::create(json_t &link, ARGS ...args) {
	std::string id = _getID(link);

	ftl::Configurable *cfg = ftl::config::find(id);
	if (!cfg) {
		cfg = new T(link, args...);
	} else {
		// Make sure configurable has newest object pointer
		cfg->patchPtr(link);
	}

	T* ptr = dynamic_cast<T*>(cfg);
	if (ptr) {
		return ptr;
	}
	else {
		throw FTL_Error("Configuration URI object is of wrong type: " << id);
	}
}

template <typename T, typename... ARGS>
T *ftl::config::create(ftl::Configurable *parent, const std::string &name, ARGS ...args) {
	return create<T>(_create(parent, name), args...);
}

template <typename T, typename... ARGS>
std::vector<T*> ftl::config::createArray(ftl::Configurable *parent, const std::string &name, ARGS ...args) {
	std::vector<T*> result;
	std::vector<nlohmann::json*> entities = _createArray(parent, name);

	for (auto *e: entities) {
		result.push_back(create<T>(*e, args...));
	}

	return result;
}



#endif  // _FTL_COMMON_CONFIGURATION_HPP_

