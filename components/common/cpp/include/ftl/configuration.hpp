#pragma once
#ifndef _FTL_COMMON_CONFIGURATION_HPP_
#define _FTL_COMMON_CONFIGURATION_HPP_

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>
#include <nlohmann/json.hpp>
//#include <ftl/configurable.hpp>
#include <string>
#include <vector>
#include <optional>

namespace ftl {

extern bool running;
extern int exit_code;

class Configurable;

bool is_directory(const std::string &path);
bool is_file(const std::string &path);
bool create_directory(const std::string &path);
bool is_video(const std::string &file);

namespace config {

typedef nlohmann::json json_t;

std::optional<std::string> locateFile(const std::string &name);

Configurable *configure(int argc, char **argv, const std::string &root);

/**
 * Change a configuration value based upon a URI. Return true if changed,
 * false if it was not able to change.
 */
bool update(const std::string &puri, const json_t &value);

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
 * Adds a Configurable instance to the database of instances so that it can
 * then be resolved using find().
 */
void registerConfigurable(Configurable *cfg);

template <typename T, typename... ARGS>
T *create(json_t &link, ARGS ...args);

template <typename T, typename... ARGS>
T *create(ftl::Configurable *parent, const std::string &name, ARGS ...args);

void set(const std::string &uri, const nlohmann::json &);

}  // namespace config

// Deprecated
using config::create;
using config::locateFile;
using config::configure;

}  // namespace ftl

#include <ftl/configurable.hpp>

template <typename T, typename... ARGS>
T *ftl::config::create(json_t &link, ARGS ...args) {
    //auto &r = link; // = ftl::config::resolve(link);

    if (!link["$id"].is_string()) {
        LOG(FATAL) << "Entity does not have $id or parent: " << link;
        return nullptr;
    }

    ftl::Configurable *cfg = ftl::config::find(link["$id"].get<std::string>());
    if (!cfg) {
       // try {
            cfg = new T(link, args...);
        //} catch(...) {
       //     LOG(FATAL) << "Could not construct " << link;
        //}
    }

    try {
        return dynamic_cast<T*>(cfg);
    } catch(...) {
        LOG(FATAL) << "Configuration URI object is of wrong type: " << link;
        return nullptr;
    }
}

template <typename T, typename... ARGS>
T *ftl::config::create(ftl::Configurable *parent, const std::string &name, ARGS ...args) {
    //nlohmann::json &entity = ftl::config::resolve(parent->getConfig()[name]);
    nlohmann::json &entity = (!parent->getConfig()[name].is_null()) ? parent->getConfig()[name] : ftl::config::resolve(parent->getConfig())[name];

    if (entity.is_object()) {
        if (!entity["$id"].is_string()) {
            std::string id_str = *parent->get<std::string>("$id");
            if (id_str.find('#') != std::string::npos) {
                entity["$id"] = id_str + std::string("/") + name;
            } else {
                entity["$id"] = id_str + std::string("#") + name;
            }
        }

        return create<T>(entity, args...);
    } else if (entity.is_null()) {
        // Must create the object from scratch...
        std::string id_str = *parent->get<std::string>("$id");
        if (id_str.find('#') != std::string::npos) {
            id_str = id_str + std::string("/") + name;
        } else {
            id_str = id_str + std::string("#") + name;
        }
        parent->getConfig()[name] = {
            {"$id", id_str}
        };

        nlohmann::json &entity2 = parent->getConfig()[name];
        return create<T>(entity2, args...);
    }
}

#endif  // _FTL_COMMON_CONFIGURATION_HPP_

