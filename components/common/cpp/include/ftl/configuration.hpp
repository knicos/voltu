#ifndef _FTL_COMMON_CONFIGURATION_HPP_
#define _FTL_COMMON_CONFIGURATION_HPP_

#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <optional>

namespace ftl {

extern nlohmann::json config;

bool is_directory(const std::string &path);
bool is_file(const std::string &path);
bool create_directory(const std::string &path);

bool is_video(const std::string &file);

std::optional<std::string> locateFile(const std::string &name);

std::vector<std::string> configure(int argc, char **argv, const std::string &app);

};

#endif  // _FTL_COMMON_CONFIGURATION_HPP_

