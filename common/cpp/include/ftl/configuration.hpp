#ifndef _FTL_COMMON_CONFIGURATION_HPP_
#define _FTL_COMMON_CONFIGURATION_HPP_

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace ftl {

extern nlohmann::json config;

std::vector<std::string> configure(int argc, char **argv, const std::string &app);

};

#endif  // _FTL_COMMON_CONFIGURATION_HPP_

