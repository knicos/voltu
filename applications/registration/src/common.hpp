#ifndef _FTL_REGISTRATION_COMMON_HPP_
#define _FTL_REGISTRATION_COMMON_HPP_

#include <ftl/rgbd/source.hpp>
#include <nlohmann/json.hpp>

#include <string>
#include <map>

namespace ftl {
namespace registration {

bool loadTransformations(const std::string &path, std::map<std::string, Eigen::Matrix4d> &data);
bool saveTransformations(const std::string &path, std::map<std::string, Eigen::Matrix4d> &data);
void from_json(nlohmann::json &json, std::map<std::string, Eigen::Matrix4d> &transformations);
void to_json(nlohmann::json &json, std::map<std::string, Eigen::Matrix4d> &transformations);

}
}

#endif  // _FTL_REGISTRATION_COMMON_HPP_
