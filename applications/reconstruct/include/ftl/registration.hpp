#ifndef _FTL_RECONSTRUCT_REGISTRATION_HPP_
#define _FTL_RECONSTRUCT_REGISTRATION_HPP_

#include <ftl/config.h>
#include <ftl/configurable.hpp>
#include <ftl/rgbd.hpp>
#include <opencv2/opencv.hpp>


namespace ftl {
namespace registration {

void to_json(nlohmann::json &json, std::map<std::string, Eigen::Matrix4d> &transformations);
void from_json(nlohmann::json &json, std::map<std::string, Eigen::Matrix4d> &transformations);

bool loadTransformations(const std::string &path, std::map<std::string, Eigen::Matrix4d> &data);
bool saveTransformations(const std::string &path, std::map<std::string, Eigen::Matrix4d> &data);

}
}

#endif  // _FTL_RECONSTRUCT_REGISTRATION_HPP_