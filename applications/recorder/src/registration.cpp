#include "registration.hpp"
#include <fstream>
#define LOGURU_WITH_STREAMS 1
#include <loguru.hpp>


namespace ftl {
namespace registration {

using ftl::rgbd::Camera;
using ftl::rgbd::Source;

using std::string;
using std::vector;
using std::pair;
using std::map;
using std::optional;

using cv::Mat;
using Eigen::Matrix4f;
using Eigen::Matrix4d;

void from_json(nlohmann::json &json, map<string, Matrix4d> &transformations) {
	for (auto it = json.begin(); it != json.end(); ++it) {
		Eigen::Matrix4d m;
		auto data = m.data();
		for(size_t i = 0; i < 16; i++) { data[i] = it.value()[i]; }
		transformations[it.key()] = m;
	}
}

void to_json(nlohmann::json &json, map<string, Matrix4d> &transformations) {
	for (auto &item : transformations) {
		auto val = nlohmann::json::array();
		for(size_t i = 0; i < 16; i++) { val.push_back((float) item.second.data()[i]); }
		json[item.first] = val;
	}
}

bool loadTransformations(const string &path, map<string, Matrix4d> &data) {
	std::ifstream file(path);
	if (!file.is_open()) {
		LOG(ERROR) << "Error loading transformations from file " << path;
		return false;
	}
	
	nlohmann::json json_registration;
	file >> json_registration;
	from_json(json_registration, data);
	return true;
}

bool saveTransformations(const string &path, map<string, Matrix4d> &data) {
	nlohmann::json data_json;
	to_json(data_json, data);
	std::ofstream file(path);

	if (!file.is_open()) {
		LOG(ERROR) << "Error writing transformations to file " << path;
		return false;
	}

	file << std::setw(4) << data_json;
	return true;
}


} // namespace registration
} // namespace ftl
