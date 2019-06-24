#include "common.hpp"

using std::string;
using std::map;

void ftl::registration::from_json(nlohmann::json &json, map<string, Eigen::Matrix4d> &transformations) {
	for (auto it = json.begin(); it != json.end(); ++it) {
		Eigen::Matrix4d m;
		auto data = m.data();
		for(size_t i = 0; i < 16; i++) { data[i] = it.value()[i]; }
		transformations[it.key()] = m;
	}
}

void ftl::registration::to_json(nlohmann::json &json, map<string, Eigen::Matrix4d> &transformations) {
	for (auto &item : transformations) {
		auto val = nlohmann::json::array();
		for(size_t i = 0; i < 16; i++) { val.push_back((float) item.second.data()[i]); }
		json[item.first] = val;
	}
}

bool ftl::registration::saveTransformations(const string &path, map<string, Eigen::Matrix4d> &data) {
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

bool ftl::registration::loadTransformations(const string &path, map<string, Eigen::Matrix4d> &data) {
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

