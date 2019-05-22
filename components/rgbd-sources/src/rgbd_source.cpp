#include <ftl/rgbd_source.hpp>

using ftl::rgbd::RGBDSource;
using ftl::Configurable;

std::map<std::string, std::function<RGBDSource*(nlohmann::json&,ftl::net::Universe*)>>
		RGBDSource::sources__;

RGBDSource::RGBDSource(nlohmann::json &config) : Configurable(config), net_(nullptr) {

}

RGBDSource::RGBDSource(nlohmann::json &config, ftl::net::Universe *net) : Configurable(config), net_(net) {

}

RGBDSource::~RGBDSource() {

}

bool RGBDSource::isReady() {
	return false;
}

bool RGBDSource::snapshot(const std::string &fileprefix) {
	cv::Mat rgb;
	cv::Mat depth;
	getRGBD(rgb, depth);

	cv::Mat d2;
    depth.convertTo(d2, CV_16UC1, 16*100);

	cv::imwrite(fileprefix+"-RGB.jpg", rgb);
	cv::imwrite(fileprefix+"-DEPTH.png",depth);
}

RGBDSource *RGBDSource::create(nlohmann::json &config, ftl::net::Universe *net) {
	if (config["type"].type_name() != "string") return nullptr;
	if (sources__.count(config["type"]) != 1) return nullptr;
	return sources__[config["type"]](config, net);
}

void RGBDSource::_register(const std::string &n,
		std::function<RGBDSource*(nlohmann::json&,ftl::net::Universe*)> f) {
	LOG(INFO) << "Register RGB-D Source: " << n;
	sources__[n] = f;
}

#include <ftl/net_source.hpp>
static RGBDSource::Register netsource("net", ftl::rgbd::NetSource::create);
#include <ftl/stereovideo_source.hpp>
static RGBDSource::Register svsource("stereovideo", ftl::rgbd::StereoVideoSource::create);
