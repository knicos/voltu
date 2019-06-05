#include <ftl/rgbd_source.hpp>

using ftl::rgbd::RGBDSource;
using ftl::Configurable;
using std::string;
using std::mutex;
using std::unique_lock;

std::map<std::string, std::function<RGBDSource*(nlohmann::json&,ftl::net::Universe*)>> *RGBDSource::sources__ = nullptr;

RGBDSource::RGBDSource(nlohmann::json &config) : Configurable(config), net_(nullptr) {

}

RGBDSource::RGBDSource(nlohmann::json &config, ftl::net::Universe *net) : Configurable(config), net_(net) {

}

RGBDSource::~RGBDSource() {

}

bool RGBDSource::isReady() {
	return false;
}

void RGBDSource::getRGBD(cv::Mat &rgb, cv::Mat &depth) {
	unique_lock<mutex> lk(mutex_);
	rgb_.copyTo(rgb);
	depth_.copyTo(depth);
}

Eigen::Vector4f RGBDSource::point(uint ux, uint uy) {
	const auto &params = getParameters();
	const float x = ((float)ux-params.width/2) / params.fx;
	const float y = ((float)uy-params.height/2) / params.fy;

	unique_lock<mutex> lk(mutex_);
	const float depth = depth_.at<float>(uy,ux);
	return Eigen::Vector4f(x*depth,y*depth,depth,1.0);
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
	auto &cfg = ftl::config::resolve(config);
	if (!cfg["type"].is_string()) {
		LOG(ERROR) << "Missing RGB-D source type: " << cfg["type"].type_name();
		//return nullptr;
	}
	if (sources__->count(cfg["type"].get<string>()) != 1) return nullptr;
	return (*sources__)[cfg["type"].get<string>()](config, net);
}

void RGBDSource::_register(const std::string &n,
		std::function<RGBDSource*(nlohmann::json&,ftl::net::Universe*)> f) {
	if (!sources__) sources__ = new std::map<std::string, std::function<RGBDSource*(nlohmann::json&,ftl::net::Universe*)>>;
	//LOG(INFO) << "Register RGB-D Source: " << n;
	(*sources__)[n] = f;
}

#include <ftl/net_source.hpp>
static RGBDSource::Register netsource("net", ftl::rgbd::NetSource::create);
#include <ftl/stereovideo_source.hpp>
static RGBDSource::Register svsource("stereovideo", ftl::rgbd::StereoVideoSource::create);
