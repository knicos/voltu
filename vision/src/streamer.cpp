#include <glog/logging.h>
#include <ftl/streamer.hpp>
#include <vector>

using ftl::Streamer;
using ftl::net::Universe;
using cv::Mat;
using nlohmann::json;
using std::string;
using std::vector;

Streamer::Streamer(Universe &net, json &config) : net_(net), config_(config) {
	uri_ = string("ftl://utu.fi/")+(string)config["name"]+string("/rgb-d");
	net.createResource(uri_);
}

Streamer::~Streamer() {

}

void Streamer::send(const Mat &rgb, const Mat &depth) {
	// Compress the rgb as jpeg.
	vector<unsigned char> rgb_buf;
	cv::imencode(".jpg", rgb, rgb_buf);
	net_.publish(uri_, rgb_buf);
	
	LOG(INFO) << "JPG Size = " << ((float)rgb_buf.size() / (1024.0f*1024.0f)) << "Mb";
}

