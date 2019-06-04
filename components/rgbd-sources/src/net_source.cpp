#include <ftl/net_source.hpp>
#include <vector>
#include <thread>
#include <chrono>

using ftl::rgbd::NetSource;
using ftl::net::Universe;
using ftl::UUID;
using std::string;
using ftl::rgbd::CameraParameters;
using std::mutex;
using std::unique_lock;
using std::vector;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;

bool NetSource::_getCalibration(Universe &net, const UUID &peer, const string &src, ftl::rgbd::CameraParameters &p) {
	while(true) {
		auto buf = net.call<vector<unsigned char>>(peer_, "source_calibration", src);
		if (buf.size() > 0) {
			memcpy((char*)&p, buf.data(), buf.size());
			
			if (sizeof(p) != buf.size()) {
				LOG(ERROR) << "Corrupted calibration";
				return false;
			}

			LOG(INFO) << "Calibration received: " << p.cx << ", " << p.cy << ", " << p.fx << ", " << p.fy;
			
			return true;
		} else {
			LOG(INFO) << "Could not get calibration, retrying";
			sleep_for(milliseconds(500));
		}
	}
}

NetSource::NetSource(nlohmann::json &config) : RGBDSource(config) {

}

NetSource::NetSource(nlohmann::json &config, ftl::net::Universe *net)
		: RGBDSource(config, net) {

	auto uri = get<string>("uri");
	if (!uri) {
		LOG(ERROR) << "NetSource does not have a URI";
		return;
	}
	auto p = net->findOne<ftl::UUID>("find_stream", *uri);
	if (!p) {
		LOG(ERROR) << "Could not find stream: " << *uri;
		return;
	}
	peer_ = *p;

	has_calibration_ = _getCalibration(*net, peer_, *uri, params_);
	
	net->bind(*uri, [this](const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
		unique_lock<mutex> lk(mutex_);
		_recv(jpg, d);
	});

	N_ = 10;

	// Initiate stream with request for first 10 frames
	net->send(peer_, "get_stream", *uri, 10, 0, net->id(), *uri);
}

NetSource::~NetSource() {
	// TODO Unsubscribe
}

void NetSource::_recv(const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
	cv::imdecode(jpg, cv::IMREAD_COLOR, &rgb_);
	//Mat(rgb_.size(), CV_16UC1);
	cv::imdecode(d, cv::IMREAD_UNCHANGED, &depth_);
	depth_.convertTo(depth_, CV_32FC1, 1.0f/(16.0f*100.0f));

	N_--;
	if (N_ == 0) {
		N_ += 10;
		net_->send(peer_, "get_stream", *get<string>("uri"), 10, 0, net_->id(), *get<string>("uri"));
	}
}

void NetSource::grab() {
	// net_.broadcast("grab");
}

bool NetSource::isReady() {
	return has_calibration_ && !rgb_.empty();
}
