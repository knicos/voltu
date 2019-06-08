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
	try {
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
	} catch (...) {
		return false;
	}
}

NetSource::NetSource(nlohmann::json &config) : RGBDSource(config) {

}

NetSource::NetSource(nlohmann::json &config, ftl::net::Universe *net)
		: RGBDSource(config, net), active_(false) {

	on("uri", [this](const config::Event &e) {
		_updateURI();
	});

	_updateURI();

	h_ = net->onConnect([this](ftl::net::Peer *p) {
		LOG(INFO) << "NetSource restart...";
		_updateURI();
	});
}

NetSource::~NetSource() {
	if (uri_.size() > 0) {
		net_->unbind(uri_);
	}

	net_->removeCallback(h_);
}

void NetSource::_recv(const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
	cv::imdecode(jpg, cv::IMREAD_COLOR, &rgb_);
	//Mat(rgb_.size(), CV_16UC1);
	cv::imdecode(d, cv::IMREAD_UNCHANGED, &depth_);
	depth_.convertTo(depth_, CV_32FC1, 1.0f/(16.0f*100.0f));

	N_--;
	if (N_ == 0) {
		N_ += 10;
		if (!net_->send(peer_, "get_stream", *get<string>("uri"), 10, 0, net_->id(), *get<string>("uri"))) {
			active_ = false;
		}
	}
}

void NetSource::setPose(const Eigen::Matrix4f &pose) {
	if (!active_) return;

	vector<unsigned char> vec((unsigned char*)pose.data(), (unsigned char*)(pose.data()+(pose.size())));
	try {
		if (!net_->send(peer_, "set_pose", *get<string>("uri"), vec)) {
			active_ = false;
		}
	} catch (...) {

	}
	RGBDSource::setPose(pose);
}

void NetSource::_updateURI() {
	unique_lock<mutex> lk(mutex_);
	active_ = false;
	auto uri = get<string>("uri");

	// TODO(Nick) If URI changes then must unbind + rebind.
	if (uri_.size() > 0) {
		net_->unbind(uri_);
	}

	if (uri) {
		auto p = net_->findOne<ftl::UUID>("find_stream", *uri);
		if (!p) {
			LOG(ERROR) << "Could not find stream: " << *uri;
			return;
		}
		peer_ = *p;

		has_calibration_ = _getCalibration(*net_, peer_, *uri, params_);

		net_->bind(*uri, [this](const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
			unique_lock<mutex> lk(mutex_);
			_recv(jpg, d);
		});

		N_ = 10;

		// Initiate stream with request for first 10 frames
		try {
			net_->send(peer_, "get_stream", *uri, 10, 0, net_->id(), *uri);
		} catch(...) {
			LOG(ERROR) << "Could not connect to stream " << *uri;
		}

		uri_ = *uri;
		active_ = true;
	} else {
		uri_ = "";
		LOG(WARNING) << "NetSource URI is missing";
	}
}

void NetSource::grab() {
	// net_.broadcast("grab");
}

bool NetSource::isReady() {
	return has_calibration_ && !rgb_.empty();
}
