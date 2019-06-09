#include "net.hpp"
#include <vector>
#include <thread>
#include <chrono>
#include <shared_mutex>

using ftl::rgbd::detail::NetSource;
using ftl::net::Universe;
using ftl::UUID;
using std::string;
using ftl::rgbd::Camera;
using std::shared_mutex;
using std::unique_lock;
using std::vector;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;

bool NetSource::_getCalibration(Universe &net, const UUID &peer, const string &src, ftl::rgbd::Camera &p) {
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

NetSource::NetSource(ftl::rgbd::Source *host)
		: ftl::rgbd::detail::Source(host), active_(false) {

	_updateURI();

	h_ = host_->getNet()->onConnect([this](ftl::net::Peer *p) {
		if (active_) return;
		LOG(INFO) << "NetSource restart...";
		_updateURI();
	});
}

NetSource::~NetSource() {
	if (uri_.size() > 0) {
		host_->getNet()->unbind(uri_);
	}

	host_->getNet()->removeCallback(h_);
}

void NetSource::_recv(const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
	unique_lock<shared_mutex> lk(host_->mutex());

	cv::imdecode(jpg, cv::IMREAD_COLOR, &rgb_);
	//Mat(rgb_.size(), CV_16UC1);
	cv::imdecode(d, cv::IMREAD_UNCHANGED, &depth_);
	depth_.convertTo(depth_, CV_32FC1, 1.0f/(16.0f*100.0f));

	N_--;
	if (N_ == 0) {
		N_ += 10;
		if (!host_->getNet()->send(peer_, "get_stream", *host_->get<string>("uri"), 10, 0, host_->getNet()->id(), *host_->get<string>("uri"))) {
			active_ = false;
		}
	}
}

void NetSource::setPose(const Eigen::Matrix4f &pose) {
	if (!active_) return;

	vector<unsigned char> vec((unsigned char*)pose.data(), (unsigned char*)(pose.data()+(pose.size())));
	try {
		if (!host_->getNet()->send(peer_, "set_pose", *host_->get<string>("uri"), vec)) {
			active_ = false;
		}
	} catch (...) {

	}
	Source::setPose(pose);
}

void NetSource::_updateURI() {
	//unique_lock<mutex> lk(mutex_);
	active_ = false;
	auto uri = host_->get<string>("uri");

	// TODO(Nick) If URI changes then must unbind + rebind.
	if (uri_.size() > 0) {
		host_->getNet()->unbind(uri_);
	}

	if (uri) {
		auto p = host_->getNet()->findOne<ftl::UUID>("find_stream", *uri);
		if (!p) {
			LOG(ERROR) << "Could not find stream: " << *uri;
			return;
		}
		peer_ = *p;

		has_calibration_ = _getCalibration(*host_->getNet(), peer_, *uri, params_);

		host_->getNet()->bind(*uri, [this](const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
			_recv(jpg, d);
		});

		N_ = 10;

		// Initiate stream with request for first 10 frames
		try {
			host_->getNet()->send(peer_, "get_stream", *uri, 10, 0, host_->getNet()->id(), *uri);
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

bool NetSource::grab() {
	// net_.broadcast("grab");
	return true;
}

bool NetSource::isReady() {
	return has_calibration_ && !rgb_.empty();
}
