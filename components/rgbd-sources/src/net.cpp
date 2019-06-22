#include "net.hpp"
#include <vector>
#include <thread>
#include <chrono>
#include <shared_mutex>

#include "colour.hpp"

#include <ftl/rgbd/streamer.hpp>

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

	gamma_ = host->value("gamma", 1.0f);
	temperature_ = host->value("temperature", 6500);

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
	cv::Mat tmp_rgb, tmp_depth;

	// Decode in temporary buffers to prevent long locks
	cv::imdecode(jpg, cv::IMREAD_COLOR, &tmp_rgb);
	cv::imdecode(d, cv::IMREAD_UNCHANGED, &tmp_depth);

	// Lock host to prevent grab
	UNIQUE_LOCK(host_->mutex(),lk);
	rgb_ = tmp_rgb;
	tmp_depth.convertTo(depth_, CV_32FC1, 1.0f/(16.0f*100.0f));
	N_--;
	//lk.unlock();
}

void NetSource::_recvChunk(int frame, int chunk, bool delta, const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
	cv::Mat tmp_rgb, tmp_depth;

	if (!active_) return;

	//LOG(INFO) << "Received chunk " << (int)chunk;

	//try {
	// Decode in temporary buffers to prevent long locks
	cv::imdecode(jpg, cv::IMREAD_COLOR, &tmp_rgb);
	cv::imdecode(d, cv::IMREAD_UNCHANGED, &tmp_depth);

	// Apply colour correction to chunk
	ftl::rgbd::colourCorrection(tmp_rgb, gamma_, temperature_);

	// Build chunk head
	int cx = (chunk % chunks_dim_) * chunk_width_;
	int cy = (chunk / chunks_dim_) * chunk_height_;

	// Lock host to prevent grab
	UNIQUE_LOCK(host_->mutex(),lk);
	
	cv::Rect roi(cx,cy,chunk_width_,chunk_height_);
	cv::Mat chunkRGB = rgb_(roi);
	//cv::Mat ichunkDepth = idepth_(roi);
	cv::Mat chunkDepth = depth_(roi);

	tmp_rgb.copyTo(chunkRGB);
	//tmp_depth.convertTo(tmp_depth, CV_16UC1);
	//if (delta) ichunkDepth = tmp_depth - ichunkDepth;
	//tmp_depth.copyTo(ichunkDepth);
	tmp_depth.convertTo(chunkDepth, CV_32FC1, 1.0f/(16.0f*10.0f));
	if (chunk == 0) N_--;
	//lk.unlock();
	//} catch(...) {
	//	LOG(ERROR) << "Decode exception";
	//	return;
	//}
}

void NetSource::setPose(const Eigen::Matrix4d &pose) {
	if (!active_) return;

	vector<unsigned char> vec((unsigned char*)pose.data(), (unsigned char*)(pose.data()+(pose.size())));
	try {
		if (!host_->getNet()->send(peer_, "set_pose", *host_->get<string>("uri"), vec)) {
			active_ = false;
		}
	} catch (...) {

	}
	//Source::setPose(pose);
}

void NetSource::_updateURI() {
	UNIQUE_LOCK(mutex_,lk);
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

		host_->getNet()->bind(*uri, [this](int frame, int chunk, bool delta, const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
			_recvChunk(frame, chunk, delta, jpg, d);
		});

		N_ = 10;

		// Initiate stream with request for first 10 frames
		try {
			host_->getNet()->send(peer_, "get_stream", *uri, 10, 0, host_->getNet()->id(), *uri);
		} catch(...) {
			LOG(ERROR) << "Could not connect to stream " << *uri;
		}

		// Update chunk details
		chunks_dim_ = ftl::rgbd::kChunkDim;
		chunk_width_ = params_.width / chunks_dim_;
		chunk_height_ = params_.height / chunks_dim_;
		// TODO(Nick) Must lock before changing these below since some thread may still be updating chunk
		rgb_ = cv::Mat(cv::Size(params_.width, params_.height), CV_8UC3, cv::Scalar(0,0,0));
		depth_ = cv::Mat(cv::Size(params_.width, params_.height), CV_32FC1, 0.0f);
		//idepth_ = cv::Mat(cv::Size(params_.width, params_.height), CV_16UC1, cv::Scalar(0));

		uri_ = *uri;
		active_ = true;
	} else {
		uri_ = "";
		LOG(WARNING) << "NetSource URI is missing";
	}
}

bool NetSource::grab() {
	// Send one frame before end to prevent unwanted pause
	if (N_ <= 2) {
		N_ = 10;
		if (!host_->getNet()->send(peer_, "get_stream", *host_->get<string>("uri"), 10, 0, host_->getNet()->id(), *host_->get<string>("uri"))) {
			active_ = false;
		}
	}
	return true;
}

bool NetSource::isReady() {
	return has_calibration_ && !rgb_.empty();
}
