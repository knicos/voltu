#include "net.hpp"
#include <vector>
#include <thread>
#include <chrono>
#include <tuple>

#include "colour.hpp"

#include <ftl/rgbd/streamer.hpp>

using ftl::rgbd::detail::NetSource;
using ftl::net::Universe;
using ftl::UUID;
using std::string;
using ftl::rgbd::Camera;
using std::vector;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::tuple;

bool NetSource::_getCalibration(Universe &net, const UUID &peer, const string &src, ftl::rgbd::Camera &p) {
	try {
		while(true) {
			auto [cap,buf] = net.call<tuple<unsigned int,vector<unsigned char>>>(peer_, "source_details", src);

			capabilities_ = cap;

			if (buf.size() > 0) {
				memcpy((char*)&p, buf.data(), buf.size());
				
				if (sizeof(p) != buf.size()) {
					LOG(ERROR) << "Corrupted calibration";
					return false;
				}

				LOG(INFO) << "Calibration received: " << p.cx << ", " << p.cy << ", " << p.fx << ", " << p.fy;

				// Put calibration into config manually
				host_->getConfig()["focal"] = p.fx;
				host_->getConfig()["centre_x"] = p.cx;
				host_->getConfig()["centre_y"] = p.cy;
				host_->getConfig()["baseline"] = p.baseline;
				
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
		: ftl::rgbd::detail::Source(host), active_(false), minB_(9), maxN_(1) {

	gamma_ = host->value("gamma", 1.0f);
	temperature_ = host->value("temperature", 6500);
	default_quality_ = host->value("quality", 0);

	host->on("gamma", [this,host](const ftl::config::Event&) {
		gamma_ = host->value("gamma", 1.0f);
	});

	host->on("temperature", [this,host](const ftl::config::Event&) {
		temperature_ = host->value("temperature", 6500);
	});

	host->on("focal", [this,host](const ftl::config::Event&) {
		params_.fx = host_->value("focal", 400.0);
		params_.fy = params_.fx;
		host_->getNet()->send(peer_, "update_cfg", host_->getURI() + "/focal", host_->getConfig()["focal"].dump());
	});

	host->on("baseline", [this,host](const ftl::config::Event&) {
		params_.baseline = host_->value("baseline", 400.0);
		host_->getNet()->send(peer_, "update_cfg", host_->getURI() + "/baseline", host_->getConfig()["baseline"].dump());
	});

	host->on("quality", [this,host](const ftl::config::Event&) {
		default_quality_ = host->value("quality", 0);
	});

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

void NetSource::_recvChunk(int frame, int chunk, bool delta, const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
	cv::Mat tmp_rgb, tmp_depth;

	if (!active_) return;

	// Decode in temporary buffers to prevent long locks
	cv::imdecode(jpg, cv::IMREAD_COLOR, &tmp_rgb);
	if (d.size() > 0) cv::imdecode(d, cv::IMREAD_UNCHANGED, &tmp_depth);

	// Apply colour correction to chunk
	ftl::rgbd::colourCorrection(tmp_rgb, gamma_, temperature_);

	// Build chunk head
	int cx = (chunk % chunks_dim_) * chunk_width_;
	int cy = (chunk / chunks_dim_) * chunk_height_;

	// Lock host to prevent grab
	UNIQUE_LOCK(host_->mutex(),lk);
	
	cv::Rect roi(cx,cy,chunk_width_,chunk_height_);
	cv::Mat chunkRGB = rgb_(roi);
	cv::Mat chunkDepth = depth_(roi);

	// Original size so just copy
	if (tmp_rgb.cols == chunkRGB.cols) {
		tmp_rgb.copyTo(chunkRGB);
		if (!tmp_depth.empty() && tmp_depth.type() == CV_16U && chunkDepth.type() == CV_32F) {
			tmp_depth.convertTo(chunkDepth, CV_32FC1, 1.0f/1000.0f); //(16.0f*10.0f));
		} else if (!tmp_depth.empty() && tmp_depth.type() == CV_8UC3 && chunkDepth.type() == CV_8UC3) {
			tmp_depth.copyTo(chunkDepth);
		} else {
			// Silent ignore?
		}
	// Downsized so needs a scale up
	} else {
		cv::resize(tmp_rgb, chunkRGB, chunkRGB.size());
		tmp_depth.convertTo(tmp_depth, CV_32FC1, 1.0f/1000.0f);
		if (!tmp_depth.empty() && tmp_depth.type() == CV_16U && chunkDepth.type() == CV_32F) {
			tmp_depth.convertTo(tmp_depth, CV_32FC1, 1.0f/1000.0f); //(16.0f*10.0f));
			cv::resize(tmp_depth, chunkDepth, chunkDepth.size());
		} else if (!tmp_depth.empty() && tmp_depth.type() == CV_8UC3 && chunkDepth.type() == CV_8UC3) {
			cv::resize(tmp_depth, chunkDepth, chunkDepth.size());
		} else {
			// Silent ignore?
		}
	}

	if (chunk == 0) {
		N_--;
	}
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
	prev_chan_ = ftl::rgbd::kChanNone;
	auto uri = host_->get<string>("uri");

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

		N_ = 0;

		// Initiate stream with request for first 10 frames
		//try {
		//	host_->getNet()->send(peer_, "get_stream", *uri, N_, 0, host_->getNet()->id(), *uri);
		//} catch(...) {
		//	LOG(ERROR) << "Could not connect to stream " << *uri;
		//}

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

bool NetSource::grab(int n, int b) {
	// Choose highest requested number of frames
	maxN_ = std::max(maxN_,(n == -1) ? 10 : n);

	// Choose best requested quality
	minB_ = std::min(minB_,(b == -1) ? 0 : b);

	// Send k frames before end to prevent unwanted pause
	// Unless only a single frame is requested
	if ((N_ <= 2 && maxN_ > 1) || N_ == 0) {
		const ftl::rgbd::channel_t chan = host_->getChannel();

		N_ = maxN_;

		// Verify depth destination is of required type
		if (chan == ftl::rgbd::kChanDepth && depth_.type() != CV_32F) {
			depth_ = cv::Mat(cv::Size(params_.width, params_.height), CV_32FC1, 0.0f);
		} else if (chan == ftl::rgbd::kChanRight && depth_.type() != CV_8UC3) {
			depth_ = cv::Mat(cv::Size(params_.width, params_.height), CV_8UC3, cv::Scalar(0,0,0));
		}

		if (prev_chan_ != chan) {
			host_->getNet()->send(peer_, "set_channel", *host_->get<string>("uri"), chan);
			prev_chan_ = chan;
		}

		if (!host_->getNet()->send(peer_, "get_stream",
				*host_->get<string>("uri"), N_, minB_,
				host_->getNet()->id(), *host_->get<string>("uri"))) {
			active_ = false;
		}

		maxN_ = 1;  // Reset to single frame
		minB_ = 9;  // Reset to worst quality
	}
	return true;
}

bool NetSource::isReady() {
	return has_calibration_ && !rgb_.empty();
}
