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

bool NetSource::_getCalibration(Universe &net, const UUID &peer, const string &src, ftl::rgbd::Camera &p, ftl::rgbd::channel_t chan) {
	try {
		while(true) {
			auto [cap,buf] = net.call<tuple<unsigned int,vector<unsigned char>>>(peer_, "source_details", src, chan);

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
				host_->getConfig()["doffs"] = p.doffs;
				
				return true;
			} else {
				LOG(INFO) << "Could not get calibration, retrying";
				sleep_for(milliseconds(500));
			}
		}
		
	} catch (const std::exception& ex) {
		LOG(ERROR) << "Exception: " << ex.what();
		return false;

	} catch (...) {
		LOG(ERROR) << "Unknown exception";
		return false;
	}
}

NetSource::NetSource(ftl::rgbd::Source *host)
		: ftl::rgbd::detail::Source(host), active_(false), minB_(9), maxN_(1), current_frame_(0) {

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

	host->on("doffs", [this,host](const ftl::config::Event&) {
		params_.doffs = host_->value("doffs", params_.doffs);
		host_->getNet()->send(peer_, "update_cfg", host_->getURI() + "/doffs", host_->getConfig()["doffs"].dump());
	});

	host->on("baseline", [this,host](const ftl::config::Event&) {
		params_.baseline = host_->value("baseline", 400.0);
		host_->getNet()->send(peer_, "update_cfg", host_->getURI() + "/baseline", host_->getConfig()["baseline"].dump());
	});

	host->on("doffs", [this,host](const ftl::config::Event&) {
		params_.doffs = host_->value("doffs", params_.doffs);
		host_->getNet()->send(peer_, "update_cfg", host_->getURI() + "/doffs", host_->getConfig()["doffs"].dump());
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

void NetSource::_recvChunk(int64_t frame, int chunk, bool delta, const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
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

	// Make certain last frame has finished decoding before swap
	while (frame > current_frame_ && chunk_count_ < 16 && chunk_count_ > 0) {
		std::this_thread::yield();
		//std::function<void(int)> j = ftl::pool.pop();
		//if ((bool)j) j(-1);
		//else std::this_thread::yield();
	}

	//{
		// A new frame has been started... finish the last one
		if (frame > current_frame_) {
			// Lock host to prevent grab
			UNIQUE_LOCK(host_->mutex(),lk);
			if (frame > current_frame_) {
				{
					// Lock to allow buffer swap
					UNIQUE_LOCK(mutex_,lk2);

					chunk_count_ = 0;

					// Swap the double buffers
					cv::Mat tmp;
					tmp = rgb_;
					rgb_ = d_rgb_;
					d_rgb_ = tmp;
					tmp = depth_;
					depth_ = d_depth_;
					d_depth_ = tmp;

					timestamp_ = current_frame_;
					current_frame_ = frame;
				}

				if (host_->callback()) {
					//ftl::pool.push([this](id) {
					//	UNIQUE_LOCK(host_->mutex(),lk);
						host_->callback()(timestamp_, rgb_, depth_);
					//});
				}
			}
		} else if (frame < current_frame_) {
			LOG(WARNING) << "Chunk dropped";
			if (chunk == 0) N_--;
			return;
		}
	//}

	// TODO:(Nick) Decode directly into double buffer if no scaling

	{
		SHARED_LOCK(mutex_, lk);
		
		cv::Rect roi(cx,cy,chunk_width_,chunk_height_);
		cv::Mat chunkRGB = d_rgb_(roi);
		cv::Mat chunkDepth = d_depth_(roi);

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
	}

	{
		
		++chunk_count_;
	}

	if (chunk == 0) {
		UNIQUE_LOCK(host_->mutex(),lk);
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

ftl::rgbd::Camera NetSource::parameters(ftl::rgbd::channel_t chan) {
	if (chan == ftl::rgbd::kChanRight) {
		auto uri = host_->get<string>("uri");
		if (!uri) return params_;

		ftl::rgbd::Camera params;
		_getCalibration(*host_->getNet(), peer_, *uri, params, chan);
		return params;
	} else {
		return params_;
	}
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

		has_calibration_ = _getCalibration(*host_->getNet(), peer_, *uri, params_, ftl::rgbd::kChanLeft);

		host_->getNet()->bind(*uri, [this](int64_t frame, int chunk, bool delta, const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
			_recvChunk(frame, chunk, delta, jpg, d);
		});

		N_ = 0;

		// Update chunk details
		chunks_dim_ = ftl::rgbd::kChunkDim;
		chunk_width_ = params_.width / chunks_dim_;
		chunk_height_ = params_.height / chunks_dim_;
		chunk_count_ = 0;
		rgb_ = cv::Mat(cv::Size(params_.width, params_.height), CV_8UC3, cv::Scalar(0,0,0));
		depth_ = cv::Mat(cv::Size(params_.width, params_.height), CV_32FC1, 0.0f);
		d_rgb_ = cv::Mat(cv::Size(params_.width, params_.height), CV_8UC3, cv::Scalar(0,0,0));
		d_depth_ = cv::Mat(cv::Size(params_.width, params_.height), CV_32FC1, 0.0f);

		uri_ = *uri;
		active_ = true;
	} else {
		uri_ = "";
		LOG(WARNING) << "NetSource URI is missing";
	}
}

bool NetSource::compute(int n, int b) {
	// Choose highest requested number of frames
	maxN_ = std::max(maxN_,(n == -1) ? ftl::rgbd::detail::kDefaultFrameCount : n);

	// Choose best requested quality
	minB_ = std::min(minB_,(b == -1) ? 0 : b);

	// Send k frames before end to prevent unwanted pause
	// Unless only a single frame is requested
	if ((N_ <= maxN_/2 && maxN_ > 1) || N_ == 0) {
		const ftl::rgbd::channel_t chan = host_->getChannel();

		N_ = maxN_;

		// Verify depth destination is of required type
		if (isFloatChannel(chan) && depth_.type() != CV_32F) {
			depth_ = cv::Mat(cv::Size(params_.width, params_.height), CV_32FC1, 0.0f);
		} else if (!isFloatChannel(chan) && depth_.type() != CV_8UC3) {
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
