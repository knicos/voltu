#include "net.hpp"
#include <vector>
#include <thread>
#include <chrono>
#include <tuple>
#include <bitset>

#include "colour.hpp"

#include <ftl/rgbd/streamer.hpp>

using ftl::rgbd::detail::NetFrame;
using ftl::rgbd::detail::NetFrameQueue;
using ftl::rgbd::detail::NetSource;
using ftl::net::Universe;
using ftl::UUID;
using std::string;
using ftl::rgbd::Camera;
using std::vector;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::tuple;
using ftl::codecs::Channel;

// ===== NetFrameQueue =========================================================

NetFrameQueue::NetFrameQueue(int size) : frames_(size) {
	for (auto &f : frames_) f.timestamp = -1;
}

NetFrameQueue::~NetFrameQueue() {

}

NetFrame &NetFrameQueue::getFrame(int64_t ts, const cv::Size &s, int c1type, int c2type) {
	UNIQUE_LOCK(mtx_, lk);

	// Find matching timestamp
	for (auto &f : frames_) {
		if (f.timestamp == ts) return f;
	}

	int64_t oldest = ts;

	// No match so find an empty slot
	for (auto &f : frames_) {
		if (f.timestamp == -1) {
			f.timestamp = ts;
			f.chunk_count[0] = 0;
			f.chunk_count[1] = 0;
			f.chunk_total[0] = 0;
			f.chunk_total[1] = 0;
			f.channel_count = 0;
			f.tx_size = 0;
			//f.channel[0].create(s, c1type);
			//f.channel[1].create(s, c2type);
			return f;
		}
		oldest = (f.timestamp < oldest) ? f.timestamp : oldest;
	}

	// No empty slot, so give a fatal error
	for (auto &f : frames_) {
		LOG(ERROR) << "Stale frame: " << f.timestamp << " - " << f.chunk_count;

		// Force release of frame!
		if (f.timestamp == oldest) {
			f.timestamp = ts;
			f.chunk_count[0] = 0;
			f.chunk_count[1] = 0;
			f.chunk_total[0] = 0;
			f.chunk_total[1] = 0;
			f.channel_count = 0;
			f.tx_size = 0;
			//f.channel[0].create(s, c1type);
			//f.channel[1].create(s, c2type);
			return f;
		}
	}
	LOG(FATAL) << "Net Frame Queue not large enough: " << ts;
	// FIXME: (Nick) Could auto resize the queue.
	return frames_[0];  // To avoid missing return error...
}

void NetFrameQueue::freeFrame(NetFrame &f) {
	UNIQUE_LOCK(mtx_, lk);
	f.timestamp = -1;
}


// ===== NetSource =============================================================

NetSource::NetSource(ftl::rgbd::Source *host)
		: ftl::rgbd::detail::Source(host), active_(false), minB_(9), maxN_(1), adaptive_(0), queue_(3) {

	gamma_ = host->value("gamma", 1.0f);
	temperature_ = host->value("temperature", 6500);
	default_quality_ = host->value("quality", 0);
	last_bitrate_ = 0;
	params_right_.width = 0;
	has_calibration_ = false;

	decoder_[0] = nullptr;
	decoder_[1] = nullptr;

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

	host->on("centre_x", [this,host](const ftl::config::Event&) {
		params_.cx = host_->value("centre_x", 0.0);
		host_->getNet()->send(peer_, "update_cfg", host_->getURI() + "/centre_x", host_->getConfig()["centre_x"].dump());
	});

	host->on("centre_y", [this,host](const ftl::config::Event&) {
		params_.cy = host_->value("centre_y", 0.0);
		host_->getNet()->send(peer_, "update_cfg", host_->getURI() + "/centre_y", host_->getConfig()["centre_y"].dump());
	});

	host->on("doffs", [this,host](const ftl::config::Event&) {
		params_.doffs = host_->value("doffs", params_.doffs);
		host_->getNet()->send(peer_, "update_cfg", host_->getURI() + "/doffs", host_->getConfig()["doffs"].dump());
	});

	host->on("baseline", [this,host](const ftl::config::Event&) {
		params_.baseline = host_->value("baseline", 400.0);
		host_->getNet()->send(peer_, "update_cfg", host_->getURI() + "/baseline", host_->getConfig()["baseline"].dump());
	});

	// Right parameters

	host->on("focal_right", [this,host](const ftl::config::Event&) {
		params_right_.fx = host_->value("focal_right", 0.0);
		params_right_.fy = params_right_.fx;
		host_->getNet()->send(peer_, "update_cfg", host_->getURI() + "/focal_right", host_->getConfig()["focal_right"].dump());
	});

	host->on("centre_x_right", [this,host](const ftl::config::Event&) {
		params_right_.cx = host_->value("centre_x_right", 0.0);
		host_->getNet()->send(peer_, "update_cfg", host_->getURI() + "/centre_x_right", host_->getConfig()["centre_x_right"].dump());
	});

	host->on("centre_y_right", [this,host](const ftl::config::Event&) {
		params_right_.cy = host_->value("centre_y_right", 0.0);
		host_->getNet()->send(peer_, "update_cfg", host_->getURI() + "/centre_y_right", host_->getConfig()["centre_y_right"].dump());
	});


	host->on("quality", [this,host](const ftl::config::Event&) {
		default_quality_ = host->value("quality", 0);
	});

	abr_.setMaximumBitrate(host->value("max_bitrate", -1));
	abr_.setMinimumBitrate(host->value("min_bitrate", -1));

	_updateURI();

	h_ = host_->getNet()->onConnect([this](ftl::net::Peer *p) {
		if (active_) return;
		LOG(INFO) << "NetSource restart...";
		_updateURI();
	});
}

NetSource::~NetSource() {
	if (decoder_[0]) ftl::codecs::free(decoder_[0]);
	if (decoder_[1]) ftl::codecs::free(decoder_[1]);

	if (uri_.size() > 0) {
		host_->getNet()->unbind(uri_);
	}

	host_->getNet()->removeCallback(h_);
}

/*void NetSource::_checkAdaptive(int64_t ts) {
	const int64_t current = ftl::timer::get_time();
	int64_t net_latency = current - ts;

	// Only change bit rate gradually
	if (current - last_br_change_ > ftl::rgbd::detail::kAdaptationRate) {
		// Was this frame late?
		if (adaptive_ < ftl::rgbd::detail::kMaxBitrateLevels && net_latency > ftl::rgbd::detail::kLatencyThreshold) {
			slow_log_ = (slow_log_ << 1) + 1;
			std::bitset<32> bs(slow_log_);

			// Enough late frames to reduce bit rate
			if (bs.count() > ftl::rgbd::detail::kSlowFramesThreshold) {
				adaptive_++;
				slow_log_ = 0;
				last_br_change_ = current;
				LOG(WARNING) << "Adjust bitrate to " << adaptive_;
			}
		// No late frames in recent history...
		} else if (adaptive_ > 0 && slow_log_ == 0) {
			// TODO: (Nick) Don't change bitrate up so quickly as down?
			// Try a higher bitrate again?
			adaptive_--;
		}
	}
}*/

void NetSource::_createDecoder(int chan, const ftl::codecs::Packet &pkt) {
	UNIQUE_LOCK(mutex_,lk);
	auto *decoder = decoder_[chan];
	if (decoder) {
		if (!decoder->accepts(pkt)) {
			ftl::codecs::free(decoder_[chan]);
		} else {
			return;
		}
	}

	decoder_[chan] = ftl::codecs::allocateDecoder(pkt);
}

void NetSource::_processConfig(const ftl::codecs::Packet &pkt) {
	std::tuple<std::string, std::string> cfg;
	auto unpacked = msgpack::unpack((const char*)pkt.data.data(), pkt.data.size());
	unpacked.get().convert(cfg);

	//LOG(INFO) << "Config Received: " << std::get<1>(cfg);
	// TODO: This needs to be put in safer / better location
	host_->set(std::get<0>(cfg), nlohmann::json::parse(std::get<1>(cfg)));
}

void NetSource::_processCalibration(const ftl::codecs::Packet &pkt) {
	std::tuple<ftl::rgbd::Camera, ftl::codecs::Channel, ftl::rgbd::capability_t> params;
	auto unpacked = msgpack::unpack((const char*)pkt.data.data(), pkt.data.size());
	unpacked.get().convert(params);

	if (std::get<1>(params) == Channel::Left) {
		params_ = std::get<0>(params);
		capabilities_ = std::get<2>(params);
		has_calibration_ = true;
		LOG(INFO) << "Got Calibration channel: " << params_.width << "x" << params_.height;
	} else {
		params_right_ = std::get<0>(params);
	}
}

void NetSource::_processPose(const ftl::codecs::Packet &pkt) {
	LOG(INFO) << "Got POSE channel";
}

void NetSource::_recvPacket(short ttimeoff, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
	// Capture time here for better net latency estimate
	int64_t now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count();
	if (!active_) return;

	// Allow acccess to the raw data elsewhere...
	host_->notifyRaw(spkt, pkt);

	const ftl::codecs::Channel chan = host_->getChannel();
	const ftl::codecs::Channel rchan = spkt.channel;
	const int channum = (rchan == Channel::Colour) ? 0 : 1;

	// Deal with the special channels...
	switch (rchan) {
	case Channel::Configuration		: _processConfig(pkt); return;
	case Channel::Calibration		: _processCalibration(pkt); return;
	case Channel::Pose				: _processPose(pkt); return;
	}

	if (!has_calibration_) {
		LOG(WARNING) << "Missing calibration, skipping frame";
		return;
	}
	
	const cv::Size size = cv::Size(ftl::codecs::getWidth(pkt.definition), ftl::codecs::getHeight(pkt.definition));
	NetFrame &frame = queue_.getFrame(spkt.timestamp, size, CV_8UC3, (isFloatChannel(chan) ? CV_32FC1 : CV_8UC3));

	// Update frame statistics
	frame.tx_size += pkt.data.size();

	frame.channel[channum].create(size, (isFloatChannel(rchan) ? CV_32FC1 : CV_8UC3));

	// Only decode if this channel is wanted.
	if (rchan == Channel::Colour || rchan == chan) {
		_createDecoder(channum, pkt);
		auto *decoder = decoder_[channum];
		if (!decoder) {
			LOG(ERROR) << "No frame decoder available";
			return;
		}
	
		decoder->decode(pkt, frame.channel[channum]);
	} else if (chan != Channel::None && rchan != Channel::Colour) {
		// Didn't receive correct second channel so just clear the images
		if (isFloatChannel(chan)) {
			frame.channel[1].setTo(cv::Scalar(0.0f));
		} else {
			frame.channel[1].setTo(cv::Scalar(0,0,0));
		}
	}

	// Apply colour correction to chunk
	//ftl::rgbd::colourCorrection(tmp_rgb, gamma_, temperature_);

	// TODO:(Nick) Decode directly into double buffer if no scaling

	if (timestamp_ > 0 && frame.timestamp <= timestamp_) {
		LOG(ERROR) << "BAD DUPLICATE FRAME - " << frame.timestamp << " received=" << int(rchan) << " uri=" << uri_;
		return;
	}

	// Calculate how many packets to expect for this channel
	if (frame.chunk_total[channum] == 0) {
		frame.chunk_total[channum] = pkt.block_total;
	}		

	++frame.chunk_count[channum];
	if (frame.chunk_count[channum] == frame.chunk_total[channum]) ++frame.channel_count;
	if (frame.chunk_count[channum] > frame.chunk_total[channum]) LOG(FATAL) << "TOO MANY CHUNKS";

	// Capture tx time of first received chunk
	// FIXME: This seems broken
	if (channum == 1 && frame.chunk_count[channum] == 1) {
		UNIQUE_LOCK(frame.mtx, flk);
		if (frame.chunk_count[channum] == 1) {
			frame.tx_latency = int64_t(ttimeoff);
		}
	}

	// Last chunk of both channels now received, so we are done.
	if (frame.channel_count == spkt.channel_count) {
		_completeFrame(frame, now-(spkt.timestamp+frame.tx_latency));
	}
}

void NetSource::_completeFrame(NetFrame &frame, int64_t latency) {
	UNIQUE_LOCK(frame.mtx, flk);

	// Frame must not have already been freed.
	if (frame.timestamp >= 0) {
		timestamp_ = frame.timestamp;
		frame.tx_latency = latency;

		// Note: Not used currently
		adaptive_ = abr_.selectBitrate(frame);

		host_->notify(frame.timestamp, frame.channel[0], frame.channel[1]);

		queue_.freeFrame(frame);
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

ftl::rgbd::Camera NetSource::parameters(ftl::codecs::Channel chan) {
	if (chan == ftl::codecs::Channel::Right) {
		return params_right_;
	} else {
		return params_;
	}
}

void NetSource::_updateURI() {
	UNIQUE_LOCK(mutex_,lk);
	active_ = false;
	prev_chan_ = ftl::codecs::Channel::None;
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

		host_->getNet()->bind(*uri, [this](short ttimeoff, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			_recvPacket(ttimeoff, spkt, pkt);
		});

		N_ = 0;
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
	minB_ = std::min(minB_,(b == -1) ? int(adaptive_) : b);

	// Send k frames before end to prevent unwanted pause
	// Unless only a single frame is requested
	if ((N_ <= maxN_/2 && maxN_ > 1) || N_ == 0) {
		const ftl::codecs::Channel chan = host_->getChannel();

		N_ = maxN_;

		// Verify depth destination is of required type
		if (isFloatChannel(chan) && depth_.type() != CV_32F) {
			depth_.create(cv::Size(params_.width, params_.height), CV_32FC1);  // 0.0f
		} else if (!isFloatChannel(chan) && depth_.type() != CV_8UC3) {
			depth_.create(cv::Size(params_.width, params_.height), CV_8UC3);  // cv::Scalar(0,0,0)
		}

		if (prev_chan_ != chan) {
			host_->getNet()->send(peer_, "set_channel", *host_->get<string>("uri"), chan);
			prev_chan_ = chan;
		}

		if (!host_->getNet()->send(peer_, "get_stream",
				*host_->get<string>("uri"), maxN_, minB_,
				host_->getNet()->id(), *host_->get<string>("uri"))) {
			active_ = false;
		}

		abr_.notifyChanged();

		maxN_ = 1;  // Reset to single frame
		minB_ = 9;  // Reset to worst quality
	}
	return true;
}

bool NetSource::isReady() {
	return has_calibration_;
}
