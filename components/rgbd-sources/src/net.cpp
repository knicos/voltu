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
using ftl::rgbd::Channel;

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
			f.chunk_count = 0;
			f.chunk_total = 0;
			f.tx_size = 0;
			f.channel1.create(s, c1type);
			f.channel2.create(s, c2type);
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
			f.chunk_count = 0;
			f.chunk_total = 0;
			f.tx_size = 0;
			f.channel1.create(s, c1type);
			f.channel2.create(s, c2type);
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

bool NetSource::_getCalibration(Universe &net, const UUID &peer, const string &src, ftl::rgbd::Camera &p, ftl::rgbd::Channel chan) {
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
		: ftl::rgbd::detail::Source(host), active_(false), minB_(9), maxN_(1), adaptive_(0), queue_(3) {

	gamma_ = host->value("gamma", 1.0f);
	temperature_ = host->value("temperature", 6500);
	default_quality_ = host->value("quality", 0);
	last_bitrate_ = 0;

	decoder_c1_ = nullptr;
	decoder_c2_ = nullptr;

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
	if (decoder_c1_) ftl::codecs::free(decoder_c1_);
	if (decoder_c2_) ftl::codecs::free(decoder_c2_);

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
	auto *decoder = (chan == 0) ? decoder_c1_ : decoder_c2_;
	if (decoder) {
		if (!decoder->accepts(pkt)) {
			ftl::codecs::free((chan == 0) ? decoder_c1_ : decoder_c2_);
		} else {
			return;
		}
	}

	if (chan == 0) {
		decoder_c1_ = ftl::codecs::allocateDecoder(pkt);
	} else {
		decoder_c2_ = ftl::codecs::allocateDecoder(pkt);
	}
}

void NetSource::_recvPacket(short ttimeoff, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
	// Capture time here for better net latency estimate
	int64_t now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count();
	if (!active_) return;

	// Allow acccess to the raw data elsewhere...
	host_->notifyRaw(spkt, pkt);

	const ftl::rgbd::Channel chan = host_->getChannel();
	int rchan = spkt.channel; // & 0x1;

	NetFrame &frame = queue_.getFrame(spkt.timestamp, cv::Size(params_.width, params_.height), CV_8UC3, (isFloatChannel(chan) ? CV_32FC1 : CV_8UC3));

	// Update frame statistics
	frame.tx_size += pkt.data.size();

	// Ignore any unwanted second channel
	if (!(chan == ftl::rgbd::Channel::None && rchan > 0)) {
		_createDecoder(rchan, pkt);
		auto *decoder = (rchan == 0) ? decoder_c1_ : decoder_c2_;
		if (!decoder) {
			LOG(ERROR) << "No frame decoder available";
			return;
		}

		decoder->decode(pkt, (rchan == 0) ? frame.channel1 : frame.channel2);
	} else {
		//LOG(INFO) << "Unwanted frame";
	}

	// Apply colour correction to chunk
	//ftl::rgbd::colourCorrection(tmp_rgb, gamma_, temperature_);

	// TODO:(Nick) Decode directly into double buffer if no scaling

	if (timestamp_ > 0 && frame.timestamp <= timestamp_) {
		LOG(ERROR) << "BAD DUPLICATE FRAME - " << frame.timestamp << " received=" << int(rchan) << " uri=" << uri_;
		return;
	}

	// Calculate how many packets to expect for this frame
	if (frame.chunk_total == 0) {
		// Getting a second channel first means expect double packets
		// FIXME: Assumes each packet has same number of blocks!
		frame.chunk_total = pkt.block_total * spkt.channel_count;
	}		

	++frame.chunk_count;

	if (frame.chunk_count > frame.chunk_total) LOG(FATAL) << "TOO MANY CHUNKS";

	// Capture tx time of first received chunk
	if (frame.chunk_count == 1) {
		UNIQUE_LOCK(frame.mtx, flk);
		if (frame.chunk_count == 1) {
			frame.tx_latency = int64_t(ttimeoff);
		}
	}

	// Last chunk now received
	if (frame.chunk_count == frame.chunk_total) {
		UNIQUE_LOCK(frame.mtx, flk);

		if (frame.timestamp >= 0 && frame.chunk_count == frame.chunk_total) {
			timestamp_ = frame.timestamp;
			frame.tx_latency = now-(spkt.timestamp+frame.tx_latency);

			adaptive_ = abr_.selectBitrate(frame);
			//LOG(INFO) << "Frame finished: " << frame.timestamp;
			auto cb = host_->callback();
			if (cb) {
				try {
					cb(frame.timestamp, frame.channel1, frame.channel2);
				} catch (...) {
					LOG(ERROR) << "Exception in net frame callback";
				}
			} else {
				LOG(ERROR) << "NO FRAME CALLBACK";
			}

			queue_.freeFrame(frame);

			{
				// Decrement expected frame counter
				N_--;
			}
		}
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

ftl::rgbd::Camera NetSource::parameters(ftl::rgbd::Channel chan) {
	if (chan == ftl::rgbd::Channel::Right) {
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
	prev_chan_ = ftl::rgbd::Channel::None;
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

		has_calibration_ = _getCalibration(*host_->getNet(), peer_, *uri, params_, ftl::rgbd::Channel::Left);

		host_->getNet()->bind(*uri, [this](short ttimeoff, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			//if (chunk == -1) {
				//#ifdef HAVE_NVPIPE
				//_recvVideo(frame, ttimeoff, bitrate, jpg, d);
				//#else
				//LOG(ERROR) << "Cannot receive HEVC, no NvPipe support";
				//#endif
			//} else {
				//_recvChunk(frame, ttimeoff, bitrate, chunk, jpg, d);
				_recvPacket(ttimeoff, spkt, pkt);
			//}
		});

		N_ = 0;

		rgb_ = cv::Mat(cv::Size(params_.width, params_.height), CV_8UC3, cv::Scalar(0,0,0));
		depth_ = cv::Mat(cv::Size(params_.width, params_.height), CV_32FC1, 0.0f);
		//d_rgb_ = cv::Mat(cv::Size(params_.width, params_.height), CV_8UC3, cv::Scalar(0,0,0));
		//d_depth_ = cv::Mat(cv::Size(params_.width, params_.height), CV_32FC1, 0.0f);

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
		const ftl::rgbd::Channel chan = host_->getChannel();

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
	return has_calibration_ && !rgb_.empty();
}
