#include <ftl/rgbd/streamer.hpp>
#include <ftl/timer.hpp>
#include <vector>
#include <optional>
#include <thread>
#include <chrono>
#include <tuple>
#include <algorithm>

#include <ftl/rgbd/detail/abr.hpp>
#include <ftl/codecs/encoder.hpp>

using ftl::rgbd::Streamer;
using ftl::rgbd::Source;
using ftl::rgbd::detail::StreamSource;
using ftl::rgbd::detail::StreamClient;
using ftl::rgbd::detail::ABRController;
using ftl::codecs::definition_t;
using ftl::codecs::device_t;
using ftl::codecs::Channel;
using ftl::net::Universe;
using std::string;
using std::list;
using std::map;
using std::optional;
using std::vector;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::tuple;
using std::make_tuple;

static const ftl::codecs::preset_t kQualityThreshold = ftl::codecs::kPresetLQThreshold;


Streamer::Streamer(nlohmann::json &config, Universe *net)
		: ftl::Configurable(config), late_(false) {

	active_ = false;
	net_ = net;
	time_peer_ = ftl::UUID(0);
	clock_adjust_ = 0;
	mspf_ = ftl::timer::getInterval(); //1000 / value("fps", 20);
	//last_dropped_ = 0;
	//drop_count_ = 0;

	encode_mode_ = ftl::rgbd::kEncodeVideo;
	hq_devices_ = (value("disable_hardware_encode", false)) ? device_t::Software : device_t::Any;
	hq_codec_ = value("video_codec", ftl::codecs::codec_t::Any);

	//group_.setFPS(value("fps", 20));
	group_.setLatency(4);
	group_.setName("StreamGroup");

	compress_level_ = value("compression", 1);
	
	net->bind("find_stream", [this](const std::string &uri) -> optional<ftl::UUID> {
		SHARED_LOCK(mutex_,slk);

		if (sources_.find(uri) != sources_.end()) {
			LOG(INFO) << "Valid source request received: " << uri;
			return net_->id();
		} else return {};
	});

	net->bind("list_streams", [this]() -> vector<string> {
		vector<string> streams;
		for (auto &i : sources_) {
			streams.push_back(i.first);
		}
		return streams;
	});

	net->bind("set_pose", [this](const std::string &uri, const std::vector<unsigned char> &buf) {
		SHARED_LOCK(mutex_,slk);

		if (sources_.find(uri) != sources_.end()) {
			Eigen::Matrix4d pose;
			memcpy(pose.data(), buf.data(), buf.size());
			sources_[uri]->src->setPose(pose);
		}
	});

	net->bind("get_pose", [this](const std::string &uri) -> std::vector<unsigned char> {
		SHARED_LOCK(mutex_,slk);

		if (sources_.find(uri) != sources_.end()) {
			Eigen::Matrix4d pose = sources_[uri]->src->getPose();
			vector<unsigned char> vec((unsigned char*)pose.data(), (unsigned char*)(pose.data()+(pose.size())));
			return vec;
		} else {
			LOG(WARNING) << "Requested pose not found: " << uri;
			return {};
		}
	});

	// Allow remote users to access camera calibration matrix
	net->bind("source_details", [this](const std::string &uri, ftl::codecs::Channel chan) -> tuple<unsigned int,vector<unsigned char>> {
		vector<unsigned char> buf;
		SHARED_LOCK(mutex_,slk);

		if (sources_.find(uri) != sources_.end()) {
			buf.resize(sizeof(Camera));
			LOG(INFO) << "Calib buf size = " << buf.size();
			auto params = sources_[uri]->src->parameters(chan);
			memcpy(buf.data(), &params, buf.size());
			return make_tuple(sources_[uri]->src->getCapabilities(), buf);
		} else {
			return make_tuple(0u,buf);
		}
	});

	net->bind("get_stream", [this](const string &source, int N, int rate, const UUID &peer, const string &dest) {
		_addClient(source, N, rate, peer, dest);
	});

	net->bind("set_channel", [this](const string &uri, Channel chan) {
		SHARED_LOCK(mutex_,slk);

		if (sources_.find(uri) != sources_.end()) {
			sources_[uri]->src->setChannel(chan);
		}
	});

	//net->bind("sync_streams", [this](unsigned long long time) {
		// Calc timestamp delta
	//});

	//net->bind("ping_streamer", [this](unsigned long long time) -> unsigned long long {
	//	return time;
	//});

	on("hq_bitrate", [this](const ftl::config::Event &e) {
		UNIQUE_LOCK(mutex_,ulk);
		for (auto &s : sources_) {
			s.second->hq_bitrate = value("hq_bitrate", ftl::codecs::kPresetBest);
		}
	});

	on("lq_bitrate", [this](const ftl::config::Event &e) {
		UNIQUE_LOCK(mutex_,ulk);
		for (auto &s : sources_) {
			s.second->lq_bitrate = value("lq_bitrate", ftl::codecs::kPresetWorst);
		}
	});
}

Streamer::~Streamer() {
	timer_job_.cancel();
	net_->unbind("find_stream");
	net_->unbind("list_streams");
	net_->unbind("source_calibration");
	net_->unbind("get_stream");
	net_->unbind("sync_streams");
	net_->unbind("ping_streamer");
	//pool_.stop();

	{
		UNIQUE_LOCK(mutex_,ulk);
		for (auto &s : sources_) {
			StreamSource *src = s.second;
			src->clientCount = 0;
		}
	}
	_cleanUp();
	{
		UNIQUE_LOCK(mutex_,ulk);
		sources_.clear();
	}
}

void Streamer::add(Source *src) {
	{
		UNIQUE_LOCK(mutex_,ulk);
		if (sources_.find(src->getID()) != sources_.end()) return;

		StreamSource *s = new StreamSource;
		s->src = src;
		//s->prev_depth = cv::Mat(cv::Size(src->parameters().width, src->parameters().height), CV_16SC1, 0);
		s->jobs = 0;
		s->frame = 0;
		s->clientCount = 0;
		s->hq_count = 0;
		s->lq_count = 0;

		s->hq_bitrate = value("hq_bitrate", ftl::codecs::kPresetBest);
		s->lq_bitrate = value("lq_bitrate", ftl::codecs::kPresetWorst);

		sources_[src->getID()] = s;

		group_.addSource(src);

		src->addRawCallback([this,s](Source *src, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
			if (spkt.channel == Channel::Calibration) {
				// Calibration changed, so lets re-check the bitrate presets
				const auto &params = src->parameters();
				s->hq_bitrate = ftl::codecs::findPreset(params.width, params.height);
			}

			//LOG(INFO) << "RAW CALLBACK";
			_transmitPacket(s, spkt, pkt, Quality::Any);
		});
	}

	LOG(INFO) << "Streaming: " << src->getID();
	net_->broadcast("add_stream", src->getID());
}

void Streamer::add(ftl::rgbd::Group *grp) {
	auto srcs = grp->sources();
	for (int i=0; i<srcs.size(); ++i) {
		auto &src = srcs[i];
		{
			UNIQUE_LOCK(mutex_,ulk);
			if (sources_.find(src->getID()) != sources_.end()) return;

			StreamSource *s = new StreamSource;
			s->src = src;
			//s->prev_depth = cv::Mat(cv::Size(src->parameters().width, src->parameters().height), CV_16SC1, 0);
			s->jobs = 0;
			s->frame = 0;
			s->clientCount = 0;
			s->hq_count = 0;
			s->lq_count = 0;
			s->id = i;
			sources_[src->getID()] = s;

			//group_.addSource(src);

			src->addRawCallback([this,s](Source *src, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
				//LOG(INFO) << "RAW CALLBACK";
				_transmitPacket(s, spkt, pkt, Quality::Any);
			});
		}

		LOG(INFO) << "Proxy Streaming: " << src->getID();
		net_->broadcast("add_stream", src->getID());
	}

	LOG(INFO) << "All proxy streams added";
}

void Streamer::_addClient(const string &source, int N, int rate, const ftl::UUID &peer, const string &dest) {
	StreamSource *s = nullptr;

	{
		UNIQUE_LOCK(mutex_,slk);
		if (sources_.find(source) == sources_.end()) return;

		if (rate < 0 || rate >= 10) return;
		if (N < 0 || N > ftl::rgbd::kMaxFrames) return;

		//DLOG(INFO) << "Adding Stream Peer: " << peer.to_string() << " rate=" << rate << " N=" << N;

		s = sources_[source];

		// Set a time peer for clock sync
		if (time_peer_ == ftl::UUID(0)) {
			time_peer_ = peer;

			// Do a time sync whenever the CPU is idle for 10ms or more.
			// FIXME: Could be starved
			timer_job_ = ftl::timer::add(ftl::timer::kTimerIdle10, [peer,this](int id) {
				auto start = std::chrono::high_resolution_clock::now();
				int64_t mastertime;

				try {
					mastertime = net_->call<int64_t>(peer, "__ping__");
				} catch (...) {
					// Reset time peer and remove timer
					time_peer_ = ftl::UUID(0);
					return false;
				}

				auto elapsed = std::chrono::high_resolution_clock::now() - start;
				int64_t latency = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
				auto clock_adjust = mastertime - (ftl::timer::get_time() + (latency/2));

				if (clock_adjust > 0) {
					LOG(INFO) << "Clock adjustment: " << clock_adjust;
					//LOG(INFO) << "Latency: " << (latency / 2);
					//LOG(INFO) << "Local: " << std::chrono::time_point_cast<std::chrono::milliseconds>(start).time_since_epoch().count() << ", master: " << mastertime;
					ftl::timer::setClockAdjustment(clock_adjust);
				}

				return true;
			});
		}
	}

	if (!s) return;  // No matching stream

	SHARED_LOCK(mutex_, slk);
	UNIQUE_LOCK(s->mutex, lk2);
	for (auto &client : s->clients) {
		// If already listening, just update chunk counters
		if (client.peerid == peer) {
			client.txmax = N;
			client.txcount = 0;

			// Possible switch from high quality to low quality encoding or vice versa
			if (client.preset < kQualityThreshold && rate >= kQualityThreshold) {
				s->hq_count--;
				s->lq_count++;
				if (s->lq_encoder_c1) s->lq_encoder_c1->reset();
				if (s->lq_encoder_c2) s->lq_encoder_c2->reset();
			} else if (client.preset >= kQualityThreshold && rate < kQualityThreshold) {
				s->hq_count++;
				s->lq_count--;
				if (s->hq_encoder_c1) s->hq_encoder_c1->reset();
				if (s->hq_encoder_c2) s->hq_encoder_c2->reset();
			}

			client.preset = rate;
			return;
		}
	}

	// Not an existing client so add one
	StreamClient &c = s->clients.emplace_back();
	c.peerid = peer;
	c.uri = dest;
	c.txcount = 0;
	c.txmax = N;
	c.preset = rate;

	if (rate >= kQualityThreshold) {
		if (s->lq_encoder_c1) s->lq_encoder_c1->reset();
		if (s->lq_encoder_c2) s->lq_encoder_c2->reset();
		s->lq_count++;
	} else {
		if (s->hq_encoder_c1) s->hq_encoder_c1->reset();
		if (s->hq_encoder_c2) s->hq_encoder_c2->reset();
		s->hq_count++;
	}

	++s->clientCount;

	// Finally, inject calibration and config data
	s->src->inject(Channel::Calibration, s->src->parameters(Channel::Left), Channel::Left, s->src->getCapabilities());
	s->src->inject(Channel::Calibration, s->src->parameters(Channel::Right), Channel::Right, s->src->getCapabilities());
	//s->src->inject(s->src->getPose());
	//if (!(*s->src->get<nlohmann::json>("meta")).is_null()) {
		s->src->inject(Channel::Configuration, "/original", s->src->getConfig().dump());
	//}
}

void Streamer::remove(Source *) {

}

void Streamer::remove(const std::string &) {

}

void Streamer::stop() {
	group_.stop();
}

void Streamer::run(bool block) {
	if (block) {
		group_.sync([this](FrameSet &fs) -> bool {
			_process(fs);
			return true;
		});
	} else {
		// Create thread job for frame ticking
		ftl::pool.push([this](int id) {
			group_.sync([this](FrameSet &fs) -> bool {
				_process(fs);
				return true;
			});
		});
	}
}

void Streamer::_cleanUp() {
	for (auto &s : sources_) {
		StreamSource *src = s.second;
		UNIQUE_LOCK(src->mutex,lk);

		auto i = src->clients.begin();
		while (i != src->clients.end()) {
			// Client request completed so remove from list
			if ((*i).txcount >= (*i).txmax) {
				// If peer was clock sync master, the remove that...
				if ((*i).peerid == time_peer_) {
					timer_job_.cancel();
					time_peer_ = ftl::UUID(0);
				}
				LOG(INFO) << "Remove client: " << (*i).uri;

				if ((*i).preset < kQualityThreshold) {
					src->hq_count--;
				} else {
					src->lq_count--;
				}

				i = src->clients.erase(i);
				--src->clientCount;
			} else {
				i++;
			}
		}

		if (src->hq_count == 0) {
			if (src->hq_encoder_c1) ftl::codecs::free(src->hq_encoder_c1);
			if (src->hq_encoder_c2) ftl::codecs::free(src->hq_encoder_c2);
		}

		if (src->lq_count == 0) {
			if (src->lq_encoder_c1) ftl::codecs::free(src->lq_encoder_c1);
			if (src->lq_encoder_c2) ftl::codecs::free(src->lq_encoder_c2);
		}

		if (src->clientCount == 0) {

		}
	}
}

void Streamer::_process(ftl::rgbd::FrameSet &fs) {
	// Prevent new clients during processing.
	SHARED_LOCK(mutex_,slk);

	// This check is not valid, always assume fs.sources is correct
	//if (fs.sources.size() != sources_.size()) {
	//	LOG(ERROR) << "Incorrect number of sources in frameset: " << fs.sources.size() << " vs " << sources_.size();
		//return;
	//}

	int totalclients = 0;

	frame_no_ = fs.timestamp;

	for (int j=0; j<fs.sources.size(); ++j) {
		StreamSource *src = sources_[fs.sources[j]->getID()];
		SHARED_LOCK(src->mutex,lk);

		// Don't do any work in the following cases
		if (!src) continue;
		if (!fs.sources[j]->isReady()) continue;
		if (src->clientCount == 0) continue;
		//if (fs.channel1[j].empty() || (fs.sources[j]->getChannel() != ftl::rgbd::kChanNone && fs.channel2[j].empty())) continue;
		if (!fs.frames[j].hasChannel(Channel::Colour) || !fs.frames[j].hasChannel(fs.sources[j]->getChannel())) continue;

		bool hasChan2 = fs.sources[j]->getChannel() != Channel::None;

		totalclients += src->clientCount;

		// Do we need to do high quality encoding?
		if (src->hq_count > 0) {
			if (!src->hq_encoder_c1) src->hq_encoder_c1 = ftl::codecs::allocateEncoder(
					definition_t::HD1080, hq_devices_, hq_codec_);
			if (!src->hq_encoder_c2 && hasChan2) src->hq_encoder_c2 = ftl::codecs::allocateEncoder(
					definition_t::HD1080, hq_devices_, hq_codec_);

			// Do we have the resources to do a HQ encoding?
			if (src->hq_encoder_c1 && (!hasChan2 || src->hq_encoder_c2)) {
				auto *enc1 = src->hq_encoder_c1;
				auto *enc2 = src->hq_encoder_c2;

				// Important to send channel 2 first if needed...
				// Receiver only waits for channel 1 by default
				// TODO: Each encode could be done in own thread
				if (hasChan2) {
					// TODO: Stagger the reset between nodes... random phasing
					if (fs.timestamp % (10*ftl::timer::getInterval()) == 0) enc2->reset();

					auto chan = fs.sources[j]->getChannel();

					enc2->encode(fs.frames[j].get<cv::cuda::GpuMat>(chan), src->hq_bitrate, [this,src,hasChan2,chan](const ftl::codecs::Packet &blk){
						_transmitPacket(src, blk, chan, hasChan2, Quality::High);
					});
				} else {
					if (enc2) enc2->reset();
				}

				// TODO: Stagger the reset between nodes... random phasing
				if (fs.timestamp % (10*ftl::timer::getInterval()) == 0) enc1->reset();
				enc1->encode(fs.frames[j].get<cv::cuda::GpuMat>(Channel::Colour), src->hq_bitrate, [this,src,hasChan2](const ftl::codecs::Packet &blk){
					_transmitPacket(src, blk, Channel::Colour, hasChan2, Quality::High);
				});
			}
		}

		// Do we need to do low quality encoding?
		if (src->lq_count > 0) {
			if (!src->lq_encoder_c1) src->lq_encoder_c1 = ftl::codecs::allocateEncoder(
					definition_t::SD480, device_t::Software);
			if (!src->lq_encoder_c2 && hasChan2) src->lq_encoder_c2 = ftl::codecs::allocateEncoder(
					definition_t::SD480, device_t::Software);

			// Do we have the resources to do a LQ encoding?
			if (src->lq_encoder_c1 && (!hasChan2 || src->lq_encoder_c2)) {
				auto *enc1 = src->lq_encoder_c1;
				auto *enc2 = src->lq_encoder_c2;

				// Important to send channel 2 first if needed...
				// Receiver only waits for channel 1 by default
				if (hasChan2) {
					auto chan = fs.sources[j]->getChannel();

					enc2->encode(fs.frames[j].get<cv::cuda::GpuMat>(chan), src->lq_bitrate, [this,src,hasChan2,chan](const ftl::codecs::Packet &blk){
						_transmitPacket(src, blk, chan, hasChan2, Quality::Low);
					});
				} else {
					if (enc2) enc2->reset();
				}

				enc1->encode(fs.frames[j].get<cv::cuda::GpuMat>(Channel::Colour), src->lq_bitrate, [this,src,hasChan2](const ftl::codecs::Packet &blk){
					_transmitPacket(src, blk, Channel::Colour, hasChan2, Quality::Low);
				});
			}
		}

		// Do we need to do low quality encoding?
		/*if (src->lq_count > 0) {
			if (!src->lq_encoder_c1) src->lq_encoder_c1 = ftl::codecs::allocateLQEncoder();
			if (!src->lq_encoder_c2) src->lq_encoder_c2 = ftl::codecs::allocateLQEncoder();

			// Do we have the resources to do a LQ encoding?
			if (src->lq_encoder_c1 && src->lq_encoder_c2) {
				const auto *enc1 = src->lq_encoder_c1;
				const auto *enc2 = src->lq_encoder_c2;

				// Do entire frame as single step
				if (!enc1->useBlocks() || !enc2->useBlocks()) {
					ftl::pool.push([this,&fs,j,src](int id) {
						_encodeLQAndTransmit(src, fs.channel1[j], fs.channel2[j], -1);
						std::unique_lock<std::mutex> lk(job_mtx_);
						--jobs_;
						if (jobs_ == 0) job_cv_.notify_one();
					});

					jobs_++;
				// Or divide frame into blocks and encode each
				} else {
					// Create jobs for each chunk
					for (int i=0; i<chunk_count_; ++i) {
						// Add chunk job to thread pool
						ftl::pool.push([this,&fs,j,i,src](int id) {
							int chunk = i;
							try {
								_encodeLQAndTransmit(src, fs.channel1[j], fs.channel2[j], chunk);
							} catch(...) {
								LOG(ERROR) << "Encode Exception: " << chunk;
							}

							//src->jobs--;
							std::unique_lock<std::mutex> lk(job_mtx_);
							--jobs_;
							if (jobs_ == 0) job_cv_.notify_one();
						});
					}

					jobs_ += chunk_count_;
				}
			}
		}*/
	}

	/*std::unique_lock<std::mutex> lk(job_mtx_);
	job_cv_.wait_for(lk, std::chrono::seconds(20), [this]{ return jobs_ == 0; });
	if (jobs_ != 0) {
		LOG(FATAL) << "Deadlock detected";
	}*/

	// Go to sleep if no clients instead of spinning the cpu
	if (totalclients == 0 || sources_.size() == 0) {
		// Make sure to unlock so clients can connect!
		//lk.unlock();
		slk.unlock();
		sleep_for(milliseconds(50));
	} else _cleanUp();
}

void Streamer::_transmitPacket(StreamSource *src, const ftl::codecs::Packet &pkt, Channel chan, bool hasChan2, Quality q) {
	ftl::codecs::StreamPacket spkt = {
		frame_no_,
		src->id,
		(hasChan2) ? 2 : 1,
		chan
		//static_cast<uint8_t>((chan & 0x1) | ((hasChan2) ? 0x2 : 0x0))
	};

	_transmitPacket(src, spkt, pkt, q);
}

void Streamer::_transmitPacket(StreamSource *src, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt, Quality q) {
	// Lock to prevent clients being added / removed
	//SHARED_LOCK(src->mutex,lk);
	auto c = src->clients.begin();
	while (c != src->clients.end()) {
		const ftl::codecs::preset_t b = (*c).preset;
		if ((q == Quality::High && b >= kQualityThreshold) || (q == Quality::Low && b < kQualityThreshold)) {
			++c;
			LOG(INFO) << "INCORRECT QUALITY";
			continue;
		}

		try {
			// TODO:(Nick) Send pose
			short pre_transmit_latency = short(ftl::timer::get_time() - spkt.timestamp);
			if (!net_->send((*c).peerid,
					(*c).uri,
					pre_transmit_latency,  // Time since timestamp for tx
					spkt,
					pkt)) {

				// Send failed so mark as client stream completed
				(*c).txcount = (*c).txmax;
			} else {
				// Count frame as completed only if last block and channel is 0
				if (pkt.block_number == pkt.block_total - 1 && spkt.channel == Channel::Colour) ++(*c).txcount;
			}
		} catch(...) {
			(*c).txcount = (*c).txmax;
		}
		++c;
	}
}

/*void Streamer::_encodeHQAndTransmit(StreamSource *src, const cv::Mat &c1, const cv::Mat &c2, int block) {
	bool hasChan2 = (!c2.empty() && src->src->getChannel() != ftl::rgbd::kChanNone);

	LOG(INFO) << "Encode HQ: " << block;

	vector<unsigned char> c1buff;
	vector<unsigned char> c2buff;

	if (block == -1) {
		src->hq_encoder_c1->encode(c1, c1buff, src->hq_bitrate, false);
		if (hasChan2) src->hq_encoder_c2->encode(c2, c2buff, src->hq_bitrate, false);
	} else {
		//bool delta = (chunk+src->frame) % 8 > 0;  // Do XOR or not
		int chunk_width = c1.cols / chunk_dim_;
		int chunk_height = c1.rows / chunk_dim_;

		// Build chunk heads
		int cx = (block % chunk_dim_) * chunk_width;
		int cy = (block / chunk_dim_) * chunk_height;
		cv::Rect roi(cx,cy,chunk_width,chunk_height);
		//vector<unsigned char> rgb_buf;
		cv::Mat chunkRGB = c1(roi);
		src->hq_encoder_c1->encode(chunkRGB, c1buff, src->hq_bitrate, false);

		if (hasChan2) {
			cv::Mat chunkDepth = c2(roi);
			src->hq_encoder_c2->encode(chunkDepth, c2buff, src->hq_bitrate, false);
		}
	}

	// Lock to prevent clients being added / removed
	SHARED_LOCK(src->mutex,lk);
	auto c = src->clients.begin();
	while (c != src->clients.end()) {
		const int b = (*c).bitrate;
		if (b >= kQualityThreshold) continue; // Not a HQ request

		try {
			// TODO:(Nick) Send pose
			short pre_transmit_latency = short(ftl::timer::get_time() - frame_no_);
			if (!net_->send((*c).peerid, (*c).uri, frame_no_, pre_transmit_latency, uint8_t(src->hq_bitrate), block, c1buff, c2buff)) {
				// Send failed so mark as client stream completed
				(*c).txcount = (*c).txmax;
			} else {
				++(*c).txcount;
				//LOG(INFO) << "SENT CHUNK : " << frame_no_ << "-" << chunk;
			}
		} catch(...) {
			(*c).txcount = (*c).txmax;
		}
		++c;
	}
}

void Streamer::_encodeLQAndTransmit(StreamSource *src, const cv::Mat &c1, const cv::Mat &c2, int block) {
	bool hasChan2 = (!c2.empty() && src->src->getChannel() != ftl::rgbd::kChanNone);

	LOG(INFO) << "Encode LQ: " << block;

	vector<unsigned char> c1buff;
	vector<unsigned char> c2buff;

	if (block == -1) {
		src->lq_encoder_c1->encode(c1, c1buff, src->lq_bitrate, false);
		if (hasChan2) src->lq_encoder_c2->encode(c2, c2buff, src->lq_bitrate, false);
	} else {
		//bool delta = (chunk+src->frame) % 8 > 0;  // Do XOR or not
		int chunk_width = c1.cols / chunk_dim_;
		int chunk_height = c1.rows / chunk_dim_;

		// Build chunk heads
		int cx = (block % chunk_dim_) * chunk_width;
		int cy = (block / chunk_dim_) * chunk_height;
		cv::Rect roi(cx,cy,chunk_width,chunk_height);
		//vector<unsigned char> rgb_buf;
		cv::Mat chunkRGB = c1(roi);
		//cv::resize(chunkRGB, downrgb, cv::Size(ABRController::getColourWidth(b) / chunk_dim_, ABRController::getColourHeight(b) / chunk_dim_));

		src->lq_encoder_c1->encode(chunkRGB, c1buff, src->lq_bitrate, false);

		if (hasChan2) {
			cv::Mat chunkDepth = c2(roi);
			//cv::resize(chunkDepth, tmp, cv::Size(ABRController::getDepthWidth(b) / chunk_dim_, ABRController::getDepthHeight(b) / chunk_dim_), 0, 0, cv::INTER_NEAREST);
			src->lq_encoder_c2->encode(chunkDepth, c2buff, src->lq_bitrate, false);
		}
	}

	// Lock to prevent clients being added / removed
	SHARED_LOCK(src->mutex,lk);
	auto c = src->clients.begin();
	while (c != src->clients.end()) {
		const int b = (*c).bitrate;
		if (b < kQualityThreshold) continue; // Not an LQ request

		try {
			// TODO:(Nick) Send pose
			short pre_transmit_latency = short(ftl::timer::get_time() - frame_no_);
			if (!net_->send((*c).peerid, (*c).uri, frame_no_, pre_transmit_latency, uint8_t(src->hq_bitrate), block, c1buff, c2buff)) {
				// Send failed so mark as client stream completed
				(*c).txcount = (*c).txmax;
			} else {
				++(*c).txcount;
				//LOG(INFO) << "SENT CHUNK : " << frame_no_ << "-" << chunk;
			}
		} catch(...) {
			(*c).txcount = (*c).txmax;
		}
		++c;
	}
}*/

/*void Streamer::_encodeImagesAndTransmit(StreamSource *src, const cv::Mat &rgb, const cv::Mat &depth, int chunk) {
	bool hasChan2 = (!depth.empty() && src->src->getChannel() != ftl::rgbd::kChanNone);

	//bool delta = (chunk+src->frame) % 8 > 0;  // Do XOR or not
	int chunk_width = rgb.cols / chunk_dim_;
	int chunk_height = rgb.rows / chunk_dim_;

	// Build chunk heads
	int cx = (chunk % chunk_dim_) * chunk_width;
	int cy = (chunk / chunk_dim_) * chunk_height;
	cv::Rect roi(cx,cy,chunk_width,chunk_height);
	//vector<unsigned char> rgb_buf;
	cv::Mat chunkRGB = rgb(roi);
	cv::Mat chunkDepth;
	//cv::Mat chunkDepthPrev = src->prev_depth(roi);

	cv::Mat d2, d3;
	//vector<unsigned char> d_buf;

	if (hasChan2) {
		chunkDepth = depth(roi);
		if (chunkDepth.type() == CV_32F) chunkDepth.convertTo(d2, CV_16UC1, 1000); // 16*10);
		else d2 = chunkDepth;
		//if (delta) d3 = (d2 * 2) - chunkDepthPrev;
		//else d3 = d2;
		//d2.copyTo(chunkDepthPrev);
	}

	// TODO: Verify these don't allocate memory if not needed.
	// TODO: Reuse these buffers to reduce allocations.
	vector<unsigned char> brgb[ftl::rgbd::detail::kMaxBitrateLevels];
	vector<unsigned char> bdepth[ftl::rgbd::detail::kMaxBitrateLevels];

	// Lock to prevent clients being added / removed
	SHARED_LOCK(src->mutex,lk);
	auto c = src->clients.begin();
	while (c != src->clients.end()) {
		const int b = (*c).bitrate;

		if (brgb[b].empty()) {
			// Max bitrate means no changes
			if (b == 0) {
				_encodeImageChannel1(chunkRGB, brgb[b], b);
				if (hasChan2) _encodeImageChannel2(d2, bdepth[b], src->src->getChannel(), b);

			// Otherwise must downscale and change compression params
			} else {
				cv::Mat downrgb, downdepth;
				cv::resize(chunkRGB, downrgb, cv::Size(ABRController::getColourWidth(b) / chunk_dim_, ABRController::getColourHeight(b) / chunk_dim_));
				if (hasChan2) cv::resize(d2, downdepth, cv::Size(ABRController::getDepthWidth(b) / chunk_dim_, ABRController::getDepthHeight(b) / chunk_dim_), 0, 0, cv::INTER_NEAREST);

				_encodeImageChannel1(downrgb, brgb[b], b);
				if (hasChan2) _encodeImageChannel2(downdepth, bdepth[b], src->src->getChannel(), b);
			}
		}

		try {
			// TODO:(Nick) Send pose
			short pre_transmit_latency = short(ftl::timer::get_time() - frame_no_);
			if (!net_->send((*c).peerid, (*c).uri, frame_no_, pre_transmit_latency, uint8_t(b), chunk, brgb[b], bdepth[b])) {
				// Send failed so mark as client stream completed
				(*c).txcount = (*c).txmax;
			} else {
				++(*c).txcount;
				//LOG(INFO) << "SENT CHUNK : " << frame_no_ << "-" << chunk;
			}
		} catch(...) {
			(*c).txcount = (*c).txmax;
		}
		++c;
	}
}

void Streamer::_encodeImageChannel1(const cv::Mat &in, vector<unsigned char> &out, unsigned int b) {
	vector<int> jpgparams = {cv::IMWRITE_JPEG_QUALITY, ABRController::getColourQuality(b)};
	cv::imencode(".jpg", in, out, jpgparams);
}

bool Streamer::_encodeImageChannel2(const cv::Mat &in, vector<unsigned char> &out, ftl::codecs::Channel_t c, unsigned int b) {
	if (c == ftl::rgbd::kChanNone) return false;  // NOTE: Should not happen

	if (isFloatChannel(c) && in.type() == CV_16U && in.channels() == 1) {
		vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, ABRController::getDepthQuality(b)};
		if (!cv::imencode(".png", in, out, params)) {
			LOG(ERROR) << "PNG Encoding error";
			return false;
		}
		return true;
	} else if (!isFloatChannel(c) && in.type() == CV_8UC3) {
		vector<int> params = {cv::IMWRITE_JPEG_QUALITY, ABRController::getColourQuality(b)};
		cv::imencode(".jpg", in, out, params);
		return true;
	} else {
		LOG(ERROR) << "Bad channel configuration: channel=" << c << " imagetype=" << in.type(); 
	}

	return false;
}

Source *Streamer::get(const std::string &uri) {
	SHARED_LOCK(mutex_,slk);
	if (sources_.find(uri) != sources_.end()) return sources_[uri]->src;
	else return nullptr;
}*/
