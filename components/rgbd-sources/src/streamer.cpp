#include <ftl/rgbd/streamer.hpp>
#include <ftl/timer.hpp>
#include <vector>
#include <optional>
#include <thread>
#include <chrono>
#include <tuple>
#include <algorithm>

#include "bitrate_settings.hpp"

using ftl::rgbd::Streamer;
using ftl::rgbd::Source;
using ftl::rgbd::detail::StreamSource;
using ftl::rgbd::detail::StreamClient;
using ftl::rgbd::detail::bitrate_settings;
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


Streamer::Streamer(nlohmann::json &config, Universe *net)
		: ftl::Configurable(config), late_(false), jobs_(0) {

	active_ = false;
	net_ = net;
	time_peer_ = ftl::UUID(0);
	clock_adjust_ = 0;
	mspf_ = 1000 / value("fps", 20);
	//last_dropped_ = 0;
	//drop_count_ = 0;

	group_.setFPS(value("fps", 20));
	group_.setLatency(10);

	compress_level_ = value("compression", 1);
	
	net->bind("find_stream", [this](const std::string &uri) -> optional<UUID> {
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
	net->bind("source_details", [this](const std::string &uri, ftl::rgbd::channel_t chan) -> tuple<unsigned int,vector<unsigned char>> {
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

	net->bind("set_channel", [this](const string &uri, unsigned int chan) {
		SHARED_LOCK(mutex_,slk);

		LOG(INFO) << "SET CHANNEL " << chan;

		if (sources_.find(uri) != sources_.end()) {
			sources_[uri]->src->setChannel((ftl::rgbd::channel_t)chan);
		}
	});

	//net->bind("sync_streams", [this](unsigned long long time) {
		// Calc timestamp delta
	//});

	//net->bind("ping_streamer", [this](unsigned long long time) -> unsigned long long {
	//	return time;
	//});
}

Streamer::~Streamer() {
	net_->unbind("find_stream");
	net_->unbind("list_streams");
	net_->unbind("source_calibration");
	net_->unbind("get_stream");
	net_->unbind("sync_streams");
	net_->unbind("ping_streamer");
	//pool_.stop();
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
		sources_[src->getID()] = s;

		group_.addSource(src);
	}

	LOG(INFO) << "Streaming: " << src->getID();
	net_->broadcast("add_stream", src->getID());
}

void Streamer::_addClient(const string &source, int N, int rate, const ftl::UUID &peer, const string &dest) {
	StreamSource *s = nullptr;

	{
		UNIQUE_LOCK(mutex_,slk);
		if (sources_.find(source) == sources_.end()) return;

		if (rate < 0 || rate >= 10) return;
		if (N < 0 || N > ftl::rgbd::kMaxFrames) return;

		DLOG(INFO) << "Adding Stream Peer: " << peer.to_string() << " rate=" << rate << " N=" << N;

		s = sources_[source];

		// Set a time peer for clock sync
		if (time_peer_ == ftl::UUID(0)) {
			time_peer_ = peer;

			// Also do a time sync (but should be repeated periodically)
			auto start = std::chrono::high_resolution_clock::now();
			int64_t mastertime = net_->call<int64_t>(peer, "__ping__");
			auto elapsed = std::chrono::high_resolution_clock::now() - start;
			int64_t latency = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
			clock_adjust_ = mastertime - (std::chrono::time_point_cast<std::chrono::milliseconds>(start).time_since_epoch().count() + (latency/2));
			LOG(INFO) << "Clock adjustment: " << clock_adjust_;
			LOG(INFO) << "Latency: " << (latency / 2);
			LOG(INFO) << "Local: " << std::chrono::time_point_cast<std::chrono::milliseconds>(start).time_since_epoch().count() << ", master: " << mastertime;
			ftl::timer::setClockAdjustment(clock_adjust_);
		}
	}

	if (!s) return;  // No matching stream

	SHARED_LOCK(mutex_, slk);
	UNIQUE_LOCK(s->mutex, lk2);
	for (auto &client : s->clients[rate]) {
		// If already listening, just update chunk counters
		if (client.peerid == peer) {
			client.txmax = N * kChunkCount;
			client.txcount = 0;
			return;
		}
	}

	// Not an existing client so add one
	StreamClient &c = s->clients[rate].emplace_back();
	c.peerid = peer;
	c.uri = dest;
	c.txcount = 0;
	c.txmax = N * kChunkCount;
	++s->clientCount;
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
			_transmit(fs);
			return true;
		});
	} else {
		// Create thread job for frame ticking
		ftl::pool.push([this](int id) {
			group_.sync([this](FrameSet &fs) -> bool {
				_transmit(fs);
				return true;
			});
		});
	}
}

void Streamer::_cleanUp() {
	for (auto &s : sources_) {
		StreamSource *src = s.second;
		UNIQUE_LOCK(src->mutex,lk);

		for (unsigned int b=0; b<10; ++b) {
			auto i = src->clients[b].begin();
			while (i != src->clients[b].end()) {
				// Client request completed so remove from list
				if ((*i).txcount >= (*i).txmax) {
					LOG(INFO) << "Remove client: " << (*i).uri;
					i = src->clients[b].erase(i);
					--src->clientCount;
				} else {
					i++;
				}
			}
		}
	}
}

void Streamer::_transmit(ftl::rgbd::FrameSet &fs) {
	// Prevent new clients during processing.
	SHARED_LOCK(mutex_,slk);

	if (fs.sources.size() != sources_.size()) {
		LOG(ERROR) << "Incorrect number of sources in frameset: " << fs.sources.size() << " vs " << sources_.size();
		return;
	}

	int totalclients = 0;

	frame_no_ = fs.timestamp;

	for (int j=0; j<fs.sources.size(); ++j) {
		StreamSource *src = sources_[fs.sources[j]->getID()];

		// Don't do any work in the following cases
		if (!src) continue;
		if (!fs.sources[j]->isReady()) continue;
		if (src->clientCount == 0) continue;
		if (fs.channel1[j].empty() || (fs.sources[j]->getChannel() != ftl::rgbd::kChanNone && fs.channel2[j].empty())) continue;

		totalclients += src->clientCount;

		// Create jobs for each chunk
		for (int i=0; i<kChunkCount; ++i) {
			// Add chunk job to thread pool
			ftl::pool.push([this,&fs,j,i,src](int id) {
				int chunk = i;
				try {
					_encodeAndTransmit(src, fs.channel1[j], fs.channel2[j], chunk);
				} catch(...) {
					LOG(ERROR) << "Encode Exception: " << chunk;
				}

				//src->jobs--;
				std::unique_lock<std::mutex> lk(job_mtx_);
				--jobs_;
				if (jobs_ == 0) job_cv_.notify_one();
			});
		}

		jobs_ += kChunkCount;
	}

	std::unique_lock<std::mutex> lk(job_mtx_);
	job_cv_.wait_for(lk, std::chrono::seconds(20), [this]{ return jobs_ == 0; });
	if (jobs_ != 0) {
		LOG(FATAL) << "Deadlock detected";
	}

	// Go to sleep if no clients instead of spinning the cpu
	if (totalclients == 0 || sources_.size() == 0) sleep_for(milliseconds(200));
	else _cleanUp();
}

void Streamer::_encodeAndTransmit(StreamSource *src, const cv::Mat &rgb, const cv::Mat &depth, int chunk) {
	bool hasChan2 = (!depth.empty() && src->src->getChannel() != ftl::rgbd::kChanNone);

	bool delta = (chunk+src->frame) % 8 > 0;  // Do XOR or not
	int chunk_width = rgb.cols / kChunkDim;
	int chunk_height = rgb.rows / kChunkDim;

	// Build chunk heads
	int cx = (chunk % kChunkDim) * chunk_width;
	int cy = (chunk / kChunkDim) * chunk_height;
	cv::Rect roi(cx,cy,chunk_width,chunk_height);
	vector<unsigned char> rgb_buf;
	cv::Mat chunkRGB = rgb(roi);
	cv::Mat chunkDepth;
	//cv::Mat chunkDepthPrev = src->prev_depth(roi);

	cv::Mat d2, d3;
	vector<unsigned char> d_buf;

	if (hasChan2) {
		chunkDepth = depth(roi);
		if (chunkDepth.type() == CV_32F) chunkDepth.convertTo(d2, CV_16UC1, 1000); // 16*10);
		else d2 = chunkDepth;
		//if (delta) d3 = (d2 * 2) - chunkDepthPrev;
		//else d3 = d2;
		//d2.copyTo(chunkDepthPrev);
	}

	// For each allowed bitrate setting (0 = max quality)
	for (unsigned int b=0; b<10; ++b) {
		{
			//SHARED_LOCK(src->mutex,lk);
			if (src->clients[b].size() == 0) continue;
		}
		
		// Max bitrate means no changes
		if (b == 0) {
			_encodeChannel1(chunkRGB, rgb_buf, b);
			if (hasChan2) _encodeChannel2(d2, d_buf, src->src->getChannel(), b);

		// Otherwise must downscale and change compression params
		// TODO:(Nick) could reuse downscales
		} else {
			cv::Mat downrgb, downdepth;
			cv::resize(chunkRGB, downrgb, cv::Size(bitrate_settings[b].width / kChunkDim, bitrate_settings[b].height / kChunkDim));
			if (hasChan2) cv::resize(d2, downdepth, cv::Size(bitrate_settings[b].width / kChunkDim, bitrate_settings[b].height / kChunkDim));

			_encodeChannel1(downrgb, rgb_buf, b);
			if (hasChan2) _encodeChannel2(downdepth, d_buf, src->src->getChannel(), b);
		}

		//if (chunk == 0) LOG(INFO) << "Sending chunk " << chunk << " : size = " << (d_buf.size()+rgb_buf.size()) << "bytes";

		// Lock to prevent clients being added / removed
		SHARED_LOCK(src->mutex,lk);
		auto c = src->clients[b].begin();
		while (c != src->clients[b].end()) {
			try {
				// TODO:(Nick) Send pose
				if (!net_->send((*c).peerid, (*c).uri, frame_no_, chunk, delta, rgb_buf, d_buf)) {
					// Send failed so mark as client stream completed
					(*c).txcount = (*c).txmax;
				} else {
					++(*c).txcount;
					//LOG(INFO) << "SENT CHUNK : " << frame_no_*mspf_ << "-" << chunk;
				}
			} catch(...) {
				(*c).txcount = (*c).txmax;
			}
			++c;
		}
	}
}

void Streamer::_encodeChannel1(const cv::Mat &in, vector<unsigned char> &out, unsigned int b) {
	vector<int> jpgparams = {cv::IMWRITE_JPEG_QUALITY, bitrate_settings[b].jpg_quality};
	cv::imencode(".jpg", in, out, jpgparams);
}

bool Streamer::_encodeChannel2(const cv::Mat &in, vector<unsigned char> &out, ftl::rgbd::channel_t c, unsigned int b) {
	if (c == ftl::rgbd::kChanNone) return false;  // NOTE: Should not happen

	if (isFloatChannel(c) && in.type() == CV_16U && in.channels() == 1) {
		vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, bitrate_settings[b].png_compression};
		cv::imencode(".png", in, out, params);
		return true;
	} else if (!isFloatChannel(c) && in.type() == CV_8UC3) {
		vector<int> params = {cv::IMWRITE_JPEG_QUALITY, bitrate_settings[b].jpg_quality};
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
}
