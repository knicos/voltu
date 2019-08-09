#include <ftl/rgbd/streamer.hpp>
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
	mspf_ = 1000 / value("fps", 25);
	//last_dropped_ = 0;
	//drop_count_ = 0;

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

void Streamer::_decideFrameRate(int64_t framesdropped, int64_t msremainder) {
	actual_fps_ = 1000.0f / (float)((framesdropped+1)*mspf_+(msremainder));
	LOG(INFO) << "Actual FPS = " << actual_fps_;

	/*if (framesdropped > 0) {
		// If N consecutive frames are dropped, work out new rate
		if (last_dropped_/mspf_ >= last_frame_/mspf_ - 2*framesdropped) drop_count_++;
		else drop_count_ = 0;

		last_dropped_ = last_frame_+mspf_;

		if (drop_count_ >= ftl::rgbd::detail::kFrameDropLimit) {
			drop_count_ = 0;

			const int64_t actualmspf = std::min((int64_t)1000, framesdropped*mspf_+(mspf_ - msremainder));

			LOG(WARNING) << "Suggest FPS @ " << (1000 / actualmspf);
			//mspf_ = actualmspf;

			// Also notify all clients of change
		}
	} else {
		// Perhaps we can boost framerate?
		const int64_t actualmspf = std::min((int64_t)1000, framesdropped*mspf_+(mspf_ - msremainder));
		LOG(INFO) << "Boost framerate: " << (1000 / actualmspf);
		//mspf_ = actualmspf;
	}*/
}

void Streamer::stop() {
	active_ = false;
	wait();
}

void Streamer::poll() {
	// Create frame jobs at correct FPS interval
	_schedule();
}

void Streamer::run(bool block) {
	active_ = true;

	if (block) {
		while (ftl::running && active_) {
			poll();
		}
	} else {
		// Create thread job for frame ticking
		ftl::pool.push([this](int id) {
			while (ftl::running && active_) {
				poll();
			}
		});
	}
}

// Must be called in source locked state or src.state must be atomic
void Streamer::_swap(StreamSource *src) {
	if (src->jobs == 0) {
		UNIQUE_LOCK(src->mutex,lk);
		if (src->jobs == 0) {
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

			src->src->swap();
			src->src->getFrames(src->rgb, src->depth);

			//if (!src->rgb.empty() && src->prev_depth.empty()) {
				//src->prev_depth = cv::Mat(src->rgb.size(), CV_16UC1, cv::Scalar(0));
				//LOG(INFO) << "Creating prevdepth: " << src->rgb.cols << "," << src->rgb.rows;
			//}
			src->jobs = -1;
			src->frame++;
		}
	}
}

void Streamer::wait() {
	// Do some jobs in this thread, might as well...
	std::function<void(int)> j;
	while ((bool)(j=ftl::pool.pop())) {
		j(-1);
	}

	// Wait for all jobs to complete before finishing frame
	//UNIQUE_LOCK(job_mtx_, lk);
	std::unique_lock<std::mutex> lk(job_mtx_);
	job_cv_.wait_for(lk, std::chrono::seconds(20), [this]{ return jobs_ == 0; });
	if (jobs_ != 0) {
		LOG(FATAL) << "Deadlock detected";
	}

	// Capture frame number?
	frame_no_ = last_frame_;
}

void Streamer::_schedule(StreamSource *src) {
	if (src == nullptr || src->jobs > 0) return;

	jobs_ += 2 + kChunkCount;
	src->jobs = 2 + kChunkCount;

	// Grab / capture job
	ftl::pool.push([this,src](int id) {
		auto start = std::chrono::high_resolution_clock::now();
		int64_t now = std::chrono::time_point_cast<std::chrono::milliseconds>(start).time_since_epoch().count()+clock_adjust_;
		int64_t target = now / mspf_;
		int64_t msdelay = mspf_ - (now % mspf_);

		if (target != last_frame_ && msdelay != mspf_) LOG(WARNING) << "Frame " << "(" << (target-last_frame_) << ") dropped by " << (now%mspf_) << "ms";

		// Use sleep_for for larger delays
		
		//LOG(INFO) << "Required Delay: " << (now / 40) << " = " << msdelay;
		while (msdelay >= 20 && msdelay < mspf_) {
			sleep_for(milliseconds(10));
			now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count()+clock_adjust_;
			msdelay = mspf_ - (now % mspf_);
		}

		// Spin loop until exact grab time
		//LOG(INFO) << "Spin Delay: " << (now / 40) << " = " << (40 - (now%40));

		if (msdelay != mspf_) {
			target = now / mspf_;
			while ((now/mspf_) == target) {
				_mm_pause();  // SSE2 nano pause intrinsic
				now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count()+clock_adjust_;
			};
		}
		last_frame_ = now/mspf_;

		try {
			src->src->capture();
		} catch (std::exception &ex) {
			LOG(ERROR) << "Exception when grabbing frame";
			LOG(ERROR) << ex.what();
		}
		catch (...) {
			LOG(ERROR) << "Unknown exception when grabbing frame";
		}

		//now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count()+clock_adjust_;
		//if (now%40 > 0) LOG(INFO) << "Grab in: " << (now%40) << "ms";

		//std::chrono::duration<double> elapsed =
		//	std::chrono::high_resolution_clock::now() - start;
		//LOG(INFO) << "Grab in " << elapsed.count() << "s";

		src->jobs--;
		_swap(src);

		// Mark job as finished
		std::unique_lock<std::mutex> lk(job_mtx_);
		--jobs_;
		if (jobs_ == 0) job_cv_.notify_one();
	});

	// Compute job
	ftl::pool.push([this,src](int id) {
		try {
			src->src->compute();
		} catch (std::exception &ex) {
			LOG(ERROR) << "Exception when computing frame";
			LOG(ERROR) << ex.what();
		}
		catch (...) {
			LOG(ERROR) << "Unknown exception when computing frame";
		}

		src->jobs--;
		_swap(src);

		// Mark job as finished
		std::unique_lock<std::mutex> lk(job_mtx_);
		--jobs_;
		if (jobs_ == 0) job_cv_.notify_one();
	});

	// Create jobs for each chunk
	for (int i=0; i<kChunkCount; ++i) {
		// Add chunk job to thread pool
		ftl::pool.push([this,src,i](int id) {
			int chunk = i;
			try {
			if (!src->rgb.empty() && (src->src->getChannel() == ftl::rgbd::kChanNone || !src->depth.empty())) {
				_encodeAndTransmit(src, chunk);
			}
			} catch(...) {
				LOG(ERROR) << "Encode Exception: " << chunk;
			}

			src->jobs--;
			_swap(src);
			std::unique_lock<std::mutex> lk(job_mtx_);
			--jobs_;
			if (jobs_ == 0) job_cv_.notify_one();
		});
	}
}

void Streamer::_schedule() {
	wait();
	//std::mutex job_mtx;
	//std::condition_variable job_cv;
	//int jobs = 0;

	//auto now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count()+clock_adjust_;
	//LOG(INFO) << "Frame time = " << (now-(last_frame_*40)) << "ms";

	// Prevent new clients during processing.
	SHARED_LOCK(mutex_,slk);

	for (auto s : sources_) {
		string uri = s.first;

		// No point in doing work if no clients
		if (s.second->clientCount == 0) {
			continue;
		}

		_schedule(s.second);
	}
}

void Streamer::_encodeAndTransmit(StreamSource *src, int chunk) {
	bool hasChan2 = (!src->depth.empty() && src->src->getChannel() != ftl::rgbd::kChanNone);

	bool delta = (chunk+src->frame) % 8 > 0;  // Do XOR or not
	int chunk_width = src->rgb.cols / kChunkDim;
	int chunk_height = src->rgb.rows / kChunkDim;

	// Build chunk heads
	int cx = (chunk % kChunkDim) * chunk_width;
	int cy = (chunk / kChunkDim) * chunk_height;
	cv::Rect roi(cx,cy,chunk_width,chunk_height);
	vector<unsigned char> rgb_buf;
	cv::Mat chunkRGB = src->rgb(roi);
	cv::Mat chunkDepth;
	//cv::Mat chunkDepthPrev = src->prev_depth(roi);

	cv::Mat d2, d3;
	vector<unsigned char> d_buf;

	if (hasChan2) {
		chunkDepth = src->depth(roi);
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
				if (!net_->send((*c).peerid, (*c).uri, frame_no_*mspf_, chunk, delta, rgb_buf, d_buf)) {
					// Send failed so mark as client stream completed
					(*c).txcount = (*c).txmax;
				} else {
					++(*c).txcount;
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
