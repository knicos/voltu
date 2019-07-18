#include <ftl/rgbd/streamer.hpp>
#include <vector>
#include <optional>
#include <thread>
#include <chrono>
#include <tuple>

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
	net->bind("source_details", [this](const std::string &uri) -> tuple<unsigned int,vector<unsigned char>> {
		vector<unsigned char> buf;
		SHARED_LOCK(mutex_,slk);

		if (sources_.find(uri) != sources_.end()) {
			buf.resize(sizeof(Camera));
			LOG(INFO) << "Calib buf size = " << buf.size();
			memcpy(buf.data(), &sources_[uri]->src->parameters(), buf.size());
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

void Streamer::stop() {
	active_ = false;
	wait();
}

void Streamer::poll() {
	//double wait = 1.0f / 25.0f;  // TODO:(Nick) Should be in config
	//auto start = std::chrono::high_resolution_clock::now();
	//int64_t now = std::chrono::time_point_cast<std::chrono::milliseconds>(start).time_since_epoch().count()+clock_adjust_;

	//int64_t msdelay = 40 - (now % 40);
	//while (msdelay >= 20) {
	//	sleep_for(milliseconds(10));
	//	now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count()+clock_adjust_;
	//	msdelay = 40 - (now % 40);
	//}
	//LOG(INFO) << "Required Delay: " << (now / 40) << " = " << msdelay;

	// Create frame jobs at correct FPS interval
	_schedule();
	//std::function<void(int)> j = ftl::pool.pop();
	//if (j) j(-1);

	//std::chrono::duration<double> elapsed =
	//	std::chrono::high_resolution_clock::now() - start;

	//if (elapsed.count() >= wait) {
		//LOG(WARNING) << "Frame rate below optimal @ " << (1.0f / elapsed.count());
	//} else {
		//LOG(INFO) << "Frame rate @ " << (1.0f / elapsed.count());
		// Otherwise, wait until next frame should start.
		// FIXME:(Nick) Is this accurate enough? Almost certainly not
		// TODO:(Nick) Synchronise by time corrections and use of fixed time points
		// but this only works if framerate can be achieved.
		//sleep_for(milliseconds((long long)((wait - elapsed.count()) * 1000.0f)));
	//}
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

		src->src->getFrames(src->rgb, src->depth);

		//if (!src->rgb.empty() && src->prev_depth.empty()) {
			//src->prev_depth = cv::Mat(src->rgb.size(), CV_16UC1, cv::Scalar(0));
			//LOG(INFO) << "Creating prevdepth: " << src->rgb.cols << "," << src->rgb.rows;
		//}
		src->jobs = 0;
		src->frame++;
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

		// There will be two jobs for this source...
		//UNIQUE_LOCK(job_mtx_,lk);
		jobs_ += 1 + kChunkCount;
		//lk.unlock();

		StreamSource *src = sources_[uri];
		if (src == nullptr || src->jobs != 0) continue;
		src->jobs = 1 + kChunkCount;

		// Grab job
		ftl::pool.push([this,src](int id) {
			//auto start = std::chrono::high_resolution_clock::now();

			auto start = std::chrono::high_resolution_clock::now();
			int64_t now = std::chrono::time_point_cast<std::chrono::milliseconds>(start).time_since_epoch().count()+clock_adjust_;
			int64_t target = now / 40;

			// TODO:(Nick) A now%40 == 0 should be accepted
			if (target != last_frame_) LOG(WARNING) << "Frame " << "(" << (target-last_frame_) << ") dropped by " << (now%40) << "ms";

			// Use sleep_for for larger delays
			int64_t msdelay = 40 - (now % 40);
			//LOG(INFO) << "Required Delay: " << (now / 40) << " = " << msdelay;
			while (msdelay >= 20) {
				sleep_for(milliseconds(10));
				now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count()+clock_adjust_;
				msdelay = 40 - (now % 40);
			}

			// Spin loop until exact grab time
			//LOG(INFO) << "Spin Delay: " << (now / 40) << " = " << (40 - (now%40));
			target = now / 40;
			while ((now/40) == target) {
				_mm_pause();  // SSE2 nano pause intrinsic
				now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count()+clock_adjust_;
			};
			last_frame_ = now/40;

			try {
				src->src->grab();
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
			job_cv_.notify_one();
		});

		// Create jobs for each chunk
		for (int i=0; i<kChunkCount; ++i) {
			// Add chunk job to thread pool
			ftl::pool.push([this,src](int id, int chunk) {
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
				job_cv_.notify_one();
			}, i);
		}
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
				// TODO:(Nick) Send pose and timestamp
				if (!net_->send((*c).peerid, (*c).uri, frame_no_, chunk, delta, rgb_buf, d_buf)) {
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

	if (c == ftl::rgbd::kChanDepth && in.type() == CV_16U && in.channels() == 1) {
		vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, bitrate_settings[b].png_compression};
		cv::imencode(".png", in, out, params);
		return true;
	} else if (c == ftl::rgbd::kChanRight && in.type() == CV_8UC3) {
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
