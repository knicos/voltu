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
using std::mutex;
using std::shared_mutex;
using std::unique_lock;
using std::shared_lock;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::tuple;
using std::make_tuple;


Streamer::Streamer(nlohmann::json &config, Universe *net)
		: ftl::Configurable(config), late_(false), jobs_(0) {

	active_ = false;
	net_ = net;

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

	net->bind("sync_streams", [this](unsigned long long time) {
		// Calc timestamp delta
	});

	net->bind("ping_streamer", [this](unsigned long long time) -> unsigned long long {
		return time;
	});
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
	StreamSource *s = nullptr;

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

		LOG(INFO) << "Adding Stream Peer: " << peer.to_string() << " rate=" << rate << " N=" << N;

		s = sources_[source];
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
	double wait = 1.0f / 25.0f;  // TODO(Nick) Should be in config
	auto start = std::chrono::high_resolution_clock::now();
	// Create frame jobs at correct FPS interval
	_schedule();

	std::chrono::duration<double> elapsed =
		std::chrono::high_resolution_clock::now() - start;

	if (elapsed.count() >= wait) {
		LOG(WARNING) << "Frame rate below optimal @ " << (1.0f / elapsed.count());
	} else {
		//LOG(INFO) << "Frame rate @ " << (1.0f / elapsed.count());
		// Otherwise, wait until next frame should start.
		// CHECK(Nick) Is this accurate enough? Almost certainly not
		// TODO(Nick) Synchronise by time corrections and use of fixed time points
		// but this only works if framerate can be achieved.
		sleep_for(milliseconds((long long)((wait - elapsed.count()) * 1000.0f)));
	}
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
	UNIQUE_LOCK(job_mtx_, lk);
	job_cv_.wait(lk, [this]{ return jobs_ == 0; });
}

void Streamer::_schedule() {
	wait();
	//std::mutex job_mtx;
	//std::condition_variable job_cv;
	//int jobs = 0;

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
		jobs_ += 1 + kChunkDim*kChunkDim;
		//lk.unlock();

		StreamSource *src = sources_[uri];
		if (src == nullptr || src->jobs != 0) continue;
		src->jobs = 1 + kChunkDim*kChunkDim;

		// Grab job
		ftl::pool.push([this,src](int id) {
			//auto start = std::chrono::high_resolution_clock::now();
			try {
				src->src->grab();
			} catch (std::exception &ex) {
				LOG(ERROR) << "Exception when grabbing frame";
				LOG(ERROR) << ex.what();
			}
			catch (...) {
				LOG(ERROR) << "Unknown exception when grabbing frame";
			}

			//std::chrono::duration<double> elapsed =
			//	std::chrono::high_resolution_clock::now() - start;
			//LOG(INFO) << "Grab in " << elapsed.count() << "s";

			src->jobs--;
			_swap(src);

			// Mark job as finished
			--jobs_;
			job_cv_.notify_one();
		});

		// Create jobs for each chunk
		for (int i=0; i<kChunkCount; ++i) {
			// Add chunk job to thread pool
			ftl::pool.push([this,src](int id, int chunk) {
				if (!src->rgb.empty() && !src->depth.empty()) {
					bool delta = (chunk+src->frame) % 8 > 0;  // Do XOR or not
					int chunk_width = src->rgb.cols / kChunkDim;
					int chunk_height = src->rgb.rows / kChunkDim;

					// Build chunk heads
					int cx = (chunk % kChunkDim) * chunk_width;
					int cy = (chunk / kChunkDim) * chunk_height;
					cv::Rect roi(cx,cy,chunk_width,chunk_height);
					vector<unsigned char> rgb_buf;
					cv::Mat chunkRGB = src->rgb(roi);
					cv::Mat chunkDepth = src->depth(roi);
					//cv::Mat chunkDepthPrev = src->prev_depth(roi);

					cv::Mat d2, d3;
					vector<unsigned char> d_buf;
					chunkDepth.convertTo(d2, CV_16UC1, 1000); // 16*10);
					//if (delta) d3 = (d2 * 2) - chunkDepthPrev;
					//else d3 = d2;
					//d2.copyTo(chunkDepthPrev);

					// For each allowed bitrate setting (0 = max quality)
					for (unsigned int b=0; b<10; ++b) {
						{
							//SHARED_LOCK(src->mutex,lk);
							if (src->clients[b].size() == 0) continue;
						}
						
						// Max bitrate means no changes
						if (b == 0) {
							cv::imencode(".jpg", chunkRGB, rgb_buf);
							vector<int> pngparams = {cv::IMWRITE_PNG_COMPRESSION, compress_level_}; // Default is 1 for fast, 9 = small but slow.
							cv::imencode(".png", d2, d_buf, pngparams);

						// Otherwise must downscale and change compression params
						// TODO(Nick) could reuse downscales
						} else {
							cv::Mat downrgb, downdepth;
							cv::resize(chunkRGB, downrgb, cv::Size(bitrate_settings[b].width / kChunkDim, bitrate_settings[b].height / kChunkDim));
							cv::resize(d2, downdepth, cv::Size(bitrate_settings[b].width / kChunkDim, bitrate_settings[b].height / kChunkDim));
							vector<int> jpgparams = {cv::IMWRITE_JPEG_QUALITY, bitrate_settings[b].jpg_quality};
							cv::imencode(".jpg", downrgb, rgb_buf, jpgparams);
							vector<int> pngparams = {cv::IMWRITE_PNG_COMPRESSION, bitrate_settings[b].png_compression}; // Default is 1 for fast, 9 = small but slow.
							cv::imencode(".png", downdepth, d_buf, pngparams);
						}

						//if (chunk == 0) LOG(INFO) << "Sending chunk " << chunk << " : size = " << (d_buf.size()+rgb_buf.size()) << "bytes";

						// Lock to prevent clients being added / removed
						SHARED_LOCK(src->mutex,lk);
						auto c = src->clients[b].begin();
						while (c != src->clients[b].end()) {
							try {
								// TODO(Nick) Send pose and timestamp
								if (!net_->send((*c).peerid, (*c).uri, 0, chunk, delta, rgb_buf, d_buf)) {
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

				src->jobs--;
				_swap(src);
				--jobs_;
				job_cv_.notify_one();
			}, i);
		}
	}
}

Source *Streamer::get(const std::string &uri) {
	SHARED_LOCK(mutex_,slk);
	if (sources_.find(uri) != sources_.end()) return sources_[uri]->src;
	else return nullptr;
}
