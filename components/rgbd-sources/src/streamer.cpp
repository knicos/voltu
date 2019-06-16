#include <ftl/rgbd/streamer.hpp>
#include <vector>
#include <optional>
#include <thread>
#include <chrono>

using ftl::rgbd::Streamer;
using ftl::rgbd::Source;
using ftl::rgbd::detail::StreamSource;
using ftl::rgbd::detail::StreamClient;
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


Streamer::Streamer(nlohmann::json &config, Universe *net)
		: ftl::Configurable(config), late_(false), jobs_(0) {

	active_ = false;
	net_ = net;
	
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
			Eigen::Matrix4f pose;
			memcpy(pose.data(), buf.data(), buf.size());
			sources_[uri]->src->setPose(pose);
		}
	});

	// Allow remote users to access camera calibration matrix
	net->bind("source_calibration", [this](const std::string &uri) -> vector<unsigned char> {
		vector<unsigned char> buf;
		SHARED_LOCK(mutex_,slk);

		if (sources_.find(uri) != sources_.end()) {
			buf.resize(sizeof(Camera));
			LOG(INFO) << "Calib buf size = " << buf.size();
			memcpy(buf.data(), &sources_[uri]->src->parameters(), buf.size());
		}
		return buf;
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
		s->state = 0;
		sources_[src->getID()] = s;
	}

	LOG(INFO) << "Streaming: " << src->getID();
	net_->broadcast("add_stream", src->getID());
}

void Streamer::_addClient(const string &source, int N, int rate, const ftl::UUID &peer, const string &dest) {
	StreamSource *s = nullptr;

	//{
		UNIQUE_LOCK(mutex_,slk);
		if (sources_.find(source) == sources_.end()) return;

		if (rate < 0 || rate >= 10) return;
		if (N < 0 || N > ftl::rgbd::kMaxFrames) return;

		DLOG(INFO) << "Adding Stream Peer: " << peer.to_string();

		s = sources_[source];
	//}

	if (!s) return;

	UNIQUE_LOCK(s->mutex, lk2);
	for (int i=0; i<s->clients[rate].size(); i++) {
		if (s->clients[rate][i].peerid == peer) {
			StreamClient &c = s->clients[rate][i];
			c.txmax = N;
			c.txcount = 0;
			return;
		}
	}

	StreamClient c;
	c.peerid = peer;
	c.uri = dest;
	c.txcount = 0;
	c.txmax = N;

	s->clients[rate].push_back(c);
}

void Streamer::remove(Source *) {

}

void Streamer::remove(const std::string &) {

}

void Streamer::stop() {
	active_ = false;
	//pool_.stop();
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
	if (src->state == (ftl::rgbd::detail::kGrabbed | ftl::rgbd::detail::kRGB | ftl::rgbd::detail::kDepth)) {
		UNIQUE_LOCK(src->mutex,lk);

		if (src->rgb_buf.size() > 0 && src->d_buf.size() > 0) {
			auto i = src->clients[0].begin();
			while (i != src->clients[0].end()) {
				try {
					// TODO(Nick) Send pose and timestamp
					if (!net_->send((*i).peerid, (*i).uri, src->rgb_buf, src->d_buf)) {
						(*i).txcount = (*i).txmax;
					}
				} catch(...) {
					(*i).txcount = (*i).txmax;
				}
				(*i).txcount++;
				if ((*i).txcount >= (*i).txmax) {
					LOG(INFO) << "Remove client: " << (*i).uri;
					i = src->clients[0].erase(i);
				} else {
					i++;
				}
			}
		}
		src->src->getFrames(src->rgb, src->depth);
		src->state = 0;
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
		if (s.second->clients[0].size() == 0) {
			continue;
		}

		// There will be two jobs for this source...
		//UNIQUE_LOCK(job_mtx_,lk);
		jobs_ += 3;
		//lk.unlock();

		StreamSource *src = sources_[uri];
		if (src == nullptr || src->state != 0) continue;

		// Grab job
		ftl::pool.push([this,src](int id) {
			//StreamSource *src = sources_[uri];
			try {
				src->src->grab();
			} catch (...) {
				LOG(ERROR) << "Exception when grabbing frame";
			}

			// CHECK (Nick) Can state be an atomic instead?
			//UNIQUE_LOCK(src->mutex, lk);
			src->state |= ftl::rgbd::detail::kGrabbed;
			_swap(src);

			// Mark job as finished
			--jobs_;
			job_cv_.notify_one();
		});

		// Compress colour job
		ftl::pool.push([this,src](int id) {
			if (!src->rgb.empty()) {
				auto start = std::chrono::high_resolution_clock::now();

				//vector<unsigned char> src->rgb_buf;
				cv::imencode(".jpg", src->rgb, src->rgb_buf);
			}

			src->state |= ftl::rgbd::detail::kRGB;
			_swap(src);
			--jobs_;
			job_cv_.notify_one();
		});

		// Compress depth job
		ftl::pool.push([this,src](int id) {
			auto start = std::chrono::high_resolution_clock::now();

			if (!src->depth.empty()) {
				cv::Mat d2;
				src->depth.convertTo(d2, CV_16UC1, 16*100);
				//vector<unsigned char> d_buf;

				// Setting 1 = fast but large
				// Setting 9 = small but slow
				// Anything up to 8 causes minimal if any impact on frame rate
				// on my (Nicks) laptop, but 9 halves the frame rate.
				vector<int> pngparams = {cv::IMWRITE_PNG_COMPRESSION, 1}; // Default is 1 for fast, 9 = small but slow.
				cv::imencode(".png", d2, src->d_buf, pngparams);
			}

			std::chrono::duration<double> elapsed =
				std::chrono::high_resolution_clock::now() - start;
			LOG(INFO) << "Depth Compress in " << elapsed.count() << "s";

			src->state |= ftl::rgbd::detail::kDepth;
			_swap(src);
			--jobs_;
			job_cv_.notify_one();
		});

		// Transmit job
		// For any single source and bitrate there is only one thread
		// meaning that no lock is required here since outer shared_lock
		// prevents addition of new clients.
		// TODO, could do one for each bitrate...
		/* pool_.push([this,src](int id) {
			//StreamSource *src = sources_[uri];

			try {
			if (src && src->rgb.rows > 0 && src->depth.rows > 0 && src->clients[0].size() > 0) {
				auto start = std::chrono::high_resolution_clock::now();

				vector<unsigned char> rgb_buf;
				cv::imencode(".jpg", src->rgb, rgb_buf);

				std::chrono::duration<double> elapsed =
					std::chrono::high_resolution_clock::now() - start;
				LOG(INFO) << "JPG in " << elapsed.count() << "s";
				
				cv::Mat d2;
				src->depth.convertTo(d2, CV_16UC1, 16*100);
				vector<unsigned char> d_buf;

				// Setting 1 = fast but large
				// Setting 9 = small but slow
				// Anything up to 8 causes minimal if any impact on frame rate
				// on my (Nicks) laptop, but 9 halves the frame rate.
				vector<int> pngparams = {cv::IMWRITE_PNG_COMPRESSION, 1}; // Default is 1 for fast, 9 = small but slow.
				cv::imencode(".png", d2, d_buf, pngparams);

				//LOG(INFO) << "Data size: " << ((rgb_buf.size() + d_buf.size()) / 1024) << "kb";

				
			}
			} catch(...) {
				LOG(ERROR) << "Error in transmission loop";
			}

			// CHECK (Nick) Could state be an atomic?
			//UNIQUE_LOCK(src->mutex,lk);
			//LOG(INFO) << "Tx Frame: " << uri;
			src->state |= ftl::rgbd::detail::kTransmitted;
			_swap(*src);
			//lk.unlock();

			// Mark job as finished
			//UNIQUE_LOCK(job_mtx_,ulk);
			//jobs_--;
			//ulk.unlock();

			--jobs_;
			job_cv_.notify_one();
		});*/
	}
}

Source *Streamer::get(const std::string &uri) {
	SHARED_LOCK(mutex_,slk);
	if (sources_.find(uri) != sources_.end()) return sources_[uri]->src;
	else return nullptr;
}
