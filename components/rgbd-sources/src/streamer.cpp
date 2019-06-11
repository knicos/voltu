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

#define THREAD_POOL_SIZE 6

Streamer::Streamer(nlohmann::json &config, Universe *net)
		: ftl::Configurable(config), pool_(THREAD_POOL_SIZE), late_(false) {

	active_ = false;
	net_ = net;
	
	net->bind("find_stream", [this](const std::string &uri) -> optional<UUID> {
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
		shared_lock<shared_mutex> slk(mutex_);

		if (sources_.find(uri) != sources_.end()) {
			Eigen::Matrix4f pose;
			memcpy(pose.data(), buf.data(), buf.size());
			sources_[uri]->src->setPose(pose);
		}
	});

	// Allow remote users to access camera calibration matrix
	net->bind("source_calibration", [this](const std::string &uri) -> vector<unsigned char> {
		vector<unsigned char> buf;
		shared_lock<shared_mutex> slk(mutex_);

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
	pool_.stop();
}

void Streamer::add(Source *src) {
	unique_lock<shared_mutex> ulk(mutex_);
	if (sources_.find(src->getID()) != sources_.end()) return;

	StreamSource *s = new StreamSource;
	s->src = src;
	s->state = 0;
	sources_[src->getID()] = s;

	LOG(INFO) << "Streaming: " << src->getID();
}

void Streamer::_addClient(const string &source, int N, int rate, const ftl::UUID &peer, const string &dest) {
	shared_lock<shared_mutex> slk(mutex_);
	if (sources_.find(source) == sources_.end()) return;

	if (rate < 0 || rate >= 10) return;
	if (N < 0 || N > ftl::rgbd::kMaxFrames) return;

	//LOG(INFO) << "Adding Stream Peer: " << peer.to_string();

	StreamClient c;
	c.peerid = peer;
	c.uri = dest;
	c.txcount = 0;
	c.txmax = N;

	StreamSource *s = sources_[source];
	unique_lock<shared_mutex> ulk(s->mutex);
	s->clients[rate].push_back(c);
}

void Streamer::remove(Source *) {

}

void Streamer::remove(const std::string &) {

}

void Streamer::stop() {
	active_ = false;
	pool_.stop();
}

void Streamer::poll() {
	double wait = 1.0f / 25.0f;
	auto start = std::chrono::high_resolution_clock::now();
	// Create frame jobs at correct FPS interval
	_schedule();

	std::chrono::duration<double> elapsed =
		std::chrono::high_resolution_clock::now() - start;

	if (elapsed.count() >= wait) {
		LOG(WARNING) << "Frame rate below optimal @ " << (1.0f / elapsed.count());
	} else {
		// Otherwise, wait until next frame should start.
		// CHECK(Nick) Is this accurate enough?
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
		pool_.push([this](int id) {
			while (ftl::running && active_) {
				poll();
			}
		});
	}
}

void Streamer::_swap(StreamSource &src) {
	if (src.state == (ftl::rgbd::detail::kGrabbed | ftl::rgbd::detail::kTransmitted)) {
		src.src->getFrames(src.rgb, src.depth);
		src.state = 0;
	}
}

void Streamer::_schedule() {
	std::mutex job_mtx;
	std::condition_variable job_cv;
	int jobs = 0;

	for (auto s : sources_) {
		string uri = s.first;

		shared_lock<shared_mutex> slk(s.second->mutex);
		// CHECK Should never be true now
		if (s.second->state != 0) {
			if (!late_) LOG(WARNING) << "Stream not ready to schedule on time: " << uri;
			late_ = true;
			continue;
		} else {
			late_ = false;
		}

		// No point in doing work if no clients
		if (s.second->clients[0].size() == 0) {
			//LOG(ERROR) << "Stream has no clients: " << uri;
			continue;
		}
		slk.unlock();

		// There will be two jobs for this source...
		unique_lock<mutex> lk(job_mtx);
		jobs += 2;
		lk.unlock();

		// Grab job
		pool_.push([this,uri,&jobs,&job_mtx,&job_cv](int id) {
			StreamSource *src = sources_[uri];

			//auto start = std::chrono::high_resolution_clock::now();
			//try {
				src->src->grab();
			//} catch(...) {
			//	LOG(ERROR) << "Grab Exception for: " << uri;
			//}
			/*std::chrono::duration<double> elapsed =
					std::chrono::high_resolution_clock::now() - start;
			LOG(INFO) << "GRAB Elapsed: " << elapsed.count();*/

			unique_lock<shared_mutex> lk(src->mutex);
			//LOG(INFO) << "Grab frame";
			src->state |= ftl::rgbd::detail::kGrabbed;
			_swap(*src);
			lk.unlock();

			// Mark job as finished
			unique_lock<mutex> ulk(job_mtx);
			jobs--;
			ulk.unlock();
			job_cv.notify_one();
		});

		// Transmit job
		// TODO, could do one for each bitrate...
		pool_.push([this,uri,&jobs,&job_mtx,&job_cv](int id) {
			StreamSource *src = sources_[uri];

			try {
			if (src && src->rgb.rows > 0 && src->depth.rows > 0 && src->clients[0].size() > 0) {
				vector<unsigned char> rgb_buf;
				cv::imencode(".jpg", src->rgb, rgb_buf);
				
				cv::Mat d2;
				src->depth.convertTo(d2, CV_16UC1, 16*100);
				vector<unsigned char> d_buf;
				cv::imencode(".png", d2, d_buf);

				auto i = src->clients[0].begin();
				while (i != src->clients[0].end()) {
					try {
						if (!net_->send((*i).peerid, (*i).uri, rgb_buf, d_buf)) {
							(*i).txcount = (*i).txmax;
						}
					} catch(...) {
						(*i).txcount = (*i).txmax;
					}
					(*i).txcount++;
					if ((*i).txcount >= (*i).txmax) {
						LOG(INFO) << "Remove client";
						unique_lock<shared_mutex> lk(src->mutex);
						i = src->clients[0].erase(i);
					} else {
						i++;
					}
				}
			}
			} catch(...) {
				LOG(ERROR) << "Error in transmission loop";
			}

			/*std::chrono::duration<double> elapsed =
					std::chrono::high_resolution_clock::now() - start;
			LOG(INFO) << "Stream Elapsed: " << elapsed.count();*/

			unique_lock<shared_mutex> lk(src->mutex);
			DLOG(2) << "Tx Frame: " << uri;
			src->state |= ftl::rgbd::detail::kTransmitted;
			_swap(*src);
			lk.unlock();

			// Mark job as finished
			unique_lock<mutex> ulk(job_mtx);
			jobs--;
			ulk.unlock();
			job_cv.notify_one();
		});
	}

	// Wait for all jobs to complete before finishing frame
	unique_lock<mutex> lk(job_mtx);
	job_cv.wait(lk, [&jobs]{ return jobs == 0; });
}

Source *Streamer::get(const std::string &uri) {
	shared_lock<shared_mutex> slk(mutex_);
	if (sources_.find(uri) != sources_.end()) return sources_[uri]->src;
	else return nullptr;
}
