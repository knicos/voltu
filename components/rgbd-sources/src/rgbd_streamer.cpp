#include <ftl/rgbd_streamer.hpp>
#include <vector>
#include <optional>
#include <thread>
#include <chrono>

using ftl::rgbd::Streamer;
using ftl::rgbd::RGBDSource;
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
		: ftl::Configurable(config), pool_(THREAD_POOL_SIZE) {

	active_ = false;
	net_ = net;
	
	net->bind("find_stream", [this](const std::string &uri) -> optional<UUID> {
		if (sources_.find(uri) != sources_.end()) return net_->id();
		else return {};
	});

	net->bind("list_streams", [this]() -> vector<string> {
		vector<string> streams;
		for (auto &i : sources_) {
			streams.push_back(i.first);
		}
		return streams;
	});

	// Allow remote users to access camera calibration matrix
	net->bind("source_calibration", [this](const std::string &uri) -> vector<unsigned char> {
		vector<unsigned char> buf;
		shared_lock<shared_mutex> slk(mutex_);

		if (sources_.find(uri) != sources_.end()) {
			buf.resize(sizeof(CameraParameters));
			LOG(INFO) << "Calib buf size = " << buf.size();
			memcpy(buf.data(), &sources_[uri]->src->getParameters(), buf.size());
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
	// TODO Unbind everything from net....
	pool_.stop();
}

void Streamer::add(RGBDSource *src) {
	unique_lock<shared_mutex> ulk(mutex_);
	if (sources_.find(src->getURI()) != sources_.end()) return;

	StreamSource *s = new StreamSource; // = sources_.emplace(std::make_pair<std::string,StreamSource>(src->getURI(),{}));
	s->src = src;
	s->state = 0;
	sources_[src->getURI()] = s;

	LOG(INFO) << "Streaming: " << src->getURI();
}

void Streamer::_addClient(const string &source, int N, int rate, const ftl::UUID &peer, const string &dest) {
	shared_lock<shared_mutex> slk(mutex_);
	if (sources_.find(source) == sources_.end()) return;

	if (rate < 0 || rate >= 10) return;
	if (N < 0 || N > ftl::rgbd::kMaxFrames) return;

	StreamClient c;
	c.peerid = peer;
	c.uri = dest;
	c.txcount = 0;
	c.txmax = N;

	StreamSource *s = sources_[source];
	unique_lock<shared_mutex> ulk(s->mutex);
	s->clients[rate].push_back(c);
}

void Streamer::remove(RGBDSource *) {

}

void Streamer::remove(const std::string &) {

}

void Streamer::stop() {
	active_ = false;
}

void Streamer::run(bool block) {
	active_ = true;

	if (block) {
		while (active_) {
			double wait = 1.0f / 25.0f;
			auto start = std::chrono::high_resolution_clock::now();
			// Create frame jobs at correct FPS interval
			_schedule();

			std::chrono::duration<double> elapsed =
				std::chrono::high_resolution_clock::now() - start;

			sleep_for(milliseconds((long long)((wait - elapsed.count()) * 1000.0f)));
		}
	} else {
		// Create thread job for frame ticking
		pool_.push([this](int id) {
			while (active_) {
				double wait = 1.0f / 25.0f;
				auto start = std::chrono::high_resolution_clock::now();
				// Create frame jobs at correct FPS interval
				_schedule();

				std::chrono::duration<double> elapsed =
					std::chrono::high_resolution_clock::now() - start;

				sleep_for(milliseconds((long long)((wait - elapsed.count()) * 1000.0f)));
			}
		});
	}
}

void Streamer::_swap(StreamSource &src) {
	if (src.state == (ftl::rgbd::detail::kGrabbed | ftl::rgbd::detail::kTransmitted)) {
		src.src->getRGBD(src.rgb, src.depth);
		src.state = 0;
	}
}

void Streamer::_schedule() {
	for (auto s : sources_) {
		string uri = s.first;

		shared_lock<shared_mutex> slk(s.second->mutex);
		if (s.second->state != 0) {
			LOG(ERROR) << "Stream not ready to schedule on time: " << uri;
			continue;
		}
		if (s.second->clients[0].size() == 0) {
			//LOG(ERROR) << "Stream has no clients: " << uri;
			continue;
		}
		slk.unlock();

		// Grab job
		pool_.push([this,uri](int id) {
			StreamSource *src = sources_[uri];
			src->src->grab();

			unique_lock<shared_mutex> lk(src->mutex);
			src->state |= ftl::rgbd::detail::kGrabbed;
			_swap(*src);
		});

		// Transmit job
		// TODO, could do one for each bitrate...
		pool_.push([this,uri](int id) {
			StreamSource *src = sources_[uri];

			if (src->rgb.rows > 0 && src->depth.rows > 0 && src->clients[0].size() > 0) {
				vector<unsigned char> rgb_buf;
				cv::imencode(".jpg", src->rgb, rgb_buf);
				
				cv::Mat d2;
				src->depth.convertTo(d2, CV_16UC1, 16*100);
				vector<unsigned char> d_buf;
				cv::imencode(".png", d2, d_buf);

				auto i = src->clients[0].begin();
				while (i != src->clients[0].end()) {
					try {
						net_->send((*i).peerid, (*i).uri, rgb_buf, d_buf);
					} catch(...) {
						(*i).txcount = (*i).txmax;
					}
					(*i).txcount++;
					if ((*i).txcount >= (*i).txmax) {
						LOG(INFO) << "Remove client";
						i = src->clients[0].erase(i);
					} else {
						i++;
					}
				}
			}

			unique_lock<shared_mutex> lk(src->mutex);
			LOG(INFO) << "Tx Frame: " << uri;
			src->state |= ftl::rgbd::detail::kTransmitted;
			_swap(*src);
		});
	}
}

RGBDSource *Streamer::get(const std::string &uri) {
	shared_lock<shared_mutex> slk(mutex_);
	if (sources_.find(uri) != sources_.end()) return sources_[uri]->src;
	else return nullptr;
}
