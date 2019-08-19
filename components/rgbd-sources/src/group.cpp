#include <ftl/rgbd/group.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/timer.hpp>

#include <chrono>

using ftl::rgbd::Group;
using ftl::rgbd::Source;
using ftl::rgbd::kFrameBufferSize;
using std::vector;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

Group::Group() : framesets_(kFrameBufferSize), head_(0) {
	framesets_[0].timestamp = -1;
	jobs_ = 0;
	skip_ = false;
	//setFPS(20);

	mspf_ = ftl::timer::getInterval();

	setLatency(5);
}

Group::~Group() {
	for (auto s : sources_) {
		s->removeCallback();
	}

	main_id_.cancel();
	swap_id_.cancel();
	cap_id_.cancel();

	UNIQUE_LOCK(mutex_, lk);
	// Make sure all jobs have finished
	while (jobs_ > 0) {
		sleep_for(milliseconds(10));
	}
}

//void Group::setFPS(int fps) {
//	mspf_ = 1000 / fps;
//	ftl::timer::setInterval(mspf_);
//}

void Group::addSource(ftl::rgbd::Source *src) {
	UNIQUE_LOCK(mutex_, lk);
	size_t ix = sources_.size();
	sources_.push_back(src);

	src->setCallback([this,ix,src](int64_t timestamp, cv::Mat &rgb, cv::Mat &depth) {
		if (timestamp == 0) return;

		//LOG(INFO) << "SRC CB: " << timestamp << " (" << framesets_[head_].timestamp << ")";

		UNIQUE_LOCK(mutex_, lk);
		if (timestamp > framesets_[head_].timestamp) {
			// Add new frameset
			_addFrameset(timestamp);
		} else if (framesets_[(head_+1)%kFrameBufferSize].timestamp > timestamp) {
			// Too old, just ditch it
			LOG(WARNING) << "Received frame too old for buffer";
			return;
		}

		// Search backwards to find match
		for (size_t i=0; i<kFrameBufferSize; ++i) {
			FrameSet &fs = framesets_[(head_+kFrameBufferSize-i) % kFrameBufferSize];
			if (fs.timestamp == timestamp) {
				lk.unlock();
				UNIQUE_LOCK(fs.mtx, lk2);

				//LOG(INFO) << "Adding frame: " << ix << " for " << timestamp;
				// Ensure channels match source mat format
				fs.channel1[ix].create(rgb.size(), rgb.type());
				fs.channel2[ix].create(depth.size(), depth.type());

				cv::swap(rgb, fs.channel1[ix]);
				cv::swap(depth, fs.channel2[ix]);

				++fs.count;
				fs.mask |= (1 << ix);

				if (fs.count == sources_.size()) {
					//LOG(INFO) << "COMPLETE SET: " << fs.timestamp;
				} else {
					//LOG(INFO) << "INCOMPLETE SET ("  << ix << "): " << fs.timestamp;
				}

				if (callback_ && fs.count == sources_.size()) {
					if (callback_(fs)) {
						// TODO: Remove callback if returns false?
					}

					// Reset count to prevent multiple reads of these frames
					fs.count = 0;
				}

				return;
			}
		}
		DLOG(WARNING) << "Frame timestamp not found in buffer";
	});
}

void Group::addGroup(Group *grp) {
	
}

void Group::_retrieveJob(ftl::rgbd::Source *src) {
	try {
		src->retrieve();
	} catch (std::exception &ex) {
		LOG(ERROR) << "Exception when retrieving frame";
		LOG(ERROR) << ex.what();
	}
	catch (...) {
		LOG(ERROR) << "Unknown exception when retrieving frame";
	}
}

void Group::_computeJob(ftl::rgbd::Source *src) {
	try {
		src->compute();
	} catch (std::exception &ex) {
		LOG(ERROR) << "Exception when computing frame";
		LOG(ERROR) << ex.what();
	}
	catch (...) {
		LOG(ERROR) << "Unknown exception when computing frame";
	}
}

void Group::sync(std::function<bool(ftl::rgbd::FrameSet &)> cb) {
	if (latency_ == 0) {
		callback_ = cb;
	}

	// 1. Capture camera frames with high precision
	cap_id_ = ftl::timer::add(ftl::timer::kTimerHighPrecision, [this](int64_t ts) {
		skip_ = jobs_ != 0;  // Last frame not finished so skip all steps

		if (skip_) return;

		last_ts_ = ts;
		for (auto s : sources_) {
			s->capture(ts);
		}
	});

	// 2. After capture, swap any internal source double buffers
	swap_id_ = ftl::timer::add(ftl::timer::kTimerSwap, [this](int64_t ts) {
		if (skip_) return;
		for (auto s : sources_) {
			s->swap();
		}
	});

	// 3. Issue IO retrieve ad compute jobs before finding a valid
	// frame at required latency to pass to callback.
	main_id_ = ftl::timer::add(ftl::timer::kTimerMain, [this,cb](int64_t ts) {
		if (skip_) return;
		jobs_++;

		for (auto s : sources_) {
			jobs_ += 2;

			ftl::pool.push([this,s](int id) {
				_retrieveJob(s);
				--jobs_;
			});
			ftl::pool.push([this,s](int id) {
				_computeJob(s);
				--jobs_;
			});
		}

		// Find a previous frameset and specified latency and do the sync
		// callback with that frameset.
		if (latency_ > 0) {
			ftl::rgbd::FrameSet *fs = nullptr;
	
			UNIQUE_LOCK(mutex_, lk);
			fs = _getFrameset(latency_);

			if (fs) {
				UNIQUE_LOCK(fs->mtx, lk2);
				lk.unlock();
				cb(*fs);

				// The buffers are invalid after callback so mark stale
				fs->stale = true;
			} else {
				//LOG(INFO) << "NO FRAME FOUND: " << last_ts_ - latency_*mspf_;
			}
		}

		jobs_--;
	});

	ftl::timer::start(true);
}

//ftl::rgbd::FrameSet &Group::_getRelativeFrameset(int rel) {
//	int idx = (rel < 0) ? (head_+kFrameBufferSize+rel)%kFrameBufferSize : (head_+rel)%kFrameBufferSize;
//	return framesets_[idx];
//}

ftl::rgbd::FrameSet *Group::_getFrameset(int f) {
	const int64_t lookfor = last_ts_-f*mspf_;

	for (size_t i=1; i<kFrameBufferSize; ++i) {
		int idx = (head_+kFrameBufferSize-i)%kFrameBufferSize;

		if (framesets_[idx].timestamp == lookfor && framesets_[idx].count != sources_.size()) {
			LOG(INFO) << "Required frame not complete (mask " << (framesets_[idx].timestamp) << ")";
			continue;
		}

		if (framesets_[idx].stale) return nullptr;

		if (framesets_[idx].timestamp == lookfor && framesets_[idx].count == sources_.size()) {
			//framesets_[idx].stale = false;
			return &framesets_[idx];
		} else if (framesets_[idx].timestamp < lookfor && framesets_[idx].count == sources_.size()) {
			//framesets_[idx].stale = true;
			return &framesets_[idx];
		}

	}
	return nullptr;
}

void Group::_addFrameset(int64_t timestamp) {
	int count = (framesets_[head_].timestamp == -1) ? 200 : (timestamp - framesets_[head_].timestamp) / mspf_;
	//LOG(INFO) << "Massive timestamp difference: " << count;

	// Allow for massive timestamp changes (Windows clock adjust)
	// Only add a single frameset for large changes
	if (count < -int(kFrameBufferSize) || count >= kFrameBufferSize-1) {
		head_ = (head_+1) % kFrameBufferSize;

		if (!framesets_[head_].mtx.try_lock()) {
			LOG(ERROR) << "Frameset in use!!";
			return;
		}
		framesets_[head_].timestamp = timestamp;
		framesets_[head_].count = 0;
		framesets_[head_].mask = 0;
		framesets_[head_].stale = false;
		framesets_[head_].channel1.resize(sources_.size());
		framesets_[head_].channel2.resize(sources_.size());

		if (framesets_[head_].sources.size() != sources_.size()) {
			framesets_[head_].sources.clear();
			for (auto s : sources_) framesets_[head_].sources.push_back(s);
		}
		framesets_[head_].mtx.unlock();
		return;
	}

	if (count < 1) return;

	// Must make sure to also insert missing framesets
	for (int i=0; i<count; ++i) {
		int64_t lt = (framesets_[head_].timestamp == -1) ? timestamp-mspf_ : framesets_[head_].timestamp;
		head_ = (head_+1) % kFrameBufferSize;

		if (!framesets_[head_].mtx.try_lock()) {
			LOG(ERROR) << "Frameset in use!!";
			break;
		}
		framesets_[head_].timestamp = lt+mspf_;
		framesets_[head_].count = 0;
		framesets_[head_].mask = 0;
		framesets_[head_].stale = false;
		framesets_[head_].channel1.resize(sources_.size());
		framesets_[head_].channel2.resize(sources_.size());

		if (framesets_[head_].sources.size() != sources_.size()) {
			framesets_[head_].sources.clear();
			for (auto s : sources_) framesets_[head_].sources.push_back(s);
		}
		framesets_[head_].mtx.unlock();
	}
}


