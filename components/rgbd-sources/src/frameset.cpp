#include <ftl/rgbd/frameset.hpp>
#include <ftl/timer.hpp>

#include <chrono>

using ftl::rgbd::Builder;
using ftl::rgbd::kMaxFramesets;
using ftl::rgbd::kMaxFramesInSet;
using std::vector;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;
using ftl::codecs::Channel;
using ftl::rgbd::FrameSet;
using ftl::codecs::Channels;

void FrameSet::upload(ftl::codecs::Channels<0> c, cudaStream_t stream) {
	for (auto &f : frames) {
		f.upload(c, stream);
	}
}

void FrameSet::download(ftl::codecs::Channels<0> c, cudaStream_t stream) {
	for (auto &f : frames) {
		f.download(c, stream);
	}
}

void FrameSet::swapTo(ftl::rgbd::FrameSet &fs) {
	UNIQUE_LOCK(fs.mtx, lk);

	//if (fs.frames.size() != frames.size()) {
		// Assume "this" is correct and "fs" is not.
		fs.frames.resize(frames.size());
	//}

	fs.timestamp = timestamp;
	fs.count = static_cast<int>(count);
	fs.stale = stale;
	fs.mask = static_cast<unsigned int>(mask);
	fs.id = id;

	for (size_t i=0; i<frames.size(); ++i) {
		frames[i].swapTo(Channels<0>::All(), fs.frames[i]);
	}

	stale = true;
}

void FrameSet::resetFull() {
	//count = 0;
	//stale = false;
	for (auto &f : frames) {
		//f.resetFull();
	}
}

// =============================================================================

Builder::Builder() : head_(0) {
	jobs_ = 0;
	skip_ = false;
	//setFPS(20);
	size_ = 0;

	mspf_ = ftl::timer::getInterval();
	name_ = "NoName";

	latency_ = 0.0f;;
	stats_count_ = 0;
	fps_ = 0.0f;
}

Builder::~Builder() {
	main_id_.cancel();

	UNIQUE_LOCK(mutex_, lk);
	// Make sure all jobs have finished
	while (jobs_ > 0) {
		sleep_for(milliseconds(10));
	}
}


void Builder::push(int64_t timestamp, int ix, ftl::rgbd::Frame &frame) {
	if (timestamp <= 0 || ix < 0 || ix >= kMaxFramesInSet) return;

	UNIQUE_LOCK(mutex_, lk);

	//LOG(INFO) << "BUILDER PUSH: " << timestamp << ", " << ix << ", " << size_;

	// Size is determined by largest frame index received... note that size
	// cannot therefore reduce.
	if (ix >= size_) {
		size_ = ix+1;
		states_.resize(size_);
	}
	states_[ix] = frame.origin();

	auto *fs = _findFrameset(timestamp);

	if (!fs) {
		// Add new frameset
		fs = _addFrameset(timestamp);
		if (!fs) return;
	}

	if (fs->frames.size() < size_) fs->frames.resize(size_);

	lk.unlock();
	SHARED_LOCK(fs->mtx, lk2);

	frame.swapTo(ftl::codecs::kAllChannels, fs->frames[ix]);

	++fs->count;

	if (fs->mask & (1 << ix)) {
		LOG(ERROR) << "Too many frames received for given timestamp: " << timestamp << " (source " << ix << ")";
	}
	fs->mask |= (1 << ix);
}

size_t Builder::size() {
	return size_;
}

void Builder::onFrameSet(const std::function<bool(ftl::rgbd::FrameSet &)> &cb) {
	if (!cb) {
		main_id_.cancel();
		return;
	}

	if (main_id_.id() != -1) {
		main_id_.cancel();
	}

	// 3. Issue IO retrieve ad compute jobs before finding a valid
	// frame at required latency to pass to callback.
	main_id_ = ftl::timer::add(ftl::timer::kTimerMain, [this,cb](int64_t ts) {
		//if (jobs_ > 0) LOG(ERROR) << "SKIPPING TIMER JOB " << ts;
		if (jobs_ > 0) return true;
		jobs_++;

		// Find a previous frameset and specified latency and do the sync
		// callback with that frameset.
		//if (latency_ > 0) {
			ftl::rgbd::FrameSet *fs = nullptr;
	
			UNIQUE_LOCK(mutex_, lk);
			fs = _getFrameset();

			//LOG(INFO) << "Latency for " << name_ << " = " << (latency_*ftl::timer::getInterval()) << "ms";

			if (fs) {
				UNIQUE_LOCK(fs->mtx, lk2);
				// The buffers are invalid after callback so mark stale
				fs->stale = true;
				lk.unlock();

				//LOG(INFO) << "PROCESS FRAMESET";

				//ftl::pool.push([this,fs,cb](int) {
					try {
						cb(*fs);
						//LOG(INFO) << "Frameset processed (" << name_ << "): " << fs->timestamp;
					} catch(std::exception &e) {
						LOG(ERROR) << "Exception in group sync callback: " << e.what();
					}

					//fs->resetFull();

					lk.lock();
					_freeFrameset(fs);

					jobs_--;
				//});
			} else {
				//LOG(INFO) << "NO FRAME FOUND: " << name_ << " " << size_;
				//latency_++;
				jobs_--;
			}
		//}

		//if (jobs_ == 0) LOG(INFO) << "LAST JOB =  Main";
		return true;
	});
}

ftl::rgbd::FrameState &Builder::state(int ix) {
	UNIQUE_LOCK(mutex_, lk);
	if (ix < 0 || ix >= states_.size()) {
		throw ftl::exception("Frame state out-of-bounds");
	}
	if (!states_[ix]) throw ftl::exception("Missing framestate");
	return *states_[ix];
}

static void mergeFrameset(ftl::rgbd::FrameSet &f1, ftl::rgbd::FrameSet &f2) {
	// Prepend all frame encodings in f2 into corresponding frame in f1.
	for (int i=0; i<f1.frames.size(); ++i) {
		if (f2.frames.size() <= i) break;
		f1.frames[i].mergeEncoding(f2.frames[i]);
	}
}

void Builder::_recordStats(float fps, float latency) {
	latency_ += latency;
	fps_ += fps;
	++stats_count_;

	if (fps_/float(stats_count_) <= float(stats_count_)) {
		fps_ /= float(stats_count_);
		latency_ /= float(stats_count_);
		LOG(INFO) << name_ << ": fps = " << fps_ << ", latency = " << latency_;
		fps_ = 0.0f;
		latency_ = 0.0f;
		stats_count_ = 0;
	}
}

ftl::rgbd::FrameSet *Builder::_findFrameset(int64_t ts) {
	// Search backwards to find match
	for (auto f : framesets_) {
		if (f->timestamp == ts) {
			return f;
		} else if (f->timestamp < ts) {
			return nullptr;
		}
	}

	return nullptr;
}

/*
 * Get the most recent completed frameset that isn't stale.
 * Note: Must occur inside a mutex lock.
 */
ftl::rgbd::FrameSet *Builder::_getFrameset() {
	//LOG(INFO) << "BUF SIZE = " << framesets_.size();
	for (auto i=framesets_.begin(); i!=framesets_.end(); i++) {
		auto *f = *i;
		//LOG(INFO) << "GET: " << f->count << " of " << size_;
		if (!f->stale && f->count >= size_) {
			//LOG(INFO) << "GET FRAMESET and remove: " << f->timestamp;
			auto j = framesets_.erase(i);
			
			int count = 0;
			// Merge all previous frames
			for (; j!=framesets_.end(); j++) {
				++count;

				auto *f2 = *j;
				j = framesets_.erase(j);
				mergeFrameset(*f,*f2);
				_freeFrameset(f2);
			}

			//if (count > 0) LOG(INFO) << "COUNT = " << count;

			int64_t now = ftl::timer::get_time();
			float framerate = 1000.0f / float(now - last_ts_);
			_recordStats(framerate, now - f->timestamp);
			last_ts_ = now;
			return f;
		}
	}

	return nullptr;
}

void Builder::_freeFrameset(ftl::rgbd::FrameSet *fs) {
	allocated_.push_back(fs);
}

ftl::rgbd::FrameSet *Builder::_addFrameset(int64_t timestamp) {
	if (allocated_.size() == 0) {
		if (framesets_.size() < kMaxFramesets) {
			allocated_.push_back(new ftl::rgbd::FrameSet);
		} else {
			LOG(WARNING) << "Frameset buffer full, resetting: " << timestamp;

			// Do a complete reset
			std::swap(framesets_, allocated_);
			//return nullptr;
		}
	}
	FrameSet *newf = allocated_.front();
	allocated_.pop_front();

	newf->timestamp = timestamp;
	newf->id = 0;
	newf->count = 0;
	newf->mask = 0;
	newf->stale = false;
	newf->frames.resize(size_);

	for (auto &f : newf->frames) f.reset();

	// Insertion sort by timestamp
	for (auto i=framesets_.begin(); i!=framesets_.end(); i++) {
		auto *f = *i;

		if (timestamp > f->timestamp) {
			framesets_.insert(i, newf);
			return newf;
		}
	}

	framesets_.push_back(newf);
	return newf;
}

void Builder::setName(const std::string &name) {
	name_ = name;
}



