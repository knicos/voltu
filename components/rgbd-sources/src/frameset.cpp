#include <ftl/rgbd/frameset.hpp>
#include <ftl/timer.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

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

float Builder::latency__ = 0.0f;
float Builder::fps__ = 0.0f;
int Builder::stats_count__ = 0;
MUTEX Builder::msg_mutex__;

/*void FrameSet::upload(ftl::codecs::Channels<0> c, cudaStream_t stream) {
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
	//for (auto &f : frames) {
		//f.resetFull();
	//}
}*/

// =============================================================================

Builder::Builder() : head_(0), id_(0) {
	jobs_ = 0;
	skip_ = false;
	bufferSize_ = 1;
	size_ = 0;
	last_frame_ = 0;

	mspf_ = ftl::timer::getInterval();
	name_ = "NoName";

	if (size_ > 0) states_.resize(size_);
}

Builder::~Builder() {
	main_id_.cancel();

	UNIQUE_LOCK(mutex_, lk);
	// Make sure all jobs have finished
	while (jobs_ > 0) {
		sleep_for(milliseconds(10));
	}
}

ftl::rgbd::Frame &Builder::get(int64_t timestamp, size_t ix) {
	if (timestamp <= 0 || ix >= kMaxFramesInSet) throw FTL_Error("Invalid frame timestamp or index");

	UNIQUE_LOCK(mutex_, lk);

	//LOG(INFO) << "BUILDER PUSH: " << timestamp << ", " << ix << ", " << size_;

	// Size is determined by largest frame index received... note that size
	// cannot therefore reduce.
	if (ix >= size_) {
		size_ = ix+1;
		states_.resize(size_);
	}
	//states_[ix] = frame.origin();

	if (timestamp <= last_frame_) {
		throw FTL_Error("Frameset already completed: " << timestamp);
	}

	auto *fs = _findFrameset(timestamp);

	if (!fs) {
		// Add new frameset
		fs = _addFrameset(timestamp);
		if (!fs) throw FTL_Error("Could not add frameset");

		_schedule();
	}

	if (fs->test(ftl::data::FSFlag::STALE)) {
		throw FTL_Error("Frameset already completed");
	}

	if (ix >= fs->frames.size()) {
		throw FTL_Error("Frame index out-of-bounds");
	}

	//if (fs->frames.size() < size_) fs->frames.resize(size_);

	//lk.unlock();
	//SHARED_LOCK(fs->mtx, lk2);

	//frame.swapTo(ftl::codecs::kAllChannels, fs->frames[ix]);

	fs->frames[ix].id = ix;
	return fs->frames[ix];
}

void Builder::completed(int64_t ts, size_t ix) {
	ftl::rgbd::FrameSet *fs = nullptr;

	{
		UNIQUE_LOCK(mutex_, lk);
		fs = _findFrameset(ts);
	}
	
	if (fs && ix < fs->frames.size()) {
		{
			UNIQUE_LOCK(fs->mtx, lk2);

			// If already completed for given frame, then skip
			if (fs->mask & (1 << ix)) return;

			states_[ix] = fs->frames[ix].origin();
			fs->mask |= (1 << ix);
			++fs->count;
		}

		// No buffering, so do a schedule here for immediate effect
		if (bufferSize_ == 0 && !fs->test(ftl::data::FSFlag::STALE) && static_cast<unsigned int>(fs->count) >= size_) {
			UNIQUE_LOCK(mutex_, lk);
			_schedule();
		}
	} else {
		LOG(ERROR) << "Completing frame that does not exist: " << ts << ":" << ix;
	}
}

void Builder::markPartial(int64_t ts) {
	ftl::rgbd::FrameSet *fs = nullptr;

	{
		UNIQUE_LOCK(mutex_, lk);
		fs = _findFrameset(ts);
		if (fs) fs->set(ftl::data::FSFlag::PARTIAL);
	}
}

void Builder::_schedule() {
	if (size_ == 0) return;
	ftl::rgbd::FrameSet *fs = nullptr;

	// Still working on a previously scheduled frame
	if (jobs_ > 0) return;

	// Find a valid / completed frameset to process
	fs = _getFrameset();

	// We have a frameset so create a thread job to call the onFrameset callback
	if (fs) {
		jobs_++;

		ftl::pool.push([this,fs](int) {
			UNIQUE_LOCK(fs->mtx, lk2);

			// Calling onFrameset but without all frames so mark as partial
			if (fs->count < fs->frames.size()) fs->set(ftl::data::FSFlag::PARTIAL);

			try {
				if (cb_) cb_(*fs);
			} catch(const ftl::exception &e) {
				LOG(ERROR) << "Exception in frameset builder: " << e.what();
				LOG(ERROR) << "Trace = " << e.trace();
			} catch(std::exception &e) {
				LOG(ERROR) << "Exception in frameset builder: " << e.what();
			}

			UNIQUE_LOCK(mutex_, lk);
			_freeFrameset(fs);
			jobs_--;

			// Schedule another frame immediately (or try to)
			_schedule();
		});
	}

}

size_t Builder::size() {
	return size_;
}

void Builder::onFrameSet(const std::function<bool(ftl::rgbd::FrameSet &)> &cb) {
	cb_ = cb;
}

ftl::rgbd::FrameState &Builder::state(size_t ix) {
	UNIQUE_LOCK(mutex_, lk);
	if (ix >= states_.size()) {
		throw FTL_Error("Frame state out-of-bounds: " << ix);
	}
	if (!states_[ix]) throw FTL_Error("Missing framestate");
	return *states_[ix];
}

static void mergeFrameset(ftl::rgbd::FrameSet &f1, ftl::rgbd::FrameSet &f2) {
	// Prepend all frame encodings in f2 into corresponding frame in f1.
	for (size_t i=0; i<f1.frames.size(); ++i) {
		if (f2.frames.size() <= i) break;
		f1.frames[i].mergeEncoding(f2.frames[i]);
	}
}

void Builder::_recordStats(float fps, float latency) {
	UNIQUE_LOCK(msg_mutex__, lk);
	latency__ += latency;
	fps__ += fps;
	++stats_count__;

	/*if (fps_/float(stats_count_) <= float(stats_count_)) {
		fps_ /= float(stats_count_);
		latency_ /= float(stats_count_);
		LOG(INFO) << name_ << ": fps = " << fps_ << ", latency = " << latency_;
		fps_ = 0.0f;
		latency_ = 0.0f;
		stats_count_ = 0;
	}*/
}

std::pair<float,float> Builder::getStatistics() {
	UNIQUE_LOCK(msg_mutex__, lk);
	if (stats_count__ == 0.0f) return {0.0f,0.0f};
	fps__ /= float(stats_count__);
	latency__ /= float(stats_count__);
	float fps = fps__;
	float latency = latency__;
	//LOG(INFO) << name_ << ": fps = " << fps_ << ", latency = " << latency_;
	fps__ = 0.0f;
	latency__ = 0.0f;
	stats_count__ = 0;
	return {fps,latency};
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

	auto i = framesets_.begin();
	int N = bufferSize_;

	// Skip N frames to fixed buffer location
	if (bufferSize_ > 0) {
		while (N-- > 0 && i != framesets_.end()) ++i;
	// Otherwise skip to first fully completed frame
	} else {
		while (i != framesets_.end() && (*i)->count < (*i)->frames.size()) ++i;
	}

	if (i != framesets_.end()) {
		auto *f = *i;
		//LOG(INFO) << "GET: " << f->count << " of " << size_;
		//if (!f->stale && static_cast<unsigned int>(f->count) >= size_) {
			//LOG(INFO) << "GET FRAMESET and remove: " << f->timestamp;
			auto j = framesets_.erase(i);

			last_frame_ = f->timestamp;
			//f->stale = true;
			f->set(ftl::data::FSFlag::STALE);
			
			int count = 0;
			// Merge all previous frames
			// TODO: Remove?
			while (j!=framesets_.end()) {
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
		//}
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
	newf->id = id_;
	newf->count = 0;
	newf->mask = 0;
	newf->clearFlags();
	newf->frames.resize(size_);
	newf->pose.setIdentity();
	newf->clearData();

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



