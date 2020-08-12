#include <ftl/streams/builder.hpp>
#include <ftl/timer.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#include <chrono>
#include <bitset>

using ftl::streams::BaseBuilder;
using ftl::streams::ForeignBuilder;
using ftl::streams::LocalBuilder;
using ftl::streams::IntervalSourceBuilder;
using ftl::streams::ManualSourceBuilder;
using ftl::streams::LockedFrameSet;
using ftl::data::FrameSet;
using ftl::data::Frame;
using namespace std::chrono;
using std::this_thread::sleep_for;


/*float Builder::latency__ = 0.0f;
float Builder::fps__ = 0.0f;
int Builder::stats_count__ = 0;
MUTEX Builder::msg_mutex__;*/

BaseBuilder::BaseBuilder(ftl::data::Pool *pool, int id) : pool_(pool), id_(id) {
	size_ = 1;
}

BaseBuilder::BaseBuilder() : pool_(nullptr), id_(0) {
	size_ = 1;
}

BaseBuilder::~BaseBuilder() {

}

// =============================================================================

LocalBuilder::LocalBuilder(ftl::data::Pool *pool, int id) : BaseBuilder(pool, id) {
	// Host receives responses that must propagate
	ctype_ = ftl::data::ChangeType::FOREIGN;
}

LocalBuilder::LocalBuilder() : BaseBuilder() {
	// Host receives responses that must propagate
	ctype_ = ftl::data::ChangeType::FOREIGN;
}

LocalBuilder::~LocalBuilder() {

}

LockedFrameSet LocalBuilder::get(int64_t timestamp, size_t ix) {
	SHARED_LOCK(mtx_, lk);
	if (!frameset_) {
		frameset_ = _allocate(timestamp);
	}

	LockedFrameSet lfs(frameset_.get(), &frameset_->smtx);
	return lfs;
}

LockedFrameSet LocalBuilder::get(int64_t timestamp) {
	SHARED_LOCK(mtx_, lk);
	if (!frameset_) {
		frameset_ = _allocate(timestamp);
	}

	LockedFrameSet lfs(frameset_.get(), &frameset_->smtx);
	return lfs;
}

void LocalBuilder::setFrameCount(size_t size) {
	// TODO: Resize the buffered frame!?
	size_ = size;
}

std::shared_ptr<ftl::data::FrameSet> LocalBuilder::getNextFrameSet(int64_t ts) {
	UNIQUE_LOCK(mtx_, lk);
	if (!frameset_) {
		frameset_ = _allocate(ts);
	}
	auto fs = frameset_;
	frameset_ = _allocate(ts+1);
	lk.unlock();

	// Must lock to ensure no updates can happen here
	UNIQUE_LOCK(fs->smtx, lk2);
	fs->changeTimestamp(ts);
	fs->localTimestamp = ts;
	fs->store();
	//for (auto &f : fs->frames) {
	//	f.store();
	//}
	return fs;
}

std::shared_ptr<ftl::data::FrameSet> LocalBuilder::_allocate(int64_t timestamp) {
	auto newf = std::make_shared<FrameSet>(pool_, ftl::data::FrameID(id_,255), timestamp);
	for (size_t i=0; i<size_; ++i) {
		newf->frames.push_back(std::move(pool_->allocate(ftl::data::FrameID(id_, i), timestamp)));
	}

	newf->mask = 0xFF;
	newf->clearFlags();
	return newf;
}

// =============================================================================

IntervalSourceBuilder::IntervalSourceBuilder(ftl::data::Pool *pool, int id, ftl::data::DiscreteSource *src) : LocalBuilder(pool, id), srcs_({src}) {

}

IntervalSourceBuilder::IntervalSourceBuilder(ftl::data::Pool *pool, int id, const std::list<ftl::data::DiscreteSource*> &srcs) : LocalBuilder(pool, id), srcs_(srcs) {

}

IntervalSourceBuilder::IntervalSourceBuilder() : LocalBuilder() {

}

IntervalSourceBuilder::~IntervalSourceBuilder() {

}

void IntervalSourceBuilder::start() {
	capture_ = std::move(ftl::timer::add(ftl::timer::timerlevel_t::kTimerHighPrecision, [this](int64_t ts) {
		for (auto *s : srcs_) s->capture(ts);
		return true;
	}));

	retrieve_ = std::move(ftl::timer::add(ftl::timer::timerlevel_t::kTimerMain, [this](int64_t ts) {
		auto fs = getNextFrameSet(ts);

		// TODO: Do in parallel...
		for (auto *s : srcs_) {
			if (!s->retrieve(fs->firstFrame())) {
				LOG(WARNING) << "Frame is being skipped: " << ts;
				fs->firstFrame().message(ftl::data::Message::Warning_FRAME_DROP, "Frame is being skipped");
			}
		}

		cb_.trigger(fs);
		return true;
	}));
}

void IntervalSourceBuilder::stop() {
	capture_.cancel();
	retrieve_.cancel();
}

// =============================================================================

ManualSourceBuilder::ManualSourceBuilder(ftl::data::Pool *pool, int id, ftl::data::DiscreteSource *src) : LocalBuilder(pool, id), src_(src) {

}

ManualSourceBuilder::ManualSourceBuilder() : LocalBuilder(), src_(nullptr) {

}

ManualSourceBuilder::~ManualSourceBuilder() {

}

void ManualSourceBuilder::tick() {
	if (!src_) return;

	int64_t ts = ftl::timer::get_time();
	if (ts < last_timestamp_ + mspf_) return;
	last_timestamp_ = ts;

	src_->capture(ts);

	auto fs = getNextFrameSet(ts);

	if (!src_->retrieve(fs->firstFrame())) {
		LOG(WARNING) << "Frame was skipping";
		fs->firstFrame().message(ftl::data::Message::Warning_FRAME_DROP, "Frame is being skipped");
	}

	cb_.trigger(fs);
}

// =============================================================================

ForeignBuilder::ForeignBuilder(ftl::data::Pool *pool, int id) : BaseBuilder(pool, id), head_(0) {
	jobs_ = 0;
	skip_ = false;
	bufferSize_ = 0;
	last_frame_ = 0;

	mspf_ = ftl::timer::getInterval();
}

ForeignBuilder::ForeignBuilder() : BaseBuilder(), head_(0) {
	jobs_ = 0;
	skip_ = false;
	bufferSize_ = 0;
	last_frame_ = 0;

	mspf_ = ftl::timer::getInterval();
}

ForeignBuilder::~ForeignBuilder() {
	main_id_.cancel();

	UNIQUE_LOCK(mutex_, lk);
	// Make sure all jobs have finished
	while (jobs_ > 0) {
		sleep_for(milliseconds(10));
	}

	// Also make sure to get unique lock on any processing framesets.
	for (auto &f : framesets_) {
		UNIQUE_LOCK(f->smtx, lk);
	}
}

LockedFrameSet ForeignBuilder::get(int64_t timestamp) {
	if (timestamp <= 0) throw FTL_Error("Invalid frame timestamp");

	UNIQUE_LOCK(mutex_, lk);

	auto fs = _get(timestamp);

	if (fs) {
		LockedFrameSet lfs(fs.get(), &fs->smtx, [this,fs](ftl::data::FrameSet *d) {
			if (fs->isComplete()) {
				if (bufferSize_ == 0 && !fs->test(ftl::data::FSFlag::STALE)) {
					UNIQUE_LOCK(mutex_, lk);
					_schedule();
				}
			}
		});
		return lfs;
	} else {
		return LockedFrameSet();
	}
}

std::shared_ptr<ftl::data::FrameSet> ForeignBuilder::_get(int64_t timestamp) {
	if (timestamp <= last_frame_) {
		//throw FTL_Error("Frameset already completed: " << timestamp << " (" << last_frame_ << ")");
		LOG(ERROR) << "Frameset already completed: " << timestamp << " (" << last_frame_ << ")";
		return nullptr;
	}

	auto fs = _findFrameset(timestamp);

	if (!fs) {
		// Add new frameset
		fs = _addFrameset(timestamp);
		if (!fs) throw FTL_Error("Could not add frameset");

		_schedule();
	}

	/*if (fs->test(ftl::data::FSFlag::STALE)) {
		throw FTL_Error("Frameset already completed");
	}*/
	return fs;
}

LockedFrameSet ForeignBuilder::get(int64_t timestamp, size_t ix) {
	if (ix == 255) {
		UNIQUE_LOCK(mutex_, lk);

		if (timestamp <= 0) throw FTL_Error("Invalid frame timestamp (" << timestamp << ")");
		auto fs = _get(timestamp);

		if (fs) {
			LockedFrameSet lfs(fs.get(), &fs->smtx);
			return lfs;
		} else {
			return LockedFrameSet();
		}
	} else {
		if (timestamp <= 0 || ix >= 32) throw FTL_Error("Invalid frame timestamp or index (" << timestamp << ", " << ix << ")");

		UNIQUE_LOCK(mutex_, lk);

		if (ix >= size_) {
			size_ = ix+1;
		}

		auto fs = _get(timestamp);

		if (fs) {
			if (ix >= fs->frames.size()) {
				// FIXME: Check that no access to frames can occur without lock
				UNIQUE_LOCK(fs->smtx, flk);
				while (fs->frames.size() < size_) {
					fs->frames.push_back(std::move(pool_->allocate(ftl::data::FrameID(fs->frameset(), + fs->frames.size()), fs->timestamp())));
				}
			}

			LockedFrameSet lfs(fs.get(), &fs->smtx, [this,fs](ftl::data::FrameSet *d) {
				if (fs->isComplete()) {
					if (bufferSize_ == 0 && !fs->test(ftl::data::FSFlag::STALE)) {
						UNIQUE_LOCK(mutex_, lk);
						_schedule();
					}
				}
			});

			return lfs;
		} else {
			return LockedFrameSet();
		}
	}
}

void ForeignBuilder::_schedule() {
	if (size_ == 0) return;
	std::shared_ptr<ftl::data::FrameSet> fs;

	// Still working on a previously scheduled frame
	if (jobs_ > 0) return;

	// Find a valid / completed frameset to process
	fs = _getFrameset();

	// We have a frameset so create a thread job to call the onFrameset callback
	if (fs) {
		jobs_++;

		ftl::pool.push([this,fs](int) {
			fs->store();

			if (!fs->isComplete()) {
				fs->set(ftl::data::FSFlag::PARTIAL);
				fs->frames[0].message(ftl::data::Message::Warning_INCOMPLETE_FRAME, "Frameset not complete");
			}

			//UNIQUE_LOCK(fs->mutex(), lk2);

			try {
				cb_.trigger(fs);
			} catch(const ftl::exception &e) {
				LOG(ERROR) << "Exception in frameset builder: " << e.what();
				//LOG(ERROR) << "Trace = " << e.trace();
			} catch(std::exception &e) {
				LOG(ERROR) << "Exception in frameset builder: " << e.what();
			}

			UNIQUE_LOCK(mutex_, lk);
			//_freeFrameset(fs);
			jobs_--;

			// Schedule another frame immediately (or try to)
			_schedule();
		});
	}

}

std::pair<float,float> BaseBuilder::getStatistics() {
	return {-1.0f, -1.0f};
}

std::shared_ptr<ftl::data::FrameSet> ForeignBuilder::_findFrameset(int64_t ts) {
	// Search backwards to find match
	for (auto f : framesets_) {
		if (f->timestamp() == ts) {
			return f;
		} else if (f->timestamp() < ts) {
			return nullptr;
		}
	}

	return nullptr;
}

/*
 * Get the most recent completed frameset that isn't stale.
 * Note: Must occur inside a mutex lock.
 */
std::shared_ptr<ftl::data::FrameSet> ForeignBuilder::_getFrameset() {
	ftl::data::FrameSetPtr f;
	auto i = framesets_.begin();
	int N = bufferSize_;

	// Skip N frames to fixed buffer location
	if (bufferSize_ > 0) {
		while (N-- > 0 && i != framesets_.end()) ++i;
		if (i != framesets_.end()) f = *i;
	} else {
		// Force complete of old frame
		if (framesets_.size() >= completion_size_) {
			LOG(WARNING) << "Forced completion: " << framesets_.back()->timestamp();
			framesets_.back()->mask = 0xFF;
		}

		// Always choose oldest frameset when it completes
		if (framesets_.size() > 0 && framesets_.back()->isComplete()) f = framesets_.back();
	}

	if (f) {
		// Lock to force completion of on going construction first
		UNIQUE_LOCK(f->smtx, slk);
		last_frame_ = f->timestamp();
		f->set(ftl::data::FSFlag::STALE);
		slk.unlock();

		if (!f->isComplete()) LOG(WARNING) << "Dispatching incomplete frameset: " << f->timestamp() << " (" << std::bitset<16>( f->mask ).to_string() << ")";

		// Remove all previous framesets
		while (framesets_.size() > 0) {
			ftl::data::FrameSetPtr &f2 = framesets_.back();
			if (f2.get() == f.get()) break;

			LOG(WARNING) << "FrameSet discarded: " << f2->timestamp() << " (" << f->timestamp() << ")";
			f2->set(ftl::data::FSFlag::DISCARD);
			{
				// Ensure frame processing is finished first
				UNIQUE_LOCK(f2->smtx, lk);
			}

			framesets_.pop_back();
		}

		framesets_.pop_back();
		return f;
	}

	return nullptr;
}

std::shared_ptr<ftl::data::FrameSet> ForeignBuilder::_addFrameset(int64_t timestamp) {
	if (framesets_.size() >= max_buffer_size_) {
		LOG(WARNING) << "Frameset buffer full, resetting: " << timestamp;
		framesets_.clear();
		//framesets_.pop_back();
	}

	auto newf = std::make_shared<FrameSet>(pool_, ftl::data::FrameID(id_,255), timestamp, size_);
	for (size_t i=0; i<size_; ++i) {
		newf->frames.push_back(std::move(pool_->allocate(ftl::data::FrameID(id_, i), timestamp)));
	}

	newf->mask = 0;
	newf->localTimestamp = timestamp;
	newf->clearFlags();

	// Insertion sort by timestamp
	for (auto i=framesets_.begin(); i!=framesets_.end(); i++) {
		auto f = *i;

		if (timestamp > f->timestamp()) {
			framesets_.insert(i, newf);
			return newf;
		}
	}

	framesets_.push_back(newf);
	return newf;
}
