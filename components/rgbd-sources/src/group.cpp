#include <ftl/rgbd/group.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/timer.hpp>
#include <ftl/operators/operator.hpp>

#include <chrono>

using ftl::rgbd::Group;
using ftl::rgbd::Source;
using ftl::rgbd::kMaxFramesets;
using std::vector;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;
using ftl::codecs::Channel;

Group::Group() : pipeline_(nullptr), head_(0) {
	//framesets_[0].timestamp = -1;
	//framesets_[0].stale = true;

	/*for (auto &i : framesets_) {
		i.stale = true;
	}*/

	// Allocate some initial framesets
	//for (int i=0; i<10; ++i) {
	//	allocated_.push_back(new ftl::rgbd::FrameSet);
	//}

	jobs_ = 0;
	skip_ = false;
	//setFPS(20);

	mspf_ = ftl::timer::getInterval();
	name_ = "NoName";

	latency_ = 0.0f;;
	stats_count_ = 0;
	fps_ = 0.0f;
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

	src->setCallback([this,ix,src](int64_t timestamp, ftl::rgbd::Frame &frame) {
		if (timestamp == 0) return;

		auto chan = src->getChannel();

		//LOG(INFO) << "SRC CB (" << name_ << "): " << timestamp << " (" << ")";

		UNIQUE_LOCK(mutex_, lk);
		auto *fs = _findFrameset(timestamp);

		if (!fs) {
			// Add new frameset
			fs = _addFrameset(timestamp);

			if (!fs) return;
		} /*else if (framesets_[(head_+1)%kFrameBufferSize].timestamp > timestamp) {
			// Too old, just ditch it
			LOG(WARNING) << "Received frame too old for buffer";
			return;
		}*/

		// Search backwards to find match
		//for (size_t i=0; i<kFrameBufferSize; ++i) {
		//	FrameSet &fs = framesets_[(head_+kFrameBufferSize-i) % kFrameBufferSize];
			//if (fs.timestamp == timestamp) {
				lk.unlock();
				SHARED_LOCK(fs->mtx, lk2);

				frame.swapTo(ftl::codecs::kAllChannels, fs->frames[ix]);

				if (fs->count+1 == sources_.size()) {
					if (pipeline_) {
						pipeline_->apply(*fs, *fs, 0);
					}
				}

				++fs->count;
				fs->mask |= (1 << ix);

				if (fs->count == sources_.size()) {
					//LOG(INFO) << "COMPLETE SET (" << name_ << "): " << fs->timestamp;
				} else if (fs->count > sources_.size()) {
					LOG(ERROR) << "Too many frames for frame set: " << fs->timestamp << " sources=" << sources_.size();
				} else {
					//LOG(INFO) << "INCOMPLETE SET ("  << ix << "): " << fs.timestamp;
				}

				/*if (callback_ && fs->count == sources_.size()) {
					try {
						if (callback_(fs)) {
							// TODO: Remove callback if returns false?
						}
					} catch (...) {
						LOG(ERROR) << "Exception in group callback";
					}

					// Reset count to prevent multiple reads of these frames
					//fs.count = 0;
				}*/

				return;
			//}
		//}
		//LOG(WARNING) << "Frame timestamp not found in buffer";
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

int Group::streamID(const ftl::rgbd::Source *s) const {
	for (int i=0; i<sources_.size(); ++i) {
		if (sources_[i] == s) return i;
	}
	return -1;
}

void Group::sync(std::function<bool(ftl::rgbd::FrameSet &)> cb) {
	//if (latency_ == 0) {
	//	callback_ = cb;
	//}

	// 1. Capture camera frames with high precision
	cap_id_ = ftl::timer::add(ftl::timer::kTimerHighPrecision, [this](int64_t ts) {
		skip_ = jobs_ != 0;  // Last frame not finished so skip all steps

		if (skip_) return true;

		for (auto s : sources_) {
			s->capture(ts);
		}

		return true;
	});

	// 2. After capture, swap any internal source double buffers
	swap_id_ = ftl::timer::add(ftl::timer::kTimerSwap, [this](int64_t ts) {
		if (skip_) return true;
		for (auto s : sources_) {
			s->swap();
		}
		return true;
	});

	// 3. Issue IO retrieve ad compute jobs before finding a valid
	// frame at required latency to pass to callback.
	main_id_ = ftl::timer::add(ftl::timer::kTimerMain, [this,cb](int64_t ts) {
		//if (skip_) LOG(ERROR) << "SKIPPING TIMER JOB " << ts;
		if (skip_) return true;
		jobs_++;

		for (auto s : sources_) {
			jobs_ += 2;

			ftl::pool.push([this,s](int id) {
				_retrieveJob(s);
				//if (jobs_ == 0) LOG(INFO) << "LAST JOB =  Retrieve";
				--jobs_;
			});
			ftl::pool.push([this,s](int id) {
				_computeJob(s);
				//if (jobs_ == 0) LOG(INFO) << "LAST JOB =  Compute";
				--jobs_;
			});
		}

		// Find a previous frameset and specified latency and do the sync
		// callback with that frameset.
		//if (latency_ > 0) {
			ftl::rgbd::FrameSet *fs = nullptr;
	
			UNIQUE_LOCK(mutex_, lk);
			fs = _getFrameset();

			//LOG(INFO) << "Latency for " << name_ << " = " << (latency_*ftl::timer::getInterval()) << "ms";

			if (fs) {
				UNIQUE_LOCK(fs->mtx, lk2);
				lk.unlock();
				// The buffers are invalid after callback so mark stale
				fs->stale = true;

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
				//LOG(INFO) << "NO FRAME FOUND: " << last_ts_ - latency_*mspf_;
				//latency_++;
				jobs_--;
			}
		//}

		//if (jobs_ == 0) LOG(INFO) << "LAST JOB =  Main";
		return true;
	});
}

void Group::addRawCallback(const std::function<void(ftl::rgbd::Source*, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)> &f) {
	for (auto s : sources_) {
		s->addRawCallback(f);
	}
}

void Group::removeRawCallback(const std::function<void(ftl::rgbd::Source*, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)> &f) {
	for (auto s : sources_) {
		s->removeRawCallback(f);
	}
}

static void mergeFrameset(ftl::rgbd::FrameSet &f1, ftl::rgbd::FrameSet &f2) {
	// Prepend all frame encodings in f2 into corresponding frame in f1.
	for (int i=0; i<f1.frames.size(); ++i) {
		f1.frames[i].mergeEncoding(f2.frames[i]);
	}
}

void Group::_recordStats(float fps, float latency) {
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

ftl::rgbd::FrameSet *Group::_findFrameset(int64_t ts) {
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
ftl::rgbd::FrameSet *Group::_getFrameset() {
	for (auto i=framesets_.begin(); i!=framesets_.end(); i++) {
		auto *f = *i;
		if (!f->stale && f->count == sources_.size()) {
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

void Group::_freeFrameset(ftl::rgbd::FrameSet *fs) {
	allocated_.push_back(fs);
}

ftl::rgbd::FrameSet *Group::_addFrameset(int64_t timestamp) {
	if (allocated_.size() == 0) {
		if (framesets_.size() < kMaxFramesets) {
			allocated_.push_back(new ftl::rgbd::FrameSet);
		} else {
			LOG(ERROR) << "Could not allocate frameset.";
			return nullptr;
		}
	}
	FrameSet *newf = allocated_.front();
	allocated_.pop_front();

	newf->timestamp = timestamp;
	newf->count = 0;
	newf->mask = 0;
	newf->stale = false;
	newf->frames.resize(sources_.size());

	for (auto &f : newf->frames) f.reset();

	if (newf->sources.size() != sources_.size()) {
		newf->sources.clear();
		for (auto s : sources_) newf->sources.push_back(s);
	}

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


	//int count = (framesets_[head_].timestamp == -1) ? 1 : (timestamp - framesets_[head_].timestamp) / mspf_;
	//LOG(INFO) << "Massive timestamp difference: " << count;

	// Allow for massive timestamp changes (Windows clock adjust)
	// Only add a single frameset for large changes
	//if (count < -int(kFrameBufferSize) || count >= kFrameBufferSize-1) {
	/*if (framesets_[head_].timestamp < timestamp) {
		head_ = (head_+1) % kFrameBufferSize;

		//if (framesets_[head_].stale == false) LOG(FATAL) << "Buffer exceeded";

		#ifdef DEBUG_MUTEX
		std::unique_lock<std::shared_timed_mutex> lk(framesets_[head_].mtx, std::defer_lock);
		#else
		std::unique_lock<std::shared_mutex> lk(framesets_[head_].mtx, std::defer_lock);
		#endif
		if (!lk.try_lock()) {
			LOG(ERROR) << "Frameset in use!!";
			return;
		}
		framesets_[head_].timestamp = timestamp;
		framesets_[head_].count = 0;
		framesets_[head_].mask = 0;
		framesets_[head_].stale = false;
		framesets_[head_].frames.resize(sources_.size());

		for (auto &f : framesets_[head_].frames) f.reset();

		if (framesets_[head_].sources.size() != sources_.size()) {
			framesets_[head_].sources.clear();
			for (auto s : sources_) framesets_[head_].sources.push_back(s);
		}

		// Find number of valid frames that will be skipped
		int count = 0;
		//for (int j=1; j<kFrameBufferSize; ++j) {
			int idx2 = (head_+kFrameBufferSize-1)%kFrameBufferSize;
			//if (framesets_[idx2].stale || framesets_[idx2].timestamp <= 0) break;
			++count;

			// Make sure encoded packets are moved from skipped frames
			//if (framesets_[idx2].count >= framesets_[idx2].sources.size()) {
			if (framesets_[idx2].stale == false) {
				mergeFrameset(framesets_[head_], framesets_[idx2]);
				framesets_[idx2].stale = true;
				framesets_[idx2].timestamp = -1;
			}
		//}

		float framerate = 1000.0f / float((count+1)*ftl::timer::getInterval());
		_recordStats(framerate, ftl::timer::get_time() - timestamp);

		return;
	} else {
		LOG(ERROR) << "Old frame received: " << (framesets_[head_].timestamp - timestamp);
		return;
	}*/
}

void Group::setName(const std::string &name) {
	name_ = name;
}


