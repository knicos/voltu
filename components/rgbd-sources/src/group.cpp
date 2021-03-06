#include <ftl/rgbd/group.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/timer.hpp>
#include <ftl/operators/operator.hpp>
#include <ftl/profiler.hpp>

#include <chrono>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::rgbd::Group;
using ftl::rgbd::Source;
using ftl::rgbd::kMaxFramesets;
using std::vector;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;
using ftl::codecs::Channel;
using ftl::codecs::Channels;

Group::Group() : pipeline_(nullptr) {
	jobs_ = 0;
	cjobs_ = 0;
	skip_ = false;
	name_ = "NoName";

	builder_.setBufferSize(0);
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

void Group::addSource(ftl::rgbd::Source *src) {
	UNIQUE_LOCK(mutex_, lk);
	size_t ix = sources_.size();
	sources_.push_back(src);

	src->setCallback([this,ix,src](int64_t timestamp, ftl::rgbd::Frame &frame) {
		// FIXME: Not safe for multiple sources
		if (pipeline_) {
			FTL_Profile("GroupPipe", 0.03);
			pipeline_->apply(frame, frame, 0);
		}

		frame.swapTo(Channels<0>::All(), builder_.get(timestamp,ix));
		builder_.completed(timestamp, ix);
		//builder_.push(timestamp, ix, frame);
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

void Group::_dispatchJob(ftl::rgbd::Source *src, int64_t ts) {
	try {
		src->dispatch(ts);
	} catch (std::exception &ex) {
		LOG(ERROR) << "Exception when dispatching frame";
		LOG(ERROR) << ex.what();
	}
	catch (...) {
		LOG(ERROR) << "Unknown exception when dispatching frame";
	}
}

int Group::streamID(const ftl::rgbd::Source *s) const {
	for (size_t i=0; i<sources_.size(); ++i) {
		if (sources_[i] == s) return i;
	}
	return -1;
}

void Group::onFrameSet(const ftl::rgbd::VideoCallback &cb) {
	// 1. Capture camera frames with high precision
	cap_id_ = ftl::timer::add(ftl::timer::kTimerHighPrecision, [this](int64_t ts) {
		skip_ = jobs_ != 0;  // Last frame not finished so skip all steps

		if (skip_) return true;

		for (auto s : sources_) {
			skip_ &= s->capture(ts);
		}

		return true;
	});

	// 2. Issue IO retrieve ad compute jobs before finding a valid
	// frame at required latency to pass to callback.
	main_id_ = ftl::timer::add(ftl::timer::kTimerMain, [this,cb](int64_t ts) {
		if (skip_) return true;

		for (auto s : sources_) {
			jobs_++;

			//ftl::pool.push([this,s,ts](int id) {
				_retrieveJob(s);
				//LOG(INFO) << "Retrieve latency: " << ftl::timer::get_time()-ts;
				--jobs_;
				_dispatchJob(s, ts);
			//});
		}
		return true;
	});

	builder_.onFrameSet([this,cb](ftl::rgbd::FrameSet &fs) {
		cb(fs);
		return true;
	});
}

void Group::setName(const std::string &name) {
	name_ = name;
	builder_.setName(name);
}


