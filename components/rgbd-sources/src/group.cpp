#include <ftl/rgbd/group.hpp>
#include <ftl/rgbd/source.hpp>

using ftl::rgbd::Group;
using ftl::rgbd::Source;
using ftl::rgbd::kFrameBufferSize;
using std::vector;

Group::Group() : framesets_(kFrameBufferSize), head_(0) {
	framesets_[0].timestamp = -1;
}

Group::~Group() {

}

void Group::addSource(ftl::rgbd::Source *src) {
	UNIQUE_LOCK(mutex_, lk);
	size_t ix = sources_.size();
	sources_.push_back(src);

	src->setCallback([this,ix](int64_t timestamp, const cv::Mat &rgb, const cv::Mat &depth) {
		if (timestamp == 0) return;
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
				//LOG(INFO) << "Adding frame: " << ix << " for " << timestamp;
				rgb.copyTo(fs.channel1[ix]);
				depth.copyTo(fs.channel2[ix]);
				++fs.count;
				fs.mask |= (1 << ix);

				if (callback_ && fs.count == sources_.size()) {
					//LOG(INFO) << "DOING CALLBACK";
					if (callback_(fs)) {
						//sources_[ix]->grab();
						//LOG(INFO) << "GRAB";
					}
				}

				return;
			}
		}
		LOG(WARNING) << "Frame timestamp not found in buffer";
	});
}

// TODO: This should be a callback
// Callback returns true if it wishes to continue receiving frames.
void Group::sync(int N, int B) {
	for (auto s : sources_) {
		s->grab(N,B);
	}
}

void Group::sync(std::function<bool(const ftl::rgbd::FrameSet &)> cb) {
	callback_ = cb;
	sync(-1,-1);
}

bool Group::getFrames(ftl::rgbd::FrameSet &fs, bool complete) {
	// Use oldest frameset or search back until first complete set is found?
	if (complete) {
		UNIQUE_LOCK(mutex_, lk);
		// Search backwards to find match
		for (size_t i=0; i<kFrameBufferSize; ++i) {
			FrameSet &f = framesets_[(head_+kFrameBufferSize-i) % kFrameBufferSize];
			if (f.count == sources_.size()) {
				LOG(INFO) << "Complete set found";
				fs = f;  // FIXME: This needs to move or copy safely...
				return true;
			}
		}
		LOG(WARNING) << "No complete frame set found";
		return false;
	}

	return false;
}

void Group::_addFrameset(int64_t timestamp) {
	int count = (framesets_[head_].timestamp == -1) ? 1 : (timestamp - framesets_[head_].timestamp) / 40;
	// Must make sure to also insert missing framesets
	//LOG(INFO) << "Adding " << count << " framesets for " << timestamp << " head=" << framesets_[head_].timestamp;

	//if (count > 10 || count < 1) return;

	for (int i=0; i<count; ++i) {
		int64_t lt = (framesets_[head_].timestamp == -1) ? timestamp-40 : framesets_[head_].timestamp;
		head_ = (head_+1) % kFrameBufferSize;
		framesets_[head_].timestamp = lt+40;
		framesets_[head_].count = 0;
		framesets_[head_].mask = 0;
		framesets_[head_].channel1.resize(sources_.size());
		framesets_[head_].channel2.resize(sources_.size());

		for (auto s : sources_) framesets_[head_].sources.push_back(s);
	}
}


