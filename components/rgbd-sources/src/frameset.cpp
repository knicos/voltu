#include <ftl/rgbd/frameset.hpp>

using ftl::rgbd::FrameSet;
using ftl::rgbd::Channels;
using ftl::rgbd::Channel;

void FrameSet::upload(ftl::rgbd::Channels c, cudaStream_t stream) {
	for (auto &f : frames) {
		f.upload(c, stream);
	}
}

void FrameSet::download(ftl::rgbd::Channels c, cudaStream_t stream) {
	for (auto &f : frames) {
		f.download(c, stream);
	}
}

void FrameSet::swapTo(ftl::rgbd::FrameSet &fs) {
	UNIQUE_LOCK(fs.mtx, lk);

	if (fs.frames.size() != frames.size()) {
		// Assume "this" is correct and "fs" is not.
		fs.sources.clear();
		for (auto s : sources) fs.sources.push_back(s);
		fs.frames.resize(frames.size());
	}

	fs.timestamp = timestamp;
	fs.count = count;
	fs.stale = stale;
	fs.mask = mask;

	for (size_t i=0; i<frames.size(); ++i) {
		frames[i].swap(Channels::All(), fs.frames[i]);
	}

	stale = true;
}
