#include <ftl/data/new_frameset.hpp>
#include <ftl/data/framepool.hpp>

using ftl::data::Frame;
using ftl::data::FrameSet;

FrameSet::FrameSet(Pool *ppool, FrameID pid, int64_t ts, size_t psize) :
	Frame(ppool->allocate(FrameID(pid.frameset(),255), ts)), mask(0) {
	
	flush_count = 0; // Reset flush on store...
	frames.reserve(psize);
}

FrameSet::~FrameSet() {
	if (status() == ftl::data::FrameStatus::CREATED) store();
	if (status() == ftl::data::FrameStatus::STORED) flush();
}

void ftl::data::FrameSet::completed(size_t ix) {
	if (ix == 255) {

	} else if (ix < frames.size()) {
		// If already completed for given frame, then skip
		if (mask & (1 << ix)) return;

		mask |= (1 << ix);
		++count;
	} else {
		throw FTL_Error("Completing frame that does not exist: " << timestamp() << ":" << ix);
	}
}

void ftl::data::FrameSet::changeTimestamp(int64_t ts) {
	timestamp_ = ts;
	for (auto &f : frames) {
		f.timestamp_ = ts;
	}
}

void ftl::data::FrameSet::resize(size_t s) {
	while (frames.size() < s) {
		frames.push_back(std::move(pool()->allocate(FrameID(frameset(), frames.size()), timestamp())));
	}
	while (frames.size() > s) frames.pop_back();
}

void ftl::data::FrameSet::moveTo(ftl::data::FrameSet &fs) {
	UNIQUE_LOCK(fs.mutex(), lk);
	Frame::moveTo(fs);
	fs.count = static_cast<int>(count);
	fs.flags_ = (int)flags_;
	fs.mask = static_cast<unsigned int>(mask);
	fs.frames = std::move(frames);

	count = 0;
	mask = 0;
	set(ftl::data::FSFlag::STALE);
}

ftl::data::Frame &ftl::data::FrameSet::firstFrame() {
	for (size_t i=0; i<frames.size(); ++i) {
		if (hasFrame(i)) return frames[i];
	}
	throw FTL_Error("No frames in frameset");
}

const ftl::data::Frame &ftl::data::FrameSet::firstFrame() const {
	for (size_t i=0; i<frames.size(); ++i) {
		if (hasFrame(i)) return frames[i];
	}
	throw FTL_Error("No frames in frameset");
}

bool ftl::data::FrameSet::hasAnyChanged(ftl::codecs::Channel c) const {
	for (size_t i=0; i<frames.size(); ++i) {
		if (frames[i].changed(c)) return true;
	}
	return false;
}

void FrameSet::store() {
	if (status() != ftl::data::FrameStatus::CREATED) throw FTL_Error("Cannot store frameset multiple times");

	{
		//UNIQUE_LOCK(smtx, lk);
		for (auto &f : frames) if (f.status() == ftl::data::FrameStatus::CREATED) f.store();
		ftl::data::Frame::store();
	}
}

void FrameSet::flush() {
	if (status() == ftl::data::FrameStatus::FLUSHED) throw FTL_Error("Cannot flush frameset multiple times");

	// Build list of all changed but unflushed channels.
	std::unordered_set<ftl::codecs::Channel> unflushed;

	{
		UNIQUE_LOCK(smtx, lk);
		for (auto &f : frames) {
			for (auto &c : f.changed()) {
				if (!f.flushed(c.first)) {
					unflushed.emplace(c.first);
				}
			}
		}

		for (auto &f : frames) if (f.status() == ftl::data::FrameStatus::STORED) f.flush();
		ftl::data::Frame::flush();
	}

	for (auto c : unflushed) {
		pool()->flush_fs_.trigger(*this, c);
	}
}

void FrameSet::flush(ftl::codecs::Channel c) {
	{
		UNIQUE_LOCK(smtx, lk);
		for (auto &f : frames) if (f.hasOwn(c)) f.flush(c);
	}
	
	pool()->flush_fs_.trigger(*this, c);
}

/**
 * Make a frameset from a single frame. It borrows the pool, id and
 * timestamp from the frame and creates a wrapping frameset instance.
 */
std::shared_ptr<FrameSet> FrameSet::fromFrame(Frame &f) {
	auto sptr = std::make_shared<FrameSet>(f.pool(), f.id(), f.timestamp());
	sptr->frames.push_back(std::move(f));
	sptr->count = 1;
	sptr->mask = 1;
	return sptr;
}

std::unordered_set<ftl::codecs::Channel> FrameSet::channels() {
	std::unordered_set<ftl::codecs::Channel> res{};
	for (auto& f : frames) {
		auto c = f.channels();
		res.insert(c.begin(), c.end());
	}
	return res;
}
