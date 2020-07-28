#include <ftl/data/creators.hpp>
#include <ftl/data/framepool.hpp>
#include <ftl/timer.hpp>

#include <loguru.hpp>

using ftl::data::Frame;
using ftl::data::Pool;
using ftl::data::FrameCreator;
using ftl::data::IntervalFrameCreator;

Frame FrameCreator::create() {
	Frame f = pool_->allocate(id_, ftl::timer::get_time());
	return f;
}

Frame FrameCreator::create(int64_t timestamp) {
	Frame f = pool_->allocate(id_, timestamp);
	return f;
}

IntervalFrameCreator::IntervalFrameCreator(Pool *p_pool, FrameID p_id, DiscreteSource *src)
	: FrameCreator(p_pool, p_id), src_(src) {}

void IntervalFrameCreator::start() {
	capture_ = std::move(ftl::timer::add(ftl::timer::timerlevel_t::kTimerHighPrecision, [this](int64_t ts) {
		src_->capture(ts);
		return true;
	}));

	retrieve_ = std::move(ftl::timer::add(ftl::timer::timerlevel_t::kTimerMain, [this](int64_t ts) {
		Frame f = create(ts);
		f.store();
		if (!src_->retrieve(f)) {
			LOG(WARNING) << "Frame was skipping";
		}
		return true;
	}));
}

void IntervalFrameCreator::stop() {
	capture_.cancel();
	retrieve_.cancel();
}
