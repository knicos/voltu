#include <ftl/profiler.hpp>
#include <ftl/timer.hpp>
#include <loguru.hpp>

using ftl::Profiler;

int Profiler::verb_ = 0;

Profiler::Profiler(const std::string &id, const std::string &label, double limit)
		: limit_(limit), label_(label) {

	stime_ = ftl::timer::get_time_seconds();
}

Profiler::~Profiler() {
	double etime = ftl::timer::get_time_seconds();

	if ((etime-stime_ > limit_ && verb_ > 0) || verb_ > 1) {
		LOG(INFO) << " -- Profiler --: " << label_ << " took " << (etime-stime_)*1000.0 << "ms";
	}
}
