/**
 * @file profiler.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_PROFILER_HPP_
#define _FTL_PROFILER_HPP_

#include <ftl/config.h>
#include <string>

namespace ftl {

/** RAII Profile object for easy internal timing of statement blocks. */
class Profiler {
	public:
	Profiler(const std::string &id, const std::string &label, double limit);
	~Profiler();

	static void verbosity(int v) { verb_ = v; }

	private:
	double stime_;
	double limit_;
	std::string label_;

	static int verb_;
};

}

#ifdef ENABLE_PROFILER
#define FTL_Profile(LABEL, LIMIT) ftl::Profiler __profile(__func__, LABEL, LIMIT)
#else
#define FTL_Profile(LABEL, LIMIT)
#endif

#endif  // _FTL_PROFILER_HPP_
