#pragma once
#ifndef _FTL_RGBD_FILE_SOURCE_HPP_
#define _FTL_RGBD_FILE_SOURCE_HPP_

#include <loguru.hpp>

#include <ftl/rgbd/source.hpp>
#include <ftl/codecs/reader.hpp>

namespace ftl {
namespace rgbd {
namespace detail {

class FileSource : public detail::Source {
	public:
	FileSource(ftl::rgbd::Source *, ftl::codecs::Reader *, int sid);
	~FileSource();

	bool capture(int64_t ts);
	bool retrieve();
	bool compute(int n, int b);
	bool isReady();

	//void reset();
	private:
	ftl::codecs::Reader *reader_;
};

}
}
}

#endif  // _FTL_RGBD_FILE_SOURCE_HPP_
