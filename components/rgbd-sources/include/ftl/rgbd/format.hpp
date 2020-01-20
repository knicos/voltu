#ifndef _FTL_RGBD_FORMAT_HPP_
#define _FTL_RGBD_FORMAT_HPP_

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <ftl/cuda_util.hpp>
#include <ftl/codecs/codecs.hpp>
#include <ftl/traits.hpp>

namespace ftl {
namespace rgbd {

struct FormatBase {
	FormatBase(size_t w, size_t h, int t) : width(w), height(h), cvType(t) {}

	size_t width;		// Width in pixels
	size_t height;		// Height in pixels
	int cvType;			// OpenCV Mat type

	inline bool empty() const { return width == 0 || height == 0; }
	inline cv::Size size() const { return cv::Size(width, height); }
};

template <typename T>
struct Format : public ftl::rgbd::FormatBase {
	Format() : FormatBase(0,0,0) {}

	Format(size_t w, size_t h) : FormatBase(
			w, h, ftl::traits::OpenCVType<T>::value) {}

	explicit Format(ftl::codecs::definition_t d) : FormatBase(
			ftl::codecs::getWidth(d),
			ftl::codecs::getHeight(d),
			ftl::traits::OpenCVType<T>::value) {}

	explicit Format(const cv::Size &s) : FormatBase(
			s.width,
			s.height,
			ftl::traits::OpenCVType<T>::value) {}

	explicit Format(const cv::InputArray &a) : FormatBase(
			a.cols,
			a.rows,
			ftl::traits::OpenCVType<T>::value) {
		CHECK(cvType == a.type());
	}
};

}
}

#endif  // _FTL_RGBD_FORMAT_HPP_
