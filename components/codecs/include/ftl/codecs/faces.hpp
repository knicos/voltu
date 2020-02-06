#ifndef _FTL_CODECS_FACE_HPP_
#define _FTL_CODECS_FACE_HPP_

#include <opencv2/core/mat.hpp>

#include <ftl/utility/msgpack.hpp>

namespace ftl {
namespace codecs {
struct Face {
	Face() {};
	//Face(const int &id, const cv::Vec3d &rvec, const cv::Vec3d &tvec) :
	//	id(id), rvec(rvec), tvec(tvec) {}

	int id;
	cv::Rect2d box;
    float depth;

	MSGPACK_DEFINE_ARRAY(id, box, depth);
};

}
}

#endif
