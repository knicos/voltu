#ifndef _FTL_CODECS_SHAPES_HPP_
#define _FTL_CODECS_SHAPES_HPP_

#include <opencv2/core/mat.hpp>

#include <ftl/utility/msgpack.hpp>

namespace ftl {
namespace codecs {

enum class Shape3DType {
	UNKNOWN = 0,
	BOX,
	SPHERE,
	STAR,
	HEAD,
	CLIPPING,
	CAMERA,
	FEATURE,
	ARUCO
};

struct Shape3D {
	int id;
	Shape3DType type;
	Eigen::Vector3f size;
    Eigen::Matrix4f pose;
	std::string label;

	MSGPACK_DEFINE_ARRAY(id, type, size, pose, label);
};

}
}

MSGPACK_ADD_ENUM(ftl::codecs::Shape3DType);

#endif
