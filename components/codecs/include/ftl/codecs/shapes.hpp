/**
 * @file shapes.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

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
	ARUCO,
	CURSOR
};

/**
 * Shape data item for Shapes3D channel. Used for various tracking, bounding and
 * general 3D positional data.
 */
struct Shape3D {
	int id;
	Shape3DType type;
	Eigen::Vector3f size;
    Eigen::Matrix4f pose;
	std::string label;

	MSGPACK_DEFINE_ARRAY(id, type, size, pose, label);
};

}  // namespace codecs
}  // namespace ftl

MSGPACK_ADD_ENUM(ftl::codecs::Shape3DType);

#endif
