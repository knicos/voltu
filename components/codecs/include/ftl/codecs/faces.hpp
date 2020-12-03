/**
 * @file faces.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_CODECS_FACE_HPP_
#define _FTL_CODECS_FACE_HPP_

#include <opencv2/core/mat.hpp>

#include <ftl/utility/msgpack.hpp>

namespace ftl {
namespace codecs {

/** Face data item for Faces channel. */
struct Face {
	Face() {};

	int id;
	cv::Rect2d box;
    float depth;

	MSGPACK_DEFINE_ARRAY(id, box, depth);
};

}  // namespace codecs
}  // namespace ftl

#endif  // _FTL_CODECS_FACE_HPP_
