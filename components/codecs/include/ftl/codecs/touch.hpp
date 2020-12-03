/**
 * @file touch.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_CODECS_TOUCH_HPP_
#define _FTL_CODECS_TOUCH_HPP_

#include <ftl/utility/msgpack.hpp>

namespace ftl {
namespace codecs {

enum class TouchType {
	MOUSE_LEFT=0,
	MOUSE_RIGHT=1,
	MOUSE_MIDDLE=2,
	TOUCH_SCREEN=3,
	COLLISION=16
};

/**
 * Data item for touch channel. Represents screen touch or 3D collision data.
 * Currently experimental and incomplete.
 */
struct Touch {
	Touch() {};

	int id;
	TouchType type;
	uint8_t strength;
	int x;
	int y;
	float d;

	MSGPACK_DEFINE(id, type, strength, x, y, d);
};

}  // namespace codecs
}  // namespace ftl

MSGPACK_ADD_ENUM(ftl::codecs::TouchType);

#endif
