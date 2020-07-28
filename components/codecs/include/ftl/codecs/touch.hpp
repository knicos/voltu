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

}
}

MSGPACK_ADD_ENUM(ftl::codecs::TouchType);

#endif
