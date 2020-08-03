#ifndef _FTL_DATA_MESSAGES_HPP_
#define _FTL_DATA_MESSAGES_HPP_

#include <msgpack.hpp>

namespace ftl {
namespace data {

// Note: On Windows ERROR_* sometimes matches a macro and fails, hence use of Error not ERROR
enum class Message : int {
	Error_UNKNOWN = 0,
	Error_OPERATOR_EXCEPTION,
	Error_FRAME_GRAB,
	Error_BAD_FORMAT,
	Error_OPENVR,
	Error_RENDER,
	Warning_UNKNOWN = 1024,
	Warning_FRAME_DROP,
	Warning_PIPELINE_DROP,
	Warning_MISSING_CHANNEL,
	Warning_INCOMPLETE_FRAME,
	INFORMATION_UNKNOWN = 2046,
	OTHER_UNKNOWN = 3072
};

}
}

MSGPACK_ADD_ENUM(ftl::data::Message);

#endif