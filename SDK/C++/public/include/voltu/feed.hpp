#pragma once

#include "defines.hpp"

#include <memory>
#include <string>

namespace voltu
{

enum class FeedType
{
	kInvalid = 0,
	kMonoCamera = 1,
	kStereoCamera = 2,
	kDepthCamera = 3,
	kVirtual = 4,
	kScreen = 5,
	kRoom = 6,
	kRooms = 7
};

class Feed
{
public:
	PY_API virtual std::string getURI() = 0;

	PY_API virtual void remove() = 0;

	// Get rooms
};

typedef std::shared_ptr<voltu::Feed> FeedPtr;

}
