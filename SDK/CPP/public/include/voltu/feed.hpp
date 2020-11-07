/**
 * @file feed.hpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#pragma once

#include "defines.hpp"
#include <voltu/types/frame.hpp>
#include <voltu/types/property.hpp>

#include <memory>
#include <string>

namespace voltu
{

enum class FeedType
{
	kInvalid = 0,
	kDevice = 1,
	kFile = 2,
	kStream = 3,
	kVirtual = 4,
	kMultiple = 5
};

enum class FeedProperty
{
	kInvalid			= 0,
	kColourCodec		= 2001,
	kDepthCodec			= 2002,
	kFPSLimit			= 2003,
	kColourBitrate		= 2004,
	kDepthBitrate		= 2005,
	kFileLooping		= 2006,
	kFileSpeed			= 2007
};

class Feed
{
public:
	virtual ~Feed() = default;
	
	PY_API virtual std::string getURI() = 0;

	PY_API virtual void remove() = 0;

	PY_API virtual void submit(const voltu::FramePtr &frame) = 0;

	PY_API virtual voltu::FeedType type() = 0;

	PY_API virtual voltu::PropertyPtr property(voltu::FeedProperty) = 0;

	// Get rooms
};

typedef std::shared_ptr<voltu::Feed> FeedPtr;

}
