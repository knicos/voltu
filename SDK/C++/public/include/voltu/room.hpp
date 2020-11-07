/**
 * @file room.hpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#pragma once

#include "defines.hpp"
#include <voltu/types/frame.hpp>
#include <memory>

namespace voltu
{

enum class RoomType
{
	kInvalid = 0,
	kPhysical = 1,
	kComposite = 2
};

typedef unsigned int RoomId;

class Room
{
public:
	virtual ~Room() = default;
	
	PY_API virtual bool waitNextFrame(int64_t) = 0;

	PY_API inline bool hasNextFrame() { return waitNextFrame(0); };

	PY_API virtual voltu::FramePtr getFrame() = 0;

	PY_API virtual std::string getName() = 0;

	PY_API virtual bool active() = 0;
};

typedef std::shared_ptr<Room> RoomPtr;

}
