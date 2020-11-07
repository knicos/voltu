/**
 * @file room.hpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#pragma once

#include "defines.hpp"
#include <voltu/types/frame.hpp>
#include <memory>
#include <string>

namespace voltu
{

/**
 * @brief Room classification.
 */
enum class RoomType
{
	kInvalid = 0,
	kPhysical = 1,	///< Physically captured space
	kComposite = 2	///< Virtual combination of other rooms
};

/**
 * @brief Unique room identifier.
 */
typedef unsigned int RoomId;

/**
 * @brief Volumetric captured room data.
 * 
 * An instance of this class provides access to discrete data frames for a
 * volumetrically captured room, or combination of rooms. The room data is
 * made available in discrete timestamped frames, but only the most recent
 * frame is kept inside the room class. Blocking and polling functions can be
 * used to wait for new frames, or the most recent frame can always be accessed.
 * If you wish to keep a history of frames, it will be up to you to maintain a
 * copy of the frame objects obtained from the room instance as they arrive.
 * 
 * For composite virtual rooms, a frame constitutes all data from all sub-rooms.
 * Timestamps for these sub-rooms may not match each other and are not expected
 * to be synchronised.
 * 
 * @todo A callback option for receiving new frames.
 * @see voltu::Frame
 */
class Room
{
public:
	virtual ~Room() = default;
	
	/**
	 * @brief Allow blocking until a new frame is received.
	 * 
	 * Depending upon the timeout value, this function does the following:
	 * 1) If `timeout` = 0 then it returns immediately.
	 * 2) If `timeout` < 0 then it blocks without timeout.
	 * 3) If `timeout` > 0 then it blocks for maximum `timeout` milliseconds.
	 * 
	 * Note that for composite rooms with multiple physical rooms, a new
	 * frame occurs whenever any of the physical rooms provides a new frame,
	 * even if other rooms do not.
	 * 
	 * @todo Allow option to wait for new frames from all sub-rooms.
	 * 
	 * @param timeout Millisecond timeout, or 0 or -1.
	 * @return True if a new unseen frame is available.
	 */
	PY_API virtual bool waitNextFrame(int64_t timeout) = 0;

	/**
	 * @brief Check if a new frame is available.
	 * 
	 * Equivalent to `waitNextFrame(0)`.
	 * 
	 * @return True if a new unseen frame is available.
	 */
	PY_API inline bool hasNextFrame() { return waitNextFrame(0); };

	/**
	 * @brief Retrieve the most recently available frame.
	 * 
	 * This method can be called any number of times and may return the same
	 * frame data if no new data has arrived between calls. Calling this once
	 * marks the data as seen, and therefore causes a subsequent call to
	 * `waitNextFrame` to block until more data arrives.
	 * 
	 * Each call to `getFrame` can return a different smart pointer for the same
	 * frame data, this is valid. The room object must remain in existence for
	 * as long as any frame objects are held.
	 * 
	 * @throw voltu::exceptions::NoFrame If no frames have yet been received.
	 * @return Timestamped frame data instance.
	 */
	PY_API virtual voltu::FramePtr getFrame() = 0;

	/**
	 * @brief Get a human readable room name.
	 * @return A room name string.
	 */
	PY_API virtual std::string getName() = 0;

	/**
	 * @brief Check if the room is actively receiving data.
	 * 
	 * It is possible that the underlying data source for a room finished or
	 * is otherwise terminated, in which case the room is marked inactive.
	 * 
	 * @return True if room is expected to continue receiving data.
	 */
	PY_API virtual bool active() = 0;
};

typedef std::shared_ptr<Room> RoomPtr;

}
