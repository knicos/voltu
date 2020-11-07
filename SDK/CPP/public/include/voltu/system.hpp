/**
 * @file system.hpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#pragma once
#include "defines.hpp"

#include <voltu/room.hpp>
#include <voltu/observer.hpp>
#include <voltu/feed.hpp>
#include <voltu/pipeline.hpp>
#include <list>
#include <string>

namespace voltu
{

/**
 * @brief Voltu semantic versioning information.
 */
struct Version
{
	int major;  ///< API Incompatible change
	int minor;	///< Possible binary incompatible, extensions
	int patch;	///< Internal only fixes.
};

/**
 * @brief Singleton Voltu system instance.
 * 
 * Provides access to the key components such as opening streams or files and
 * creating virtual cameras. Use `voltu::instance()` to obtain the object. All
 * object instances in VolTu are managed by shared smart pointers.
 */
class System
{
public:
	virtual ~System() = default;
	
	/**
	 * @brief Get the semantic version information.
	 * 
	 * @return Always returns semantic versioning structure.
	 */
	virtual voltu::Version getVersion() const = 0;

	/**
	 * @brief Make a virtual room or composite room.
	 * 
	 * @return A new virtual room instance.
	 */
	PY_API virtual voltu::RoomPtr createRoom() = 0;

	/**
	 * @brief Create a virtual observer.
	 * 
	 * An observer renderers virtual camera views, audio and other data from
	 * submitted framesets. It is possible and recommended that a single
	 * observer instance be used to renderer multiple different views, rather
	 * than creating lots of observers. This saves memory resources.
	 * 
	 * @return A new observer instance.
	 */
	PY_API virtual voltu::ObserverPtr createObserver() = 0;

	/**
	 * @brief Open a file, device or network stream using a URI.
	 * 
	 * All data sources in VolTu are represented by Universal Resource
	 * Identifiers (URIs), with some non-standard additions. A few examples
	 * are:
	 * * `file:///home/user/file.ftl`
	 * * `tcp://localhost:9001/*`
	 * * `ftl://my.stream.name/room1`
	 * * `ws://ftlab.utu.fi/lab/`
	 * * `./file.ftl`
	 * * `device:camera`
	 * * `device:screen`
	 * 
	 * Note that returning from this call does not guarantee that the source
	 * is fully open and operational, this depends on network handshakes or
	 * file processing that occurs asynchronously.
	 * 
	 * @throw voltu::exceptions::BadSourceURI If an unrecognised URI is given.
	 * @return A feed management object for the data source.
	 */
	PY_API virtual voltu::FeedPtr open(const std::string &uri) = 0;

	/**
	 * @brief Get a list of all available rooms.
	 * 
	 * A room is a 3D captured physical space, or a combination of such spaces,
	 * and is represented by a unique identifier within the local system. This
	 * method obtains a list of all available rooms from all sources. To obtain
	 * rooms, either use `open` or `createRoom`.
	 * 
	 * @return A list of room ids, which can be empty.
	 * 
	 * @see getRoom
	 * @see open
	 */
	PY_API virtual std::list<voltu::RoomId> listRooms() = 0;

	/**
	 * @brief Get a room instance from identifier.
	 * 
	 * A room instance enables access to all data frames associated with that
	 * room, including image data. Calling `getRoom` with the same ID
	 * multiple times will return different smart pointers to room instances
	 * but provides access to the same data regardless and is valid. An invalid
	 * room ID will throw an exception.
	 * 
	 * @throw voltu::exceptions::InvalidRoomId If the ID does not exist.
	 * @return Room instance or accessing room data.
	 */
	PY_API virtual voltu::RoomPtr getRoom(voltu::RoomId room) = 0;

	/** Make a file or streaming feed, to which you can send frames. */
	PY_API virtual voltu::FeedPtr createFeed(const std::string &uri) = 0;

	/**
	 * @brief Make an empty operator pipeline for frame processing.
	 * 
	 * A pipeline allows a sequence of processing operations to be applied to
	 * a data frame. These operations include stereo correspondence, fusion,
	 * data evaluation and various image processing operations. Only some
	 * of these operators are exposed in the SDK. Once a pipeline instance
	 * is obtained, you can add specific operators to it, configure them and
	 * then submit frames from processing.
	 * 
	 * @return A unique pipeline instance.
	 */
	PY_API virtual voltu::PipelinePtr createPipeline() = 0;
};

}  // namespace voltu
