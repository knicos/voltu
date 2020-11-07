#pragma once
#include "defines.hpp"

#include <voltu/room.hpp>
#include <voltu/observer.hpp>
#include <voltu/feed.hpp>
#include <voltu/pipeline.hpp>
#include <list>

namespace voltu
{

struct Version
{
	int major;  // API Incompatible change
	int minor;	// Possible binary incompatible, extensions
	int patch;	// Internal only fixes.
};

/**
 * Singleton Voltu system instance. Provides access to the key components such
 * as opening streams or files and creating virtual cameras.
 */
class System
{
public:
	virtual ~System() = default;
	
	/** Get the semantic version information. */
	virtual voltu::Version getVersion() const = 0;

	/** Make a virtual room or composite room. */
	PY_API virtual voltu::RoomPtr createRoom() = 0;

	/** Create a virtual observer. This renderers virtual camera views. */
	PY_API virtual voltu::ObserverPtr createObserver() = 0;

	/** Open a file, device or network stream using a URI. */
	PY_API virtual voltu::FeedPtr open(const std::string &uri) = 0;

	PY_API virtual std::list<voltu::RoomId> listRooms() = 0;

	PY_API virtual voltu::RoomPtr getRoom(voltu::RoomId room) = 0;

	/** Make a file or streaming feed, to which you can send frames. */
	PY_API virtual voltu::FeedPtr createFeed(const std::string &uri) = 0;

	/** Make an empty operator pipeline for frame processing. */
	PY_API virtual voltu::PipelinePtr createPipeline() = 0;
};

}
