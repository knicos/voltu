#pragma once
#include "defines.hpp"

#include <voltu/room.hpp>
#include <voltu/observer.hpp>
#include <voltu/feed.hpp>
#include <list>

namespace voltu
{

struct Version
{
	int major;
	int minor;
	int patch;
};

class System
{
public:
	virtual voltu::Version getVersion() const = 0;

	PY_API virtual voltu::RoomPtr createRoom() = 0;

	PY_API virtual voltu::ObserverPtr createObserver() = 0;

	PY_API virtual voltu::FeedPtr open(const std::string&) = 0;

	PY_API virtual std::list<voltu::RoomId> listRooms() = 0;

	PY_API virtual voltu::RoomPtr getRoom(voltu::RoomId) = 0;
};

}
