#pragma once

#include <voltu/system.hpp>

#include <ftl/streams/feed.hpp>
#include <ftl/net/universe.hpp>

namespace voltu
{
namespace internal
{

class SystemImpl : public voltu::System
{
public:
	SystemImpl();
	~SystemImpl() override;

	voltu::Version getVersion() const override;

	voltu::RoomPtr createRoom() override { return nullptr; };

	voltu::ObserverPtr createObserver() override;

	voltu::FeedPtr open(const std::string&) override;

	std::list<voltu::RoomId> listRooms() override;

	voltu::RoomPtr getRoom(voltu::RoomId) override;

	voltu::FeedPtr createFeed(const std::string &uri) override;

	voltu::PipelinePtr createPipeline() override;

private:
	ftl::Configurable* root_;
	ftl::stream::Feed* feed_;
	ftl::net::Universe* net_;
};

}  // namespace internal
}  // namespace voltu
