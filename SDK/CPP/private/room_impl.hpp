#pragma once

#include <ftl/streams/feed.hpp>
#include <ftl/threads.hpp>
#include <condition_variable>

namespace voltu
{
namespace internal
{

class RoomImpl : public voltu::Room
{
public:
	explicit RoomImpl(ftl::stream::Feed*);
	
	~RoomImpl() override;

	bool waitNextFrame(int64_t, bool except) override;

	voltu::FramePtr getFrame() override;

	std::string getName() override;

	bool active() override;

	void addFrameSet(uint32_t fsid);

private:
	ftl::stream::Feed* feed_;
	std::unordered_set<uint32_t> fsids_;
	ftl::stream::Feed::Filter* filter_=nullptr;
	MUTEX mutex_;
	std::condition_variable cv_;
	int64_t last_seen_ = -1;
	int64_t last_read_ = -1;
};

}
}
