#include <voltu/room.hpp>
#include "room_impl.hpp"
#include "frame_impl.hpp"
#include <voltu/types/errors.hpp>

using voltu::internal::RoomImpl;

RoomImpl::RoomImpl(ftl::stream::Feed* feed)
 : feed_(feed)
{

}

RoomImpl::~RoomImpl()
{
	if (filter_) filter_->remove();
}

bool RoomImpl::waitNextFrame(int64_t timeout, bool except)
{
	if (!filter_)
	{
		filter_ = feed_->filter(fsids_, {ftl::codecs::Channel::Colour, ftl::codecs::Channel::Depth, ftl::codecs::Channel::GroundTruth});
		filter_->on([this](const std::shared_ptr<ftl::data::FrameSet> &fs)
		{
			UNIQUE_LOCK(mutex_, lk);
			last_seen_ = std::max(last_seen_, fs->timestamp());
			cv_.notify_all();
			return true;
		});
	}

	std::unique_lock<std::mutex> lk(mutex_);

	if (last_read_ >= last_seen_)
	{
		if (timeout > 0)
		{
			cv_.wait_for(lk, std::chrono::milliseconds(timeout), [this] {
				return last_read_ < last_seen_;
			});

			if (except && last_read_ >= last_seen_)
			{
				throw voltu::exceptions::Timeout();
			}
			return last_read_ < last_seen_;
		}
		else if (timeout == 0)
		{
			if (except) throw voltu::exceptions::Timeout();
			return false;
		}
		else
		{
			cv_.wait(lk, [this] {
				return last_read_ < last_seen_;
			});
		}
	}

	return true;
}

voltu::FramePtr RoomImpl::getFrame()
{
	auto f = std::make_shared<voltu::internal::FrameImpl>();

	std::unique_lock<std::mutex> lk(mutex_);
	int count = 0;

	for (auto fsid : fsids_)
	{
		auto fs = feed_->getFrameSet(fsid);
		if (!fs) continue;

		f->pushFrameSet(fs);
		last_read_ = std::max(last_read_, fs->timestamp());
		++count;
	}

	if (count == 0) throw voltu::exceptions::NoFrame();

	return f;
}

std::string RoomImpl::getName()
{
	return "";
}

void RoomImpl::addFrameSet(uint32_t fsid)
{
	fsids_.insert(fsid);
}

bool RoomImpl::active()
{
	return ftl::running;
}
