#pragma once

#include <voltu/feed.hpp>
#include <ftl/streams/feed.hpp>
#include <unordered_set>

namespace voltu
{
namespace internal
{

class InputFeedImpl : public voltu::Feed
{
public:
	InputFeedImpl(ftl::stream::Feed*, uint32_t fsid);
	~InputFeedImpl() override;
	
	std::string getURI() override;

	void remove() override;

	void submit(const voltu::FramePtr &frame) override;

	voltu::FeedType type() override;

	voltu::PropertyPtr property(voltu::FeedProperty) override;

private:
	ftl::stream::Feed *feed_;
	std::unordered_set<uint32_t> fsids_;
};

class OutputFeedImpl : public voltu::Feed
{
public:
	OutputFeedImpl(ftl::stream::Feed*, const std::string &uri);
	~OutputFeedImpl() override;
	
	std::string getURI() override;

	void remove() override;

	void submit(const voltu::FramePtr &frame) override;

	voltu::FeedType type() override;

	voltu::PropertyPtr property(voltu::FeedProperty) override;

private:
	ftl::stream::Feed *feed_;
	std::string uri_;
};

}
}
