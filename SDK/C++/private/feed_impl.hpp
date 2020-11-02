#pragma once

#include <voltu/feed.hpp>
#include <ftl/streams/feed.hpp>
#include <unordered_set>

namespace voltu
{
namespace internal
{

class FeedImpl : public voltu::Feed
{
public:
	FeedImpl(ftl::stream::Feed*, uint32_t fsid);
	~FeedImpl();
	
	std::string getURI() override;

	void remove() override;

private:
	ftl::stream::Feed *feed_;
	std::unordered_set<uint32_t> fsids_;
};

}
}
