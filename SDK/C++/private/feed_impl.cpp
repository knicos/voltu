#include "feed_impl.hpp"

using voltu::internal::FeedImpl;

FeedImpl::FeedImpl(ftl::stream::Feed* feed, uint32_t fsid)
 : feed_(feed)
{
	fsids_.insert(fsid);
}

FeedImpl::~FeedImpl()
{
	remove();
}
	
std::string FeedImpl::getURI()
{
	return feed_->getURI(*fsids_.begin());
}

void FeedImpl::remove()
{

}

