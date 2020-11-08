#include "feed_impl.hpp"
#include <voltu/types/errors.hpp>

using voltu::internal::InputFeedImpl;
using voltu::internal::OutputFeedImpl;

// ==== Input ==================================================================

InputFeedImpl::InputFeedImpl(ftl::stream::Feed* feed, uint32_t fsid)
 : feed_(feed)
{
	fsids_.insert(fsid);
}

InputFeedImpl::~InputFeedImpl()
{
	//remove();
}
	
std::string InputFeedImpl::getURI()
{
	return feed_->getURI(*fsids_.begin());
}

void InputFeedImpl::remove()
{
	throw voltu::exceptions::NotImplemented();
}

void InputFeedImpl::submit(const voltu::FramePtr &frame)
{
	throw voltu::exceptions::ReadOnly();
}

voltu::FeedType InputFeedImpl::type()
{
	throw voltu::exceptions::NotImplemented();
}

voltu::PropertyPtr InputFeedImpl::property(voltu::FeedProperty)
{
	throw voltu::exceptions::NotImplemented();
}

// ==== Output =================================================================

OutputFeedImpl::OutputFeedImpl(ftl::stream::Feed* feed, const std::string &uri)
 : feed_(feed), uri_(uri)
{

}

OutputFeedImpl::~OutputFeedImpl()
{
	//remove();
}
	
std::string OutputFeedImpl::getURI()
{
	return uri_;
}

void OutputFeedImpl::remove()
{
	throw voltu::exceptions::NotImplemented();
}

void OutputFeedImpl::submit(const voltu::FramePtr &frame)
{
	throw voltu::exceptions::NotImplemented();
}

voltu::FeedType OutputFeedImpl::type()
{
	throw voltu::exceptions::NotImplemented();
}

voltu::PropertyPtr OutputFeedImpl::property(voltu::FeedProperty)
{
	throw voltu::exceptions::NotImplemented();
}

