#include "system_impl.hpp"
#include "feed_impl.hpp"
#include "room_impl.hpp"
#include "observer_impl.hpp"
#include <voltu/voltu.hpp>
#include <voltu/types/errors.hpp>
#include <ftl/timer.hpp>
#include <iostream>

using voltu::internal::SystemImpl;

static bool g_isinit = false;

#if defined(WIN32)
#define EXTERN_DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define EXTERN_DLL_EXPORT extern "C"
#endif

EXTERN_DLL_EXPORT voltu::System* voltu_initialise()
{
	if (!g_isinit)
	{
		return new SystemImpl();
	}
	else
	{
		throw voltu::exceptions::AlreadyInit();
	}
	return nullptr;
}

SystemImpl::SystemImpl()
{
	int argc = 1;
	char arg1[] = {'v','o','l','t','u',0};
	char* argv[] = {arg1,0};

	root_ = ftl::configure(argc, argv, "sdk");
	net_ = ftl::create<ftl::net::Universe>(root_, "net");
	feed_ = ftl::create<ftl::stream::Feed>(root_, "system", net_);

	net_->start();

	ftl::timer::start(false);
}

SystemImpl::~SystemImpl()
{
	ftl::timer::stop(true);
	ftl::pool.stop(true);
}

voltu::Version SystemImpl::getVersion() const
{
	voltu::Version v;
	v.major = VOLTU_VERSION_MAJOR;
	v.minor = VOLTU_VERSION_MINOR;
	v.patch = VOLTU_VERSION_PATCH;
	return v;
}

voltu::FeedPtr SystemImpl::open(const std::string& uri)
{
	try
	{
		uint32_t fsid = feed_->add(uri);
		return std::make_shared<voltu::internal::FeedImpl>(feed_, fsid);
	}
	catch(const std::exception &e)
	{
		throw voltu::exceptions::BadSourceURI();
	}
};

std::list<voltu::RoomId> SystemImpl::listRooms()
{
	auto fsids = feed_->listFrameSets();
	std::list<voltu::RoomId> res;
	for (unsigned int fsid : fsids) res.push_front(static_cast<voltu::RoomId>(fsid));
	return res;
}

voltu::RoomPtr SystemImpl::getRoom(voltu::RoomId id)
{
	auto s = std::make_shared<voltu::internal::RoomImpl>(feed_);
	s->addFrameSet(id);
	return s;
}

voltu::ObserverPtr SystemImpl::createObserver()
{
	return std::make_shared<voltu::internal::ObserverImpl>(root_);
}
