#include "frame_impl.hpp"
#include "image_impl.hpp"
#include <ftl/rgbd/frame.hpp>
#include <voltu/types/errors.hpp>

using voltu::internal::FrameImpl;

FrameImpl::FrameImpl()
{

}

FrameImpl::~FrameImpl()
{

}

std::list<voltu::ImagePtr> FrameImpl::getImageSet(voltu::Channel c)
{
	std::list<voltu::ImagePtr> result;
	ftl::codecs::Channel channel = ftl::codecs::Channel::Colour;

	switch (c)
	{
	case voltu::Channel::kColour	: channel = ftl::codecs::Channel::Colour; break;
	case voltu::Channel::kDepth		: channel = ftl::codecs::Channel::Depth; break;
	case voltu::Channel::kNormals	: channel = ftl::codecs::Channel::Normals; break;
	default: throw voltu::exceptions::BadImageChannel();
	}

	for (const auto &fs : framesets_)
	{
		for (const auto &f : fs->frames)
		{
			if (f.hasChannel(channel))
			{
				auto img = std::make_shared<voltu::internal::ImageImpl>(
					f.cast<ftl::rgbd::Frame>(), channel
				);
				result.push_back(img);
			}
		}
	}

	return result;
}

voltu::PointCloudPtr FrameImpl::getPointCloud(voltu::PointCloudFormat cloudfmt, voltu::PointFormat pointfmt)
{
	return nullptr;
}

std::vector<std::string> FrameImpl::getMessages()
{
	std::vector<std::string> msgs;

	for (const auto &fs : framesets_)
	{
		for (const auto &f : fs->frames)
		{
			if (f.hasChannel(ftl::codecs::Channel::Messages))
			{
				const auto &m = f.get<std::vector<std::string>>(ftl::codecs::Channel::Messages);
				msgs.insert(msgs.end(), m.begin(), m.end());
			}
		}
	}

	return msgs;
}

void FrameImpl::pushFrameSet(const std::shared_ptr<ftl::data::FrameSet> &fs)
{
	framesets_.push_back(fs);
}

int64_t FrameImpl::getTimestamp()
{
	return 0;
}
