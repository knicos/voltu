#include "image_impl.hpp"

using voltu::internal::ImageImpl;

ImageImpl::ImageImpl(const ftl::rgbd::Frame& f, ftl::codecs::Channel c)
 : frame_(f), channel_(c)
{

}

ImageImpl::~ImageImpl()
{

}

voltu::ImageData ImageImpl::getHost()
{
	cv::Mat m = frame_.get<cv::Mat>(channel_);
	voltu::ImageData r;
	r.data = m.data;
	r.width = m.cols;
	r.height = m.rows;
	r.pitch = m.step;

	if (m.type() == CV_8UC4)
	{
		r.format = voltu::ImageFormat::kBGRA8;
	}
	else if (m.type() == CV_32F)
	{
		r.format = voltu::ImageFormat::kFloat32;
	}
	else if (m.type() == CV_16FC4)
	{
		r.format = voltu::ImageFormat::kFloat16_4;
	}

	return r;
}

voltu::ImageData ImageImpl::getDevice()
{
	cv::cuda::GpuMat m = frame_.get<cv::cuda::GpuMat>(channel_);
	voltu::ImageData r;
	r.data = m.data;
	r.width = m.cols;
	r.height = m.rows;
	r.pitch = m.step;

	if (m.type() == CV_8UC4)
	{
		r.format = voltu::ImageFormat::kBGRA8;
	}
	else if (m.type() == CV_32F)
	{
		r.format = voltu::ImageFormat::kFloat32;
	}
	else if (m.type() == CV_16FC4)
	{
		r.format = voltu::ImageFormat::kFloat16_4;
	}

	return r;
}

bool ImageImpl::isDevice()
{
	return true;
}

voltu::Channel ImageImpl::getChannel()
{
	switch (channel_)
	{
	case ftl::codecs::Channel::Colour		: return voltu::Channel::kColour;
	case ftl::codecs::Channel::Depth		: return voltu::Channel::kDepth;
	case ftl::codecs::Channel::Normals		: return voltu::Channel::kNormals;
	default: return voltu::Channel::kInvalid;
	}
}

std::string ImageImpl::getName()
{
	return std::to_string(frame_.frameset()) + std::string("-") + std::to_string(frame_.source());
}

voltu::Intrinsics ImageImpl::getIntrinsics()
{
	auto raw = frame_.getLeft();
	voltu::Intrinsics result;
	result.width = raw.width;
	result.height = raw.height;
	result.principle_x = raw.cx;
	result.principle_y = raw.cy;
	result.focal_x = raw.fx;
	result.focal_y = raw.fy;
	return result;
}

voltu::StereoIntrinsics ImageImpl::getStereoIntrinsics()
{
	auto raw = frame_.getLeft();
	voltu::StereoIntrinsics result;
	result.width = raw.width;
	result.height = raw.height;
	result.principle_x = raw.cx;
	result.principle_y = raw.cy;
	result.focal_x = raw.fx;
	result.focal_y = raw.fy;
	result.min_depth = raw.minDepth;
	result.max_depth = raw.maxDepth;
	result.baseline = raw.baseline;
	return result;
}

Eigen::Matrix4d ImageImpl::getPose()
{
	return frame_.getPose();
}

int64_t ImageImpl::getTimestamp()
{
	return frame_.timestamp();
}

int ImageImpl::getCameraNumber()
{
	return frame_.source();
}

uint32_t ImageImpl::getUniqueId()
{
	return frame_.id().id;
}
