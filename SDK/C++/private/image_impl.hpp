#pragma once

#include <voltu/types/image.hpp>
#include <ftl/rgbd/frame.hpp>

namespace voltu
{
namespace internal
{

class ImageImpl : public voltu::Image
{
public:
	ImageImpl(const ftl::rgbd::Frame&, ftl::codecs::Channel c);
	~ImageImpl();

	voltu::ImageData getHost() override;

	voltu::ImageData getDevice() override;

	bool isDevice() override;

	voltu::Channel getChannel() override;

	std::string getName() override;

	voltu::Intrinsics getIntrinsics() override;

	voltu::StereoIntrinsics getStereoIntrinsics() override;

	Eigen::Matrix4d getPose() override;

	int64_t getTimestamp() override;

	//virtual voltu::RoomId getRoomId() override;

	int getCameraNumber() override;

	uint32_t getUniqueId() override;

private:
	const ftl::rgbd::Frame &frame_;
	ftl::codecs::Channel channel_;
};

}
}
