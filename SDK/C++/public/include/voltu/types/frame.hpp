#pragma once
#include "../defines.hpp"

#include <voltu/types/channel.hpp>
#include <voltu/types/image.hpp>
#include <voltu/types/pointcloud.hpp>
#include <voltu/types/intrinsics.hpp>
#include <list>
#include <memory>

namespace voltu
{

class Frame
{
public:
	virtual ~Frame() = default;
	
	PY_API PY_RV_LIFETIME_PARENT virtual std::list<voltu::ImagePtr> getImageSet(voltu::Channel channel) = 0;

	PY_API PY_RV_LIFETIME_PARENT virtual voltu::PointCloudPtr getPointCloud(voltu::PointCloudFormat cloudfmt, voltu::PointFormat pointfmt) = 0;

	PY_API virtual std::vector<std::string> getMessages() = 0;

	PY_API virtual int64_t getTimestamp() = 0;
};

typedef std::shared_ptr<Frame> FramePtr;

}
