#ifndef _FTL_RGBD_STEREOVIDEO_DEVICE_HPP_
#define _FTL_RGBD_STEREOVIDEO_DEVICE_HPP_

#include <ftl/configurable.hpp>
#include <ftl/cuda_common.hpp>
#include <string>

namespace ftl {
namespace rgbd {
class Frame;

namespace detail {

class StereoRectification;

struct DeviceDetails {
	std::string name;
	int id;
	size_t maxwidth;
	size_t maxheight;
};

/**
 * Abstract base class for camera or stereo camera sources. Just wraps the
 * basic grab and retrieve functionality with rectification.
 *
 * @see OpenCVDevice
 * @see PylonDevice
 */
class Device : public Configurable {
	public:
	explicit Device(nlohmann::json &config);
	virtual ~Device();

	//virtual const std::vector<DeviceDetails> &listDevices()=0;

	virtual bool grab()=0;
	virtual bool get(ftl::rgbd::Frame &frame, StereoRectification *c, cv::cuda::Stream &stream)=0;

	virtual unsigned int width() const =0;
	virtual unsigned int height() const =0;

	virtual double getTimestamp() const =0;

	virtual bool isStereo() const =0;

	virtual void populateMeta(std::map<std::string,std::string> &meta) const {}
};

}
}
}

#endif
