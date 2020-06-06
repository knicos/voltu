#ifndef _FTL_RGBD_STEREOVIDEO_DEVICE_HPP_
#define _FTL_RGBD_STEREOVIDEO_DEVICE_HPP_

#include <ftl/configurable.hpp>
#include <ftl/cuda_common.hpp>
#include <string>

namespace ftl {
namespace rgbd {
namespace detail {

class Calibrate;

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
	virtual bool get(cv::cuda::GpuMat &l, cv::cuda::GpuMat &r, cv::cuda::GpuMat &h_l, cv::Mat &h_r, Calibrate *c, cv::cuda::Stream &stream)=0;

	virtual unsigned int width() const =0;
	virtual unsigned int height() const =0;

	virtual unsigned int fullWidth() const =0;
	virtual unsigned int fullHeight() const =0;

	inline bool hasHigherRes() const { return fullWidth() != width(); }
	
	virtual double getTimestamp() const =0;
	
	virtual bool isStereo() const =0;
};

}
}
}

#endif