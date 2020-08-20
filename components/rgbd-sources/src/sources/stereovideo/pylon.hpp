#pragma once
#ifndef _FTL_RGBD_PYLONDEVICE_HPP_
#define _FTL_RGBD_PYLONDEVICE_HPP_

#include "device.hpp"
#include <string>

namespace Pylon {
class CBaslerUniversalInstantCamera;
class CGrabResultPtr;
}

namespace ftl {
namespace rgbd {
namespace detail {

class PylonDevice : public ftl::rgbd::detail::Device {
	public:
	explicit PylonDevice(nlohmann::json &config);
	~PylonDevice();

	static std::vector<DeviceDetails> listDevices();

	bool grab() override;
	bool get(ftl::rgbd::Frame &frame, StereoRectification *c, cv::cuda::Stream &stream) override;

	unsigned int width() const override { return fullwidth_; }
	unsigned int height() const override { return fullheight_; }

	double getTimestamp() const override { return 0.0; }

	bool isStereo() const override { return lcam_ && rcam_; }

	bool isReady() const;

	void populateMeta(std::map<std::string,std::string> &meta) const override;

	private:
	bool ready_;
	Pylon::CBaslerUniversalInstantCamera *lcam_;
	Pylon::CBaslerUniversalInstantCamera *rcam_;
	cv::Mat tmp_;
	uint32_t fullwidth_;
	uint32_t fullheight_;
	std::string name_;
	std::string serial_;
	int left_fail_=0;
	int right_fail_=0;
	int buffer_size_=1;

	cv::cuda::HostMem left_hm_;
	cv::cuda::HostMem right_hm_;
	cv::Mat rtmp_;
	cv::Mat ltmp_;
	int interpolation_;

	std::atomic_bool monitor_;
	ftl::Handle temperature_monitor_;

	void _configureCamera(Pylon::CBaslerUniversalInstantCamera *cam);
	bool _retrieveFrames(Pylon::CGrabResultPtr &result, Pylon::CBaslerUniversalInstantCamera *cam);
};

}
}
}

#endif  // _FTL_RGBD_PYLON_HPP_
