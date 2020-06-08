#pragma once
#ifndef _FTL_RGBD_PYLONDEVICE_HPP_
#define _FTL_RGBD_PYLONDEVICE_HPP_

#include "device.hpp"
#include <string>

namespace Pylon {
class CBaslerUniversalInstantCamera;
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
	bool get(cv::cuda::GpuMat &l, cv::cuda::GpuMat &r, cv::cuda::GpuMat &h_l, cv::Mat &h_r, Calibrate *c, cv::cuda::Stream &stream) override;

	unsigned int width() const override { return width_; }
	unsigned int height() const override { return height_; };

	unsigned int fullWidth() const override { return fullwidth_; }
	unsigned int fullHeight() const override { return fullheight_; }
	
	double getTimestamp() const override { return 0.0; }
	
	bool isStereo() const override { return lcam_ && rcam_; }

	bool isReady() const;

	private:
	bool ready_;
	Pylon::CBaslerUniversalInstantCamera *lcam_;
	Pylon::CBaslerUniversalInstantCamera *rcam_;
	cv::Mat tmp_;
	uint32_t fullwidth_;
	uint32_t fullheight_;
	uint32_t width_;
	uint32_t height_;

	cv::cuda::HostMem left_hm_;
	cv::cuda::HostMem right_hm_;
	cv::cuda::HostMem hres_hm_;
	cv::Mat rtmp_;

	void _configureCamera(Pylon::CBaslerUniversalInstantCamera *cam);
};

}
}
}

#endif  // _FTL_RGBD_PYLON_HPP_
