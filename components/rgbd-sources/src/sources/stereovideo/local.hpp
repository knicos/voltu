#ifndef _FTL_LOCAL_HPP_
#define _FTL_LOCAL_HPP_

#include "device.hpp"

namespace cv {
	class Mat;
	class VideoCapture;
};

namespace ftl {
namespace rgbd {
namespace detail {

class OpenCVDevice : public ftl::rgbd::detail::Device {
	public:
	explicit OpenCVDevice(nlohmann::json &config);
	~OpenCVDevice();

	static std::vector<DeviceDetails> listDevices();
	
	bool grab() override;
	bool get(cv::cuda::GpuMat &l, cv::cuda::GpuMat &r, cv::cuda::GpuMat &h_l, cv::Mat &h_r, Calibrate *c, cv::cuda::Stream &stream) override;

	unsigned int width() const override { return dwidth_; }
	unsigned int height() const override { return dheight_; }

	unsigned int fullWidth() const override { return width_; }
	unsigned int fullHeight() const override { return height_; }
	
	double getTimestamp() const override;
	
	bool isStereo() const override;
	
	private:
	double timestamp_;
	//double tps_;
	bool stereo_;
	//float fps_;
	//bool flip_;
	//bool flip_v_;
	bool nostereo_;
	//float downsize_;
	cv::VideoCapture *camera_a_;
	cv::VideoCapture *camera_b_;
	unsigned int width_;
	unsigned int height_;
	unsigned int dwidth_;
	unsigned int dheight_;

	cv::cuda::HostMem left_hm_;
	cv::cuda::HostMem right_hm_;
	cv::cuda::HostMem hres_hm_;
	cv::Mat rtmp_;

	cv::Mat frame_l_;
	cv::Mat frame_r_;

	std::vector<DeviceDetails> _selectDevices();
};

}
}
}

#endif // _FTL_LOCAL_HPP_

