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
	explicit OpenCVDevice(nlohmann::json &config, bool stereo);
	~OpenCVDevice();

	static std::vector<DeviceDetails> listDevices();

	bool grab() override;
	bool get(ftl::rgbd::Frame &frame, cv::cuda::GpuMat &l, cv::cuda::GpuMat &r, cv::cuda::GpuMat &h_l, cv::Mat &h_r, StereoRectification *c, cv::cuda::Stream &stream) override;

	unsigned int width() const override { return dwidth_; }
	unsigned int height() const override { return dheight_; }

	unsigned int fullWidth() const override { return width_; }
	unsigned int fullHeight() const override { return height_; }

	double getTimestamp() const override;

	bool isStereo() const override;

	void populateMeta(std::map<std::string,std::string> &meta) const override;

	static std::vector<DeviceDetails> getDevices();

	private:
	std::vector<ftl::rgbd::detail::DeviceDetails> devices_;
	int dev_ix_left_ = -1;
	int dev_ix_right_ = -1;
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
};

}
}
}

#endif // _FTL_LOCAL_HPP_

