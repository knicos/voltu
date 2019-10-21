#ifndef _FTL_LOCAL_HPP_
#define _FTL_LOCAL_HPP_

#include <ftl/configurable.hpp>
#include <ftl/cuda_common.hpp>
#include <string>
#include <nlohmann/json.hpp>

namespace cv {
	class Mat;
	class VideoCapture;
};

namespace ftl {
namespace rgbd {
namespace detail {

class Calibrate;

class LocalSource : public Configurable {
	public:
	explicit LocalSource(nlohmann::json &config);
	LocalSource(nlohmann::json &config, const std::string &vid);
	
	//bool left(cv::Mat &m);
	//bool right(cv::Mat &m);
	bool grab();
	bool get(cv::cuda::GpuMat &l, cv::cuda::GpuMat &r, Calibrate *c, cv::cuda::Stream &stream);

	unsigned int width() const { return width_; }
	unsigned int height() const { return height_; }
	
	//void setFramerate(float fps);
	//float getFramerate() const;
	
	double getTimestamp() const;
	
	bool isStereo() const;
	
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

	cv::cuda::HostMem left_hm_;
	cv::cuda::HostMem right_hm_;
};

}
}
}

#endif // _FTL_LOCAL_HPP_

