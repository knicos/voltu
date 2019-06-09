#ifndef _FTL_RGBD_DETAIL_SOURCE_HPP_
#define _FTL_RGBD_DETAIL_SOURCE_HPP_

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <ftl/rgbd/camera.hpp>

namespace ftl{
namespace rgbd {

class Source;

namespace detail {

class Source {
	public:
	friend class ftl::rgbd::Source;

	public:
	explicit Source(ftl::rgbd::Source *host) : host_(host), params_({0}) { }
	virtual ~Source() {}

	virtual bool grab()=0;
	virtual bool isReady() { return false; };
	virtual void setPose(const Eigen::Matrix4f &pose) { };

	protected:
	ftl::rgbd::Source *host_;
	ftl::rgbd::Camera params_;
	cv::Mat rgb_;
	cv::Mat depth_;
	//Eigen::Matrix4f pose_;
};

}	
}
}

#endif  // _FTL_RGBD_DETAIL_SOURCE_HPP_
