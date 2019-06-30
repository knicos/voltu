#ifndef _FTL_RGBD_DETAIL_SOURCE_HPP_
#define _FTL_RGBD_DETAIL_SOURCE_HPP_

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <ftl/rgbd/camera.hpp>

namespace ftl{
namespace rgbd {

class Source;

typedef unsigned int capability_t;

static const capability_t kCapMovable	= 0x0001;	// A movable virtual cam
static const capability_t kCapVideo		= 0x0002;	// Is a video feed
static const capability_t kCapActive	= 0x0004;	// An active depth sensor
static const capability_t kCapStereo	= 0x0005;	// Has right RGB

namespace detail {

class Source {
	public:
	friend class ftl::rgbd::Source;

	public:
	explicit Source(ftl::rgbd::Source *host) : capabilities_(0), host_(host), params_({0}) { }
	virtual ~Source() {}

	/**
	 * @param n Number of frames to request in batch. Default -1 means automatic (10)
	 * @param b Bit rate setting. -1 = automatic, 0 = best quality, 9 = lowest quality
	 */
	virtual bool grab(int n, int b)=0;
	virtual bool isReady() { return false; };
	virtual void setPose(const Eigen::Matrix4d &pose) { };

	protected:
	capability_t capabilities_;
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
