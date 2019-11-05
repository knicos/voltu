#ifndef _FTL_RGBD_DETAIL_SOURCE_HPP_
#define _FTL_RGBD_DETAIL_SOURCE_HPP_

#include <Eigen/Eigen>
#include <ftl/cuda_util.hpp>
#include <opencv2/opencv.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/rgbd/frame.hpp>

namespace ftl{
namespace rgbd {

class Source;

typedef unsigned int capability_t;

static const capability_t kCapMovable	= 0x0001;	// A movable virtual cam
static const capability_t kCapVideo		= 0x0002;	// Is a video feed
static const capability_t kCapActive	= 0x0004;	// An active depth sensor
static const capability_t kCapStereo	= 0x0008;	// Has right RGB
static const capability_t kCapDepth		= 0x0010;	// Has depth capabilities


namespace detail {

class Source {
	public:
	friend class ftl::rgbd::Source;

	public:
	explicit Source(ftl::rgbd::Source *host) : capabilities_(0), host_(host), params_({0}), timestamp_(0) { }
	virtual ~Source() {}

	/**
	 * Perform hardware data capture.
	 */
	virtual bool capture(int64_t ts)=0;

	/**
	 * Perform IO operation to get the data.
	 */
	virtual bool retrieve()=0;

	/**
	 * Do any processing from previously captured frames...
	 * @param n Number of frames to request in batch. Default -1 means automatic (10)
	 * @param b Bit rate setting. -1 = automatic, 0 = best quality, 9 = lowest quality
	 */
	virtual bool compute(int n, int b)=0;

	/**
	 * Between frames, or before next frame, do any buffer swapping operations.
	 */
	virtual void swap() {}

	virtual bool isReady() { return false; };
	virtual void setPose(const Eigen::Matrix4d &pose) { };

	virtual Camera parameters(ftl::codecs::Channel) { return params_; };

	protected:
	capability_t capabilities_;
	ftl::rgbd::Source *host_;
	ftl::rgbd::Camera params_;
	cv::cuda::GpuMat rgb_;
	cv::cuda::GpuMat depth_;
	int64_t timestamp_;
	//Eigen::Matrix4f pose_;
};

}	
}
}

#endif  // _FTL_RGBD_DETAIL_SOURCE_HPP_
