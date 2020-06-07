#ifndef _FTL_RGBD_DETAIL_SOURCE_HPP_
#define _FTL_RGBD_DETAIL_SOURCE_HPP_

#include <Eigen/Eigen>
#include <ftl/cuda_util.hpp>
//#include <opencv2/opencv.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/rgbd/frame.hpp>

namespace ftl{
namespace rgbd {

class Source;

class BaseSourceImpl {
	public:
	friend class ftl::rgbd::Source;

	public:
	explicit BaseSourceImpl(ftl::rgbd::Source *host) : capabilities_(0), host_(host), params_(state_.getLeft()) { }
	virtual ~BaseSourceImpl() {}

	/**
	 * Perform hardware data capture. This should be low latency.
	 */
	virtual bool capture(int64_t ts)=0;

	/**
	 * Perform slow IO operation to get the data into the given frame object.
	 */
	virtual bool retrieve(ftl::rgbd::Frame &frame)=0;

	/**
	 * Is the source ready to capture and retrieve?
	 */
	virtual bool isReady() { return false; };

	[[deprecated]] virtual void setPose(const Eigen::Matrix4d &pose) { state_.setPose(pose); };
	[[deprecated]] virtual Camera parameters(ftl::codecs::Channel) { return params_; };

	ftl::rgbd::Source *host() { return host_; }
	ftl::rgbd::FrameState &state() { return state_; }

	protected:
	ftl::rgbd::FrameState state_;
	capability_t capabilities_;
	ftl::rgbd::Source *host_;
	ftl::rgbd::Camera &params_;
};

}
}

#endif  // _FTL_RGBD_DETAIL_SOURCE_HPP_
