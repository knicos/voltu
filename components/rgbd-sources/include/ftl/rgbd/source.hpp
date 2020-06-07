#ifndef _FTL_RGBD_SOURCE_HPP_
#define _FTL_RGBD_SOURCE_HPP_

#include <ftl/cuda_util.hpp>
#include <ftl/configuration.hpp>
#include <ftl/threads.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/uri.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/rgbd/detail/source.hpp>
#include <ftl/codecs/packet.hpp>
#include <opencv2/core/mat.hpp>
#include <Eigen/Eigen>
#include <string>
#include <map>

#include <ftl/cuda_common.hpp>
#include <ftl/rgbd/frame.hpp>

namespace ftl {

namespace net {
class Universe;
}

namespace rgbd {

static inline bool isValidDepth(float d) { return (d > 0.01f) && (d < 39.99f); }

/**
 * RGBD Generic data source configurable entity. This class hides the
 * internal implementation of an RGBD source by providing accessor functions
 * and by automatically changing the implementation in response to any URI
 * changes.
 * 
 * Cannot be constructed directly, use ftl::create<Source>(...).
 * @see ftl::create
 */
class Source : public ftl::Configurable {
	public:
	template <typename T, typename... ARGS>
	friend T *ftl::config::create(ftl::config::json_t &, ARGS ...);
	friend class VirtualSource;

	// This class cannot be constructed directly, use ftl::create
	Source()=delete;

	// Also cannot be copied
	Source(const Source&)=delete;
	Source &operator=(const Source&) =delete;

	protected:
	explicit Source(ftl::config::json_t &cfg);
	Source(ftl::config::json_t &cfg, ftl::net::Universe *net);
	virtual ~Source();

	public:
	/**
	 * Is this source valid and ready to grab?.
	 */
	bool isReady() { return (impl_) ? impl_->isReady() : false; }

	/**
	 * Change the second channel source.
	 */
	bool setChannel(ftl::codecs::Channel c);

	/**
	 * Get the channel allocated to the second source.
	 */
	ftl::codecs::Channel getChannel() const { return channel_; }

	/**
	 * Perform the hardware or virtual frame grab operation. This should be
	 * fast and non-blocking. 
	 */
	bool capture(int64_t ts);

	/**
	 * Download captured frame. This could be a blocking IO operation.
	 */
	bool retrieve();

	/**
	 * Generate a thread job using the provided callback for the most recently
	 * retrieved frame (matching the provided timestamp). If already busy
	 * dispatching, returns false.
	 */
	bool dispatch(int64_t ts);

	/**
	 * Between frames, do any required buffer swaps.
	 */
	//void swap() { if (impl_) impl_->swap(); }

	/**
	 * Do any post-grab processing. This function
	 * may take considerable time to return, especially for sources requiring
	 * software stereo correspondance.
	 */
	//bool compute(int64_t ts);

	//bool isVirtual() const { return impl_ == nullptr; }

	/**
	 * Get the source's camera intrinsics.
	 */
	const Camera &parameters() const {
		if (impl_) return impl_->params_;
		else throw FTL_Error("Cannot get parameters for bad source");
	}

	/**
	 * Get camera intrinsics for another channel. For example the right camera
	 * in a stereo pair.
	 */
	const Camera parameters(ftl::codecs::Channel) const;

	cv::Mat cameraMatrix() const;

	/**
	 * Change the camera extrinsics by providing a new pose matrix. For virtual
	 * cameras this will move the camera, for physical cameras it is set by the
	 * registration process as it attempts to work out a cameras relative pose.
	 */
	virtual void setPose(const Eigen::Matrix4d &pose);

	/**
	 * Check what features this source has available.
	 */
	[[deprecated]] bool hasCapabilities(capability_t);

	[[deprecated]] capability_t getCapabilities() const;

	/**
	 * Force the internal implementation to be reconstructed.
	 */
	void reset();

	[[deprecated]] ftl::net::Universe *getNet() const { return net_; }

	std::string getURI() { return value("uri", std::string("")); }

	ftl::rgbd::FrameState &state() { return impl_->state_; }

	SHARED_MUTEX &mutex() { return mutex_; }

	const FrameCallback &callback() { return callback_; }

	/**
	 * Set the callback that receives decoded frames as they are generated.
	 * There can be only a single such callback as the buffers can be swapped
	 * by the callback.
	 */
	void setCallback(const FrameCallback &cb);
	void removeCallback() { callback_ = nullptr; }

	/**
	 * Notify of a decoded or available pair of frames. This calls the source
	 * callback after having verified the correct resolution of the frames.
	 */
	//void notify(int64_t ts, cv::cuda::GpuMat &c1, cv::cuda::GpuMat &c2);
	//void notify(int64_t ts, ftl::rgbd::Frame &f);


	private:
	BaseSourceImpl *impl_;
	Eigen::Matrix4d pose_;
	ftl::net::Universe *net_;
	SHARED_MUTEX mutex_;
	ftl::codecs::Channel channel_;
	cudaStream_t stream_;
	FrameCallback callback_;
	ftl::rgbd::Frame frames_[2];
	bool is_dispatching;
	bool is_retrieving;

	void _swap();
};

}
}

#endif  // _FTL_RGBD_SOURCE_HPP_
