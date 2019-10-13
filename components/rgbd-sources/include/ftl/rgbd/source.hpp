#ifndef _FTL_RGBD_SOURCE_HPP_
#define _FTL_RGBD_SOURCE_HPP_

#include <ftl/cuda_util.hpp>
#include <ftl/configuration.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/threads.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/uri.hpp>
#include <ftl/rgbd/detail/source.hpp>
#include <ftl/codecs/packet.hpp>
#include <ftl/codecs/reader.hpp>
#include <opencv2/opencv.hpp>
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

class SnapshotReader;
class VirtualSource;

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

	//template <typename T, typename... ARGS>
	//friend T *ftl::config::create(ftl::Configurable *, const std::string &, ARGS ...);

	// This class cannot be constructed directly, use ftl::create
	Source()=delete;

	// Also cannot be copied
	Source(const Source&)=delete;
	Source &operator=(const Source&) =delete;

	protected:
	explicit Source(ftl::config::json_t &cfg);
	Source(ftl::config::json_t &cfg, ftl::rgbd::SnapshotReader *);
	Source(ftl::config::json_t &cfg, ftl::net::Universe *net);
	virtual ~Source();

	public:
	/**
	 * Is this source valid and ready to grab?.
	 */
	bool isReady() { return (impl_) ? impl_->isReady() : params_.width != 0; }

	/**
	 * Change the second channel source.
	 */
	bool setChannel(ftl::codecs::Channel c);

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
	 * Between frames, do any required buffer swaps.
	 */
	void swap() { if (impl_) impl_->swap(); }

	/**
	 * Do any post-grab processing. This function
	 * may take considerable time to return, especially for sources requiring
	 * software stereo correspondance.
	 */
	bool compute(int N=-1, int B=-1);

	/**
	 * Wrapper grab that performs capture, swap and computation steps in one.
	 * It is more optimal to perform capture and compute in parallel.
	 */
	bool grab(int N=-1, int B=-1) {
		bool c = capture(0);
		c = c && retrieve();
		swap();
		return c && compute(N,B);
	}

	/**
	 * Get a copy of both colour and depth frames. Note that this does a buffer
	 * swap rather than a copy, so the parameters should be persistent buffers for
	 * best performance.
	 */
	void getFrames(cv::Mat &c, cv::Mat &d);

	/**
	 * Get a copy of the colour frame only.
	 */
	void getColour(cv::Mat &c);

	/**
	 * Get a copy of the depth frame only.
	 */
	void getDepth(cv::Mat &d);

	int64_t timestamp() const { return timestamp_; }

	/**
	 * Directly upload source RGB and Depth to GPU.
	 */
	void upload(cv::cuda::GpuMat&, cv::cuda::GpuMat&);

	void uploadColour(cv::cuda::GpuMat&);
	void uploadDepth(cv::cuda::GpuMat&);

	bool isVirtual() const { return impl_ == nullptr; }

	/**
	 * Get the source's camera intrinsics.
	 */
	const Camera &parameters() const {
		if (impl_) return impl_->params_;
		else return params_;
	}

	const Camera parameters(ftl::codecs::Channel) const;

	cv::Mat cameraMatrix() const;

	/**
	 * Change the camera extrinsics by providing a new pose matrix. For virtual
	 * cameras this will move the camera, for physical cameras it is set by the
	 * registration process as it attempts to work out a cameras relative pose.
	 */
	virtual void setPose(const Eigen::Matrix4d &pose);

	/**
	 * Get the camera position as a pose matrix.
	 */
	const Eigen::Matrix4d &getPose() const;

	/**
	 * Check what features this source has available.
	 */
	bool hasCapabilities(capability_t);

	capability_t getCapabilities() const;

	/**
	 * Get a point in camera coordinates at specified pixel location.
	 */
	Eigen::Vector4d point(uint x, uint y);

	/**
	 * Get a point in camera coordinates at specified pixel location, with a
	 * given depth value.
	 */
	Eigen::Vector4d point(uint x, uint y, double d);

	Eigen::Vector2i point(const Eigen::Vector4d &p);

	/**
	 * Force the internal implementation to be reconstructed.
	 */
	void reset();

	void pause(bool);
	bool isPaused() { return paused_; }

	void bullet(bool);
	bool isBullet() { return bullet_; }

	bool thumbnail(cv::Mat &t);

	ftl::net::Universe *getNet() const { return net_; }

	std::string getURI() { return value("uri", std::string("")); }

	void customImplementation(detail::Source *);

	SHARED_MUTEX &mutex() { return mutex_; }

	std::function<void(int64_t, cv::Mat &, cv::Mat &)> &callback() { return callback_; }

	/**
	 * Set the callback that receives decoded frames as they are generated.
	 */
	void setCallback(std::function<void(int64_t, cv::Mat &, cv::Mat &)> cb);
	void removeCallback() { callback_ = nullptr; }

	/**
	 * Add a callback to immediately receive any raw data from this source.
	 * Currently this only works for a net source since other sources don't
	 * produce raw encoded data.
	 */
	void addRawCallback(const std::function<void(ftl::rgbd::Source*, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)> &);

	void removeRawCallback(const std::function<void(ftl::rgbd::Source*, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)> &);

	/**
	 * INTERNAL. Used to send raw data to callbacks.
	 */
	void notifyRaw(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt);

	protected:
	detail::Source *impl_;
	cv::Mat rgb_;
	cv::Mat depth_;
	cv::Mat thumb_;
	Camera params_;		// TODO Find better solution
	Eigen::Matrix4d pose_;
	ftl::net::Universe *net_;
	SHARED_MUTEX mutex_;
	bool paused_;
	bool bullet_;
	ftl::codecs::Channel channel_;
	cudaStream_t stream_;
	int64_t timestamp_;
	std::function<void(int64_t, cv::Mat &, cv::Mat &)> callback_;
	std::list<std::function<void(ftl::rgbd::Source*, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)>> rawcallbacks_;

	detail::Source *_createImplementation();
	detail::Source *_createFileImpl(const ftl::URI &uri);
	detail::Source *_createNetImpl(const ftl::URI &uri);
	detail::Source *_createDeviceImpl(const ftl::URI &uri);

	static ftl::codecs::Reader *__createReader(const std::string &path);

	static std::map<std::string, ftl::codecs::Reader*> readers__;
};

}
}

#endif  // _FTL_RGBD_SOURCE_HPP_
