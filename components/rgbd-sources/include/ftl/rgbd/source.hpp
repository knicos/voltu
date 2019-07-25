#ifndef _FTL_RGBD_SOURCE_HPP_
#define _FTL_RGBD_SOURCE_HPP_

#include <ftl/configuration.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/threads.hpp>
//#include <ftl/net/universe.hpp>
#include <ftl/uri.hpp>
#include <ftl/rgbd/detail/source.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <string>

#include <ftl/cuda_common.hpp>

namespace ftl {

namespace net {
class Universe;
}

namespace rgbd {

static inline bool isValidDepth(float d) { return (d > 0.01f) && (d < 39.99f); }

class SnapshotReader;

typedef unsigned int channel_t;

static const channel_t kChanNone = 0;
static const channel_t kChanLeft = 0x0001;
static const channel_t kChanDepth = 0x0002;
static const channel_t kChanRight = 0x0004;
static const channel_t kChanDisparity = 0x0008;
static const channel_t kChanDeviation = 0x0010;

static const channel_t kChanOverlay1 = 0x1000;

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

	//template <typename T, typename... ARGS>
	//friend T *ftl::config::create(ftl::Configurable *, const std::string &, ARGS ...);

	// This class cannot be constructed directly, use ftl::create
	Source()=delete;

	// Also cannot be copied
	Source(const Source&)=delete;
	Source &operator=(const Source&) =delete;

	private:
	explicit Source(ftl::config::json_t &cfg);
	Source(ftl::config::json_t &cfg, ftl::rgbd::SnapshotReader *);
	Source(ftl::config::json_t &cfg, ftl::net::Universe *net);
	~Source();

	public:
	/**
	 * Is this source valid and ready to grab?.
	 */
	bool isReady() { return (impl_) ? impl_->isReady() : params_.width != 0; }

	/**
	 * Change the second channel source.
	 */
	bool setChannel(channel_t c);

	channel_t getChannel() const { return channel_; }

	/**
	 * Perform the hardware or virtual frame grab operation. 
	 */
	bool grab(int N=-1, int B=-1);

	/**
	 * Do any post-grab processing. This function
	 * may take considerable time to return, especially for sources requiring
	 * software stereo correspondance. If `process` is not called manually
	 * after a `grab` and before a `get`, then it will be called automatically
	 * on first `get`.
	 */
	//void process();

	/**
	 * Get a copy of both colour and depth frames.
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

	/**
	 * Write frames into source buffers from an external renderer. Virtual
	 * sources do not have an internal generator of frames but instead have
	 * their data provided from an external rendering class. This function only
	 * works when there is no internal generator.
	 */
	void writeFrames(const cv::Mat &rgb, const cv::Mat &depth);
	void writeFrames(const ftl::cuda::TextureObject<uchar4> &rgb, const ftl::cuda::TextureObject<uint> &depth, cudaStream_t stream);
	void writeFrames(const ftl::cuda::TextureObject<uchar4> &rgb, const ftl::cuda::TextureObject<float> &depth, cudaStream_t stream);

	int64_t timestamp() const { return timestamp_; }

	/**
	 * Directly upload source RGB and Depth to GPU.
	 */
	void upload(cv::cuda::GpuMat&, cv::cuda::GpuMat&);

	void uploadColour(cv::cuda::GpuMat&);
	void uploadDepth(cv::cuda::GpuMat&);

	/**
	 * Get the source's camera intrinsics.
	 */
	const Camera &parameters() const {
		if (impl_) return impl_->params_;
		else return params_;
	}

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

	std::function<void(int64_t, const cv::Mat &, const cv::Mat &)> &callback() { return callback_; }
	void setCallback(std::function<void(int64_t, const cv::Mat &, const cv::Mat &)> cb) { callback_ = cb; }


	private:
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
	channel_t channel_;
	cudaStream_t stream_;
	int64_t timestamp_;
	std::function<void(int64_t, const cv::Mat &, const cv::Mat &)> callback_;

	detail::Source *_createImplementation();
	detail::Source *_createFileImpl(const ftl::URI &uri);
	detail::Source *_createNetImpl(const ftl::URI &uri);
	detail::Source *_createDeviceImpl(const ftl::URI &uri);
};

}
}

#endif  // _FTL_RGBD_SOURCE_HPP_
