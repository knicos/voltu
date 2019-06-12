#ifndef _FTL_RGBD_SOURCE_HPP_
#define _FTL_RGBD_SOURCE_HPP_

#include <ftl/configuration.hpp>
#include <ftl/rgbd/camera.hpp>
//#include <ftl/net/universe.hpp>
#include <ftl/uri.hpp>
#include <ftl/rgbd/detail/source.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <shared_mutex>
#include <string>

namespace ftl {

namespace net {
class Universe;
}

namespace rgbd {

class SnapshotReader;

enum capability_t {
	kCapColour,		// Has a colour feed
	kCapDepth,		// Has a depth feed
	kCapRight,		// It is possible to get a right image
	kCapMovable,	// Camera is software movable
	kCapVideo,		// It is a video feed, not static
	kCapDisparity	// Raw disparity is available
};

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
	bool isReady() { return (impl_) ? impl_->isReady() : false; }

	/**
	 * Perform the hardware or virtual frame grab operation. 
	 */
	bool grab();

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

	/**
	 * Change the camera extrinsics by providing a new pose matrix. For virtual
	 * cameras this will move the camera, for physical cameras it is set by the
	 * registration process as it attempts to work out a cameras relative pose.
	 */
	virtual void setPose(const Eigen::Matrix4f &pose);

	/**
	 * Get the camera position as a pose matrix.
	 */
	const Eigen::Matrix4f &getPose() const;

	/**
	 * Check what features this source has available.
	 */
	virtual bool hasCapability(capability_t);

	/**
	 * Get a point in camera coordinates at specified pixel location.
	 */
	Eigen::Vector4f point(uint x, uint y);

	/**
	 * Force the internal implementation to be reconstructed.
	 */
	void reset();

	ftl::net::Universe *getNet() const { return net_; }

	std::string getURI() { return value("uri", std::string("")); }

	void customImplementation(detail::Source *);

	std::shared_mutex &mutex() { return mutex_; }

	private:
	detail::Source *impl_;
	cv::Mat rgb_;
	cv::Mat depth_;
	Camera params_;		// TODO Find better solution
	Eigen::Matrix4f pose_;
	ftl::net::Universe *net_;
	std::shared_mutex mutex_;

	detail::Source *_createImplementation();
	detail::Source *_createFileImpl(const ftl::URI &uri);
	detail::Source *_createNetImpl(const ftl::URI &uri);
	detail::Source *_createDeviceImpl(const ftl::URI &uri);
};

}
}

#endif  // _FTL_RGBD_SOURCE_HPP_