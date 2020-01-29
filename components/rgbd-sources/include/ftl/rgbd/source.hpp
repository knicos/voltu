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
class Player;

typedef std::function<void(ftl::rgbd::Source*, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)> RawCallback;
typedef std::function<void(int64_t,ftl::rgbd::Frame&)> FrameCallback;

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
	Source(ftl::config::json_t &cfg, ftl::rgbd::SnapshotReader *);
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
	[[deprecated]] void getFrames(cv::Mat &c, cv::Mat &d);

	/**
	 * Directly upload source RGB and Depth to GPU.
	 */
	void upload(cv::cuda::GpuMat&, cv::cuda::GpuMat&);

	void uploadColour(cv::cuda::GpuMat&);
	void uploadDepth(cv::cuda::GpuMat&);

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
	 * Get the camera position as a pose matrix.
	 */
	[[deprecated]] const Eigen::Matrix4d &getPose() const;

	/**
	 * Check what features this source has available.
	 */
	bool hasCapabilities(capability_t);

	capability_t getCapabilities() const;

	/**
	 * Force the internal implementation to be reconstructed.
	 */
	void reset();

	ftl::net::Universe *getNet() const { return net_; }

	std::string getURI() { return value("uri", std::string("")); }

	ftl::rgbd::FrameState &state() { return impl_->state_; }

	//void customImplementation(detail::Source *);

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
	 * Add a callback to immediately receive any raw data from this source.
	 * Currently this only works for a net source since other sources don't
	 * produce raw encoded data.
	 */
	void addRawCallback(const RawCallback &);

	/**
	 * THIS DOES NOT WORK CURRENTLY.
	 */
	void removeRawCallback(const RawCallback &);

	/**
	 * INTERNAL. Used to send raw data to callbacks.
	 */
	void notifyRaw(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt);

	/**
	 * Notify of a decoded or available pair of frames. This calls the source
	 * callback after having verified the correct resolution of the frames.
	 */
	//void notify(int64_t ts, cv::cuda::GpuMat &c1, cv::cuda::GpuMat &c2);
	void notify(int64_t ts, ftl::rgbd::Frame &f);

	// ==== Inject Data into stream ============================================

	/**
	 * Generate a stream packet with arbitrary data. The data is packed using
	 * msgpack and is given the timestamp of the most recent frame.
	 */
	template <typename... ARGS>
	void inject(ftl::codecs::Channel c, ARGS... args);

	void inject(const Eigen::Matrix4d &pose);

	protected:
	detail::Source *impl_;
	Eigen::Matrix4d pose_;
	ftl::net::Universe *net_;
	SHARED_MUTEX mutex_;
	ftl::codecs::Channel channel_;
	cudaStream_t stream_;
	FrameCallback callback_;
	std::list<RawCallback> rawcallbacks_;

	detail::Source *_createImplementation();
	detail::Source *_createFileImpl(const ftl::URI &uri);
	detail::Source *_createNetImpl(const ftl::URI &uri);
	detail::Source *_createDeviceImpl(const ftl::URI &uri);

	static ftl::rgbd::Player *__createReader(const std::string &path);

	static std::map<std::string, ftl::rgbd::Player*> readers__;
};

}
}

class VectorBuffer {
	public:
	inline explicit VectorBuffer(std::vector<unsigned char> &v) : vector_(v) {}

	inline void write(const char *data, std::size_t size) {
		vector_.insert(vector_.end(), (const unsigned char*)data, (const unsigned char*)data+size);
	}

	private:
	std::vector<unsigned char> &vector_;
};

template <typename... ARGS>
void ftl::rgbd::Source::inject(ftl::codecs::Channel c, ARGS... args) {
	if (!impl_) return;
	auto data = std::make_tuple(args...);

	ftl::codecs::StreamPacket spkt;
	ftl::codecs::Packet pkt;

	spkt.timestamp = impl_->timestamp_;
	spkt.channel = c;
	spkt.frame_number = 0;
	spkt.streamID = 0;
	pkt.codec = ftl::codecs::codec_t::MSGPACK;
	pkt.bitrate = 0;
	pkt.frame_count = 1;
	pkt.definition = ftl::codecs::definition_t::Any;
	pkt.flags = 0;

	VectorBuffer buf(pkt.data);
	msgpack::pack(buf, data);

	notifyRaw(spkt, pkt);
}

#endif  // _FTL_RGBD_SOURCE_HPP_
