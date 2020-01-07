#ifndef _FTL_RGBD_STREAMER_HPP_
#define _FTL_RGBD_STREAMER_HPP_

#include <loguru.hpp>
#include <ftl/configuration.hpp>
#include <ftl/configurable.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/rgbd/group.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/codecs/encoder.hpp>
#include <ftl/threads.hpp>
#include <string>
#include <vector>
#include <map>
#include <atomic>

namespace ftl {
namespace rgbd {

//static const int kChunkDim = 4;
//static constexpr int kChunkCount = kChunkDim * kChunkDim;

namespace detail {

struct StreamClient {
	std::string uri;
	ftl::UUID peerid;
	std::atomic<int> txcount;	// Frames sent since last request
	int txmax;					// Frames to send in request
	ftl::codecs::preset_t preset;
};

static const unsigned int kGrabbed = 0x1;
static const unsigned int kRGB = 0x2;
static const unsigned int kDepth = 0x4;

static const unsigned int kFrameDropLimit = 5;
static const unsigned int kMaxBitrateLevels = 10;

struct StreamSource {
	ftl::rgbd::Source *src;
	std::atomic<int> jobs;				// Busy or ready to swap?
	std::atomic<unsigned int> clientCount;

	int hq_count;	// Number of high quality requests
	int lq_count;	// Number of low quality requests
	ftl::codecs::preset_t hq_bitrate=ftl::codecs::kPresetBest;	// Max bitrate
	ftl::codecs::preset_t lq_bitrate=ftl::codecs::kPresetWorst;	// Min bitrate

	cv::Mat rgb;									// Tx buffer
	cv::Mat depth;									// Tx buffer
	cv::Mat prev_rgb;
	cv::Mat prev_depth;
	std::list<detail::StreamClient> clients;
	SHARED_MUTEX mutex;
	unsigned long long frame;
	int id;

	ftl::codecs::Encoder *hq_encoder_c1 = nullptr;
	ftl::codecs::Encoder *hq_encoder_c2 = nullptr;
	ftl::codecs::Encoder *lq_encoder_c1 = nullptr;
	ftl::codecs::Encoder *lq_encoder_c2 = nullptr;
};

}

/**
 * The maximum number of frames a client can request in a single request.
 */
static const int kMaxFrames = 100;

enum encoder_t {
	kEncodeVideo,
	kEncodeImages
};

/**
 * Allows network streaming of a number of RGB-Depth sources. Remote machines
 * can discover available streams from an instance of Streamer. It also allows
 * for adaptive bitrate streams where bandwidth can be monitored and different
 * data rates can be requested, it is up to the remote machine to decide on
 * desired bitrate.
 * 
 * The capture and compression steps operate in different threads and each
 * source and bitrate also operate on different threads. For a specific source
 * and specific bitrate there is a single thread that sends data to all
 * requesting clients.
 * 
 * Use ftl::create<Streamer>(parent, name) to construct, don't construct
 * directly.
 * 
 * Note that the streamer attempts to maintain a fixed frame rate by
 * monitoring job processing times and sleeping if there is spare time.
 */
class Streamer : public ftl::Configurable {
	public:
	Streamer(nlohmann::json &config, ftl::net::Universe *net);
	~Streamer();

	/**
	 * Add an RGB-Depth source to be made available for streaming.
	 */
	void add(Source *);

	/**
	 * Allow all sources in another group to be proxy streamed by this streamer.
	 */
	void add(ftl::rgbd::Group *grp);

	ftl::rgbd::Group *group() { return &group_; }

	void remove(Source *);
	void remove(const std::string &);

	/**
	 * Enable the streaming. This creates the threads, and if block is false
	 * then another thread will manage the stream process.
	 */
	void run(bool block=false);

	/**
	 * Terminate all streaming and join the threads.
	 */
	void stop();

	void wait();

	void setLatency(int l) { group_.setLatency(l); }

	/**
	 * Alternative to calling run(), it will operate a single frame capture,
	 * compress and stream cycle.
	 */
	void poll();

	Source *get(const std::string &uri);

	private:
	ftl::rgbd::Group group_;
	std::map<std::string, detail::StreamSource*> sources_;
	std::list<ftl::rgbd::Group*> proxy_grps_;
	//ctpl::thread_pool pool_;
	SHARED_MUTEX mutex_;
	bool active_;
	ftl::net::Universe *net_;
	bool late_;
	int compress_level_;
	int64_t clock_adjust_;
	ftl::UUID time_peer_;
	int64_t last_frame_;
	int64_t frame_no_;

	encoder_t encode_mode_;

	int64_t mspf_;
	float actual_fps_;
	//int64_t last_dropped_;
	//int drop_count_;

	ftl::timer::TimerHandle timer_job_;

	ftl::codecs::device_t hq_devices_;
	ftl::codecs::codec_t hq_codec_;

	enum class Quality {
		High,
		Low,
		Any
	};

	void _process(ftl::rgbd::FrameSet &);
	void _cleanUp();
	void _addClient(const std::string &source, int N, int rate, const ftl::UUID &peer, const std::string &dest);
	void _transmitPacket(detail::StreamSource *src, const ftl::codecs::Packet &pkt, ftl::codecs::Channel chan, bool hasChan2, Quality q);
	void _transmitPacket(detail::StreamSource *src, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt, Quality q);

	//void _encodeHQAndTransmit(detail::StreamSource *src, const cv::Mat &, const cv::Mat &, int chunk);
	//void _encodeLQAndTransmit(detail::StreamSource *src, const cv::Mat &, const cv::Mat &, int chunk);
	//void _encodeAndTransmit(detail::StreamSource *src, ftl::codecs::Encoder *enc1, ftl::codecs::Encoder *enc2, const cv::Mat &, const cv::Mat &);
	//void _encodeImageChannel1(const cv::Mat &in, std::vector<unsigned char> &out, unsigned int b);
	//bool _encodeImageChannel2(const cv::Mat &in, std::vector<unsigned char> &out, ftl::codecs::Channel_t c, unsigned int b);
};

}
}

#endif  // _FTL_RGBD_STREAMER_HPP_
