#ifndef _FTL_RGBD_STREAMER_HPP_
#define _FTL_RGBD_STREAMER_HPP_

#include <ctpl_stl.h>
#include <loguru.hpp>
#include <ftl/configuration.hpp>
#include <ftl/configurable.hpp>
#include <ftl/rgbd/source.hpp>
#include <ftl/net/universe.hpp>
#include <string>
#include <list>
#include <map>
#include <shared_mutex>

namespace ftl {
namespace rgbd {

namespace detail {

struct StreamClient {
	std::string uri;
	ftl::UUID peerid;
	int txcount;		// Frames sent since last request
	int txmax;			// Frames to send in request
};

static const unsigned int kGrabbed = 0x1;
static const unsigned int kTransmitted = 0x2; 

struct StreamSource {
	ftl::rgbd::Source *src;
	unsigned int state;								// Busy or ready to swap?
	cv::Mat rgb;									// Tx buffer
	cv::Mat depth;									// Tx buffer
	std::list<detail::StreamClient> clients[10];	// One list per bitrate
	std::shared_mutex mutex;
};

}

/**
 * The maximum number of frames a client can request in a single request.
 */
static const int kMaxFrames = 25;

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

	/**
	 * Alternative to calling run(), it will operate a single frame capture,
	 * compress and stream cycle.
	 */
	void poll();

	Source *get(const std::string &uri);

	private:
	std::map<std::string, detail::StreamSource*> sources_;
	ctpl::thread_pool pool_;
	std::shared_mutex mutex_;
	bool active_;
	ftl::net::Universe *net_;
	bool late_;

	void _schedule();
	void _swap(detail::StreamSource &);
	void _addClient(const std::string &source, int N, int rate, const ftl::UUID &peer, const std::string &dest);
};

}
}

#endif  // _FTL_RGBD_STREAMER_HPP_