#ifndef _FTL_RGBD_STREAMER_HPP_
#define _FTL_RGBD_STREAMER_HPP_

#include <ctpl_stl.h>
#include <loguru.hpp>
#include <ftl/configuration.hpp>
#include <ftl/configurable.hpp>
#include <ftl/rgbd_source.hpp>
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
	ftl::rgbd::RGBDSource *src;
	unsigned int state;								// Busy or ready to swap?
	cv::Mat rgb;									// Tx buffer
	cv::Mat depth;									// Tx buffer
	std::list<detail::StreamClient> clients[10];	// One list per bitrate
	std::shared_mutex mutex;
};

}

static const int kMaxFrames = 25;

class Streamer : public ftl::Configurable {
	public:
	Streamer(nlohmann::json &config, ftl::net::Universe *net);
	~Streamer();

	void add(RGBDSource *);
	void remove(RGBDSource *);
	void remove(const std::string &);

	void run(bool block=false);
	void stop();

	RGBDSource *get(const std::string &uri);

	private:
	std::map<std::string, detail::StreamSource*> sources_;
	ctpl::thread_pool pool_;
	std::shared_mutex mutex_;
	bool active_;
	ftl::net::Universe *net_;

	void _schedule();
	void _swap(detail::StreamSource &);
	void _addClient(const std::string &source, int N, int rate, const ftl::UUID &peer, const std::string &dest);
};

}
}

#endif  // _FTL_RGBD_STREAMER_HPP_
