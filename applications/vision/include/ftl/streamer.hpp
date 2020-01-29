#ifndef _FTL_STREAMER_HPP_
#define _FTL_STREAMER_HPP_

#include <ftl/net/universe.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/core/mat.hpp>
#include <string>

namespace ftl {

class Streamer {
	public:
	Streamer(ftl::net::Universe &net, nlohmann::json &config);
	~Streamer();
	
	void send(const cv::Mat &rgb, const cv::Mat &depth);
	
	private:
	ftl::net::Universe &net_;
	nlohmann::json config_;
	std::string uri_;
};

};

#endif  // _FTL_STREAMER_HPP_

