#ifndef _FTL_SYNCHED_HPP_
#define _FTL_SYNCHED_HPP_

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace ftl {

static const int LEFT = 0;
static const int RIGHT = 1;

class SyncSource {
	public:
	SyncSource();
	
	void addChannel(const std::string &c);
	void feed(int channel, cv::Mat &m, double ts);
	bool get(int channel, cv::Mat &m);
	double latency() const;
	
	private:
	std::vector<cv::Mat> channels_;
};
};

#endif // _FTL_SYNCHED_HPP_

