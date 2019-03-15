#ifndef _FTL_RTCENSUS_HPP_
#define _FTL_RTCENSUS_HPP_

namespace ftl {
class RTCensus {
	public:
	void disparity(cv::Mat &l, cv::Mat &r, cv::Mat &disp, size_t num_disp=32, float gamma=0.0f, float tau=0.0f);
};
};

#endif // _FTL_RTCENSUS_HPP_

