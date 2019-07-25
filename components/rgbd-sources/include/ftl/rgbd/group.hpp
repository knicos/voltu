#ifndef _FTL_RGBD_GROUP_HPP_
#define _FTL_RGBD_GROUP_HPP_

#include <ftl/threads.hpp>

#include <opencv2/opencv.hpp>
#include <vector>

namespace ftl {
namespace rgbd {

class Source;

struct FrameSet {
	int64_t timestamp;
	std::vector<Source*> sources;
	std::vector<cv::Mat> channel1;
	std::vector<cv::Mat> channel2;
	int count;
	unsigned int mask;
};

static const size_t kFrameBufferSize = 10;

class Group {
	public:
	Group();
	~Group();

	void addSource(ftl::rgbd::Source *);

	void sync(int N=-1, int B=-1);
	void sync(std::function<bool(const FrameSet &)>);

	bool getFrames(FrameSet &, bool complete=false);

	private:
	std::vector<FrameSet> framesets_;
	std::vector<Source*> sources_;
	size_t head_;
	std::function<bool(const FrameSet &)> callback_;
	MUTEX mutex_;

	void _addFrameset(int64_t timestamp);
};

}
}

#endif  // _FTL_RGBD_GROUP_HPP_
