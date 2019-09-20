#ifndef _FTL_RGBD_NETFRAME_HPP_
#define _FTL_RGBD_NETFRAME_HPP_

#include <cstdint>
#include <vector>
#include <ftl/rgbd/source.hpp>

namespace ftl {
namespace rgbd {
namespace detail {

/**
 * Buffers for a single frame as it is being received over the network.
 * Also maintains statistics about the frame transmission for later analysis.
 */
struct NetFrame {
	cv::Mat channel1;
	cv::Mat channel2;
	volatile int64_t timestamp;
	std::atomic<int> chunk_count;
	int chunk_total;
	std::atomic<int> tx_size;
	int64_t tx_latency;
	MUTEX mtx;
};

/**
 * Manage multiple frames with their timestamp as an identifier. Once a frame
 * is completed it should be freed immediately from the queue for reuse. It
 * is not the job of this queue to buffer frames for longer periods, see Group
 * for this functionality. This queue is only to manage chunk ordering problems.
 */
class NetFrameQueue {
	public:
	explicit NetFrameQueue(int size=2);
	~NetFrameQueue();

	NetFrame &getFrame(int64_t ts, const cv::Size &, int c1type, int c2type);
	void freeFrame(NetFrame &);

	private:
	std::vector<NetFrame> frames_;
	MUTEX mtx_;
};

}
}
}

#endif  // _FTL_RGBD_NETFRAME_HPP_
