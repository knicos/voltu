#ifndef _FTL_RGBD_GROUP_HPP_
#define _FTL_RGBD_GROUP_HPP_

#include <ftl/cuda_util.hpp>
#include <ftl/threads.hpp>
#include <ftl/timer.hpp>
#include <ftl/rgbd/frame.hpp>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/codecs/packet.hpp>

//#include <opencv2/opencv.hpp>
#include <vector>

namespace ftl {
namespace operators {
	class Graph;
}

namespace rgbd {

class Source;

/**
 * Manage a group of RGB-D sources to obtain synchronised sets of frames from
 * those sources. The Group class provides a synchronised callback mechanism
 * that uses the high precision timer to ensure that it is called once per
 * frame. The callback is not called if the frameset is not completed or
 * is unavailable for some other reason. By default if the required frame is
 * not available but there is an older frame available that has not been used
 * then it will be used. This can be disabled. It is also possible to allow
 * incomplete frames to be used, but this is disabled by default.
 */
class Group : public ftl::rgbd::Generator {
	public:
	Group();
	~Group();

	/**
	 * Give this group a name for logging purposes.
	 */
	void setName(const std::string &name);

	/**
	 * Add a new source to the group. Framesets generated prior to the source
	 * being added will still be valid and will not contain a frame from this
	 * source. Sets generated after addition will require a frame from this
	 * source.
	 */
	void addSource(ftl::rgbd::Source *);

	/**
	 * Add another group to this one. All sources in the other group are made
	 * available to this group in a synchronised way. There is additional
	 * overhead in supporting this as additional data copies are required
	 * internally for all the source frames.
	 */
	void addGroup(ftl::rgbd::Group *);

	/**
	 * Add a pipeline to be run after each frame is received from source but
	 * before it as added to a synchronised frameset.
	 */
	void addPipeline(ftl::operators::Graph *g) { pipeline_ = g; };

	/**
	 * Provide a function to be called once per frame with a valid frameset
	 * at the specified latency. The function may not be called under certain
	 * conditions (missing frameset). No guarantee is made about the timing
	 * accuracy of the call, it should be close to the frame point. This
	 * function may or may not block. It is intended that the buffers within
	 * the frameset are swapped during the function call, meaning that the
	 * frameset data is no longer valid upon returning.
	 */
	void onFrameSet(const VideoCallback &cb) override;

	/**
	 * Whenever any source within the group receives raw data, this callback
	 * will be called with that raw data. This is used to allow direct data
	 * capture (to disk) or proxy over a network without needing to re-encode.
	 * There is no guarantee about order or timing and the callback itself will
	 * need to ensure synchronisation of timestamps.
	 */
	void addRawCallback(const std::function<void(ftl::rgbd::Source*, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)> &);

	/**
	 * Removes a raw data callback from all sources in the group.
	 */
	//void removeRawCallback(const std::function<void(ftl::rgbd::Source*, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)> &);

	inline std::vector<Source*> sources() const { return sources_; }

	size_t size() override { return builder_.size(); }

	ftl::rgbd::FrameState &state(size_t ix) override { return builder_.state(ix); }

	void stop() {}

	int streamID(const ftl::rgbd::Source *s) const;

	private:
	ftl::rgbd::Builder builder_;
	std::vector<Source*> sources_;
	ftl::operators::Graph *pipeline_;
	
	std::atomic<int> jobs_;
	std::atomic<int> cjobs_;
	volatile bool skip_;
	ftl::Handle cap_id_;
	ftl::Handle swap_id_;
	ftl::Handle main_id_;
	std::string name_;
	MUTEX mutex_;

	void _retrieveJob(ftl::rgbd::Source *);
	void _dispatchJob(ftl::rgbd::Source *, int64_t);
};

}
}

#endif  // _FTL_RGBD_GROUP_HPP_
