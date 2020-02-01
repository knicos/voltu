#ifndef _FTL_RECONSTRUCTION_HPP_
#define _FTL_RECONSTRUCTION_HPP_

#include "ftl/configurable.hpp"
#include "ftl/rgbd/source.hpp"
#include "ftl/rgbd/frame.hpp"
#include "ftl/rgbd/group.hpp"
#include "ftl/rgbd/frameset.hpp"
#include "ftl/operators/operator.hpp"

namespace ftl {

class Reconstruction : public ftl::Configurable, public ftl::rgbd::Generator {
	public:
	Reconstruction(nlohmann::json &config, const std::string name);
	~Reconstruction();

	//void addSource(ftl::rgbd::Source *);

	//void addRawCallback(const std::function<void(ftl::rgbd::Source *src, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)> &cb);

	void setGenerator(ftl::rgbd::Generator *);

	/** Number of frames in last frameset. This can change over time. */
	size_t size() override;

	/**
	 * Get the persistent state object for a frame. An exception is thrown
	 * for a bad index.
	 */
	ftl::rgbd::FrameState &state(size_t ix) override;

	/** Register a callback to receive new frame sets. */
	void onFrameSet(const ftl::rgbd::VideoCallback &) override;

	bool post(ftl::rgbd::FrameSet &fs);

	private:
	bool busy_;
	bool rbusy_;
	bool new_frame_;
	MUTEX exchange_mtx_;
	
	ftl::rgbd::FrameSet fs_render_;
	ftl::rgbd::FrameSet fs_align_;
	ftl::rgbd::Generator *gen_;
	ftl::operators::Graph *pipeline_;

	ftl::rgbd::VideoCallback cb_;

	std::vector<cv::cuda::GpuMat> rgb_;
};

}

#endif  // _FTL_RECONSTRUCTION_HPP_
