#ifndef _FTL_RECONSTRUCTION_HPP_
#define _FTL_RECONSTRUCTION_HPP_

#include "ftl/configurable.hpp"
#include "ftl/rgbd/source.hpp"
#include "ftl/rgbd/frame.hpp"
#include "ftl/rgbd/group.hpp"
#include "ftl/rgbd/frameset.hpp"
#include "ftl/operators/operator.hpp"
#include "ftl/render/tri_render.hpp"

namespace ftl {

class Reconstruction : public ftl::Configurable {
	public:
	Reconstruction(nlohmann::json &config, const std::string name);
	~Reconstruction();

	void addSource(ftl::rgbd::Source *);

	void addRawCallback(const std::function<void(ftl::rgbd::Source *src, const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt)> &cb);

	/**
	 * Do the render for a specified virtual camera.
	 */
	bool render(ftl::rgbd::VirtualSource *vs, ftl::rgbd::Frame &out);

	private:
	bool busy_;
	bool rbusy_;
	bool new_frame_;
	MUTEX exchange_mtx_;
	
	ftl::rgbd::FrameSet fs_render_;
	ftl::rgbd::FrameSet fs_align_;
	ftl::rgbd::Group *group_;
	ftl::operators::Graph *pipeline_;
	ftl::render::Triangular *renderer_;

	std::vector<cv::cuda::GpuMat> rgb_;
};

}

#endif  // _FTL_RECONSTRUCTION_HPP_
