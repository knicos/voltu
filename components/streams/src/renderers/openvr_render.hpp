#ifndef _FTL_RENDER_OPENVR_SOURCE_HPP_
#define _FTL_RENDER_OPENVR_SOURCE_HPP_

#include <ftl/data/creators.hpp>
#include <ftl/data/new_frameset.hpp>
#include <ftl/render/renderer.hpp>
#include <ftl/render/CUDARender.hpp>
#include <ftl/streams/feed.hpp>
#include <ftl/utility/gltexture.hpp>
#include <ftl/audio/mixer.hpp>

#include "../baserender.hpp"

#include <ftl/config.h>

#ifdef HAVE_OPENVR
#include <openvr/openvr.h>
#endif

namespace ftl {
namespace render {

class OpenVRRender : public ftl::render::BaseSourceImpl {
    public:
    OpenVRRender(ftl::render::Source *host, ftl::stream::Feed *feed);
	~OpenVRRender();

    bool capture(int64_t ts) override;
	bool retrieve(ftl::data::Frame &) override;

	bool isReady() override;

	static bool supported();

	private:
	ftl::stream::Feed *feed_;
	ftl::stream::Feed::Filter *filter_;
	ftl::data::FrameSetPtr input_;
	std::unique_ptr<ftl::render::CUDARender> renderer_;
	std::unique_ptr<ftl::render::CUDARender> renderer2_;
	ftl::Configurable *intrinsics_;
	uint32_t my_id_;

	ftl::operators::Graph *post_pipe_;

	std::atomic_flag pose_calibrated_;

	float baseline_;
	Eigen::Matrix4d initial_pose_;
	Eigen::Matrix4d rotmat_;
	Eigen::Vector3d eye_;
	ftl::utility::GLTexture texture1_; // first channel (always left at the moment)
	ftl::utility::GLTexture texture2_;

	#ifdef HAVE_OPENVR
	vr::TrackedDevicePose_t rTrackedDevicePose_[ vr::k_unMaxTrackedDeviceCount ];
	#endif

	struct AudioMixerMapping {
		int64_t last_timestamp=0;
		int track=-1;
	};

	std::unordered_map<uint32_t, AudioMixerMapping> mixmap_;

	bool initVR();
};

}
}

#endif
