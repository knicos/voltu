#ifndef _FTL_RENDER_SCREEN_SOURCE_HPP_
#define _FTL_RENDER_SCREEN_SOURCE_HPP_

#include <ftl/data/creators.hpp>
#include <ftl/data/new_frameset.hpp>
#include <ftl/render/renderer.hpp>
#include <ftl/render/GLRender.hpp>
#include <ftl/streams/feed.hpp>

#include "../baserender.hpp"

namespace ftl {
namespace render {

/**
 * Wrap a renderer into a source entity that manages it. This obtains the
 * relevant framesets and can be triggered by a builder to generate frames.
 */
class ScreenRender : public ftl::render::BaseSourceImpl {
    public:
    ScreenRender(ftl::render::Source *host, ftl::stream::Feed *feed);
	~ScreenRender();

    bool capture(int64_t ts) override;
	bool retrieve(ftl::data::Frame &) override;

	bool isReady() override;

	ftl::stream::Feed::Filter *filter() override { return filter_; }

	private:
	ftl::stream::Feed *feed_;
	ftl::stream::Feed::Filter *filter_;
	ftl::data::FrameSetPtr input_;
	std::unique_ptr<ftl::render::FSRenderer> renderer_;
	ftl::Configurable *intrinsics_;
	uint32_t my_id_;
	ftl::operators::Graph *post_pipe_;
	std::atomic_flag calibration_uptodate_;
	std::string rendtype_;

	/*struct AudioMixerMapping {
		int64_t last_timestamp=0;
		int track=-1;
	};

	std::unordered_map<uint32_t, AudioMixerMapping> mixmap_;*/

	void _createGLRender();
	void _createCUDARender();
	void _checkRenderer();
};

}
}

#endif
