#ifndef _FTL_RENDER_SOURCE_HPP_
#define _FTL_RENDER_SOURCE_HPP_

#include <ftl/data/creators.hpp>
#include <ftl/data/new_frameset.hpp>
#include <ftl/render/renderer.hpp>
#include <ftl/render/CUDARender.hpp>
#include <ftl/streams/feed.hpp>
#include <ftl/audio/mixer.hpp>

namespace ftl {
namespace render {

class BaseSourceImpl;

/**
 * Wrap a renderer into a source entity that manages it. This obtains the
 * relevant framesets and can be triggered by a builder to generate frames.
 */
class Source : public ftl::Configurable, public ftl::data::DiscreteSource {
    public:
    Source(nlohmann::json &, ftl::stream::Feed*);
	~Source();

	inline std::string getURI() { return value("uri", std::string("")); }

    bool capture(int64_t ts) override;
	bool retrieve(ftl::data::Frame &) override;

	static bool supports(const std::string &uri);

	ftl::audio::StereoMixerF<100> &mixer();

	ftl::stream::Feed::Filter *filter() const;

	private:
	ftl::stream::Feed *feed_;
	ftl::render::BaseSourceImpl *impl_;

	void reset();
};

}
}

#endif
