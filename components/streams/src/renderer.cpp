#include <ftl/streams/renderer.hpp>
#include <ftl/rgbd/frame.hpp>
#include <ftl/rgbd/frameset.hpp>
#include <ftl/rgbd/capabilities.hpp>
#include <loguru.hpp>

#include "./renderers/screen_render.hpp"
#include "./renderers/openvr_render.hpp"

using ftl::render::Source;
using ftl::codecs::Channel;
using ftl::rgbd::Capability;
using std::string;


Source::Source(nlohmann::json &config, ftl::stream::Feed *feed)
: ftl::Configurable(config), feed_(feed), impl_(nullptr) {
	reset();

	on("uri", [this]() {
		LOG(INFO) << "URI change for renderer: " << getURI();
		reset();
	});
}

Source::~Source() {
	if (impl_) delete impl_;
}

ftl::audio::StereoMixerF<100> &Source::mixer() {
	if (!impl_) throw FTL_Error("No implementation");
	return impl_->mixer();
}

bool Source::supports(const std::string &puri) {
	ftl::URI uri(puri);
	if (!uri.isValid() || uri.getScheme() != ftl::URI::SCHEME_DEVICE) return false;

	if (uri.getPathSegment(0) == "render") return true;
	if (uri.getPathSegment(0) == "openvr") return ftl::render::OpenVRRender::supported();
	return false;
}

void Source::reset() {
	if (impl_) delete impl_;
	impl_ = nullptr;

	auto uristr = get<string>("uri");
	if (!uristr) return;

	restore(*uristr, {
		"renderer",
		"source",
		"intrinsics",
		"name"
	});

	ftl::URI uri(*uristr);
	if (!uri.isValid()) return;
	if (uri.getScheme() != ftl::URI::SCHEME_DEVICE) return;

	if (uri.getPathSegment(0) == "render") {
		impl_ = new ftl::render::ScreenRender(this, feed_);
	} else if (uri.getPathSegment(0) == "openvr") {
		impl_ = new ftl::render::OpenVRRender(this, feed_);
	} else {
		throw FTL_Error("Invalid render device: " << *uristr);
	}
}

bool Source::capture(int64_t ts) {
	if (impl_) return impl_->capture(ts);
	else return false;
}

bool Source::retrieve(ftl::data::Frame &frame_out) {
	if (impl_) return impl_->retrieve(frame_out);
	else return false;
}
