#ifndef _FTL_RENDER_DETAIL_SOURCE_HPP_
#define _FTL_RENDER_DETAIL_SOURCE_HPP_

#include <Eigen/Eigen>
#include <ftl/cuda_util.hpp>
#include <ftl/rgbd/camera.hpp>
#include <ftl/rgbd/frame.hpp>
#include <ftl/audio/mixer.hpp>

namespace ftl{
namespace render {

class Source;

class BaseSourceImpl {
	public:
	friend class ftl::render::Source;

	public:
	explicit BaseSourceImpl(ftl::render::Source *host) : host_(host) { }
	virtual ~BaseSourceImpl() {}

	virtual bool capture(int64_t ts)=0;

	virtual bool retrieve(ftl::data::Frame &frame)=0;

	virtual bool isReady() { return false; };

	ftl::render::Source *host() { return host_; }

	inline ftl::audio::StereoMixerF<100> &mixer() { return mixer_; }

	virtual ftl::stream::Feed::Filter *filter()=0;

	protected:
	ftl::render::Source *host_;
	ftl::audio::StereoMixerF<100> mixer_;  // TODO: Remove
};

}
}

#endif  // _FTL_RENDER_DETAIL_SOURCE_HPP_
