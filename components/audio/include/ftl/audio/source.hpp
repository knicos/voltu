#ifndef _FTL_AUDIO_SOURCE_HPP_
#define _FTL_AUDIO_SOURCE_HPP_

#include <ftl/audio/buffer.hpp>
#include <ftl/audio/frameset.hpp>
#include <ftl/configurable.hpp>
#include <ftl/config.h>

#ifdef HAVE_PORTAUDIO
#include <portaudio.h>
#endif

namespace ftl {
namespace audio {

static constexpr int kFrameSize = 960;

typedef ftl::data::Generator<ftl::audio::FrameSet> Generator;

class Source : public ftl::Configurable, public ftl::audio::Generator {
    public:
    explicit Source(nlohmann::json &config);
    ~Source();

    /** Number of frames in last frameset. This can change over time. */
	size_t size() override;

	/**
	 * Get the persistent state object for a frame. An exception is thrown
	 * for a bad index.
	 */
	ftl::audio::FrameState &state(size_t ix) override;

	/** Register a callback to receive new frame sets. */
	void onFrameSet(const ftl::audio::FrameSet::Callback &) override;

    private:
    ftl::audio::FrameState state_;
    bool active_;
    ftl::timer::TimerHandle timer_hp_;
	ftl::timer::TimerHandle timer_main_;
	ftl::audio::FrameSet::Callback cb_;

	ftl::audio::Buffer<short> *buffer_;
	int to_read_;
	int64_t latency_;

	ftl::audio::FrameSet frameset_;

	#ifdef HAVE_PORTAUDIO
	PaStream *stream_;
	#endif
};

}
}

#endif  // _FTL_AUDIO_SOURCE_HPP_
