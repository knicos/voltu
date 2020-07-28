#ifndef _FTL_AUDIO_SOFTWARE_ENCODER_HPP_
#define _FTL_AUDIO_SOFTWARE_ENCODER_HPP_

#include <ftl/audio/encoder.hpp>

struct OpusMSEncoder;

namespace ftl {
namespace audio {

class SoftwareEncoder : public ftl::audio::Encoder {
	public:
	SoftwareEncoder();
	~SoftwareEncoder();

	bool encode(const std::vector<float> &in, ftl::codecs::Packet &pkt) override;

	void reset() override;

	bool supports(ftl::codecs::codec_t codec) override;

	private:
	OpusMSEncoder *opus_encoder_;
	bool cur_stereo_;
	ftl::codecs::definition_t cur_definition_;
	uint8_t cur_bitrate_;

	bool _encodeRaw(const std::vector<float> &in, ftl::codecs::Packet &pkt);
	bool _encodeOpus(const std::vector<float> &in, ftl::codecs::Packet &pkt);
	bool _createOpus(ftl::codecs::Packet &pkt);
};

}
}

#endif
