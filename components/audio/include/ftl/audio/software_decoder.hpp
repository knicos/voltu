#ifndef _FTL_AUDIO_SOFTWARE_DECODER_HPP_
#define _FTL_AUDIO_SOFTWARE_DECODER_HPP_

#include <ftl/audio/decoder.hpp>

struct OpusMSDecoder;

namespace ftl {
namespace audio {

class SoftwareDecoder : public ftl::audio::Decoder {
	public:
	SoftwareDecoder();
	~SoftwareDecoder();

	bool decode(const ftl::codecs::Packet &pkt, std::vector<short> &out) override;

	bool accepts(const ftl::codecs::Packet &) override;

	private:
	OpusMSDecoder *opus_decoder_;
	bool cur_stereo_;
	ftl::codecs::definition_t cur_definition_;

	bool _decodeOpus(const ftl::codecs::Packet &pkt, std::vector<short> &out);
	bool _decodeRaw(const ftl::codecs::Packet &pkt, std::vector<short> &out);
	bool _createOpus(const ftl::codecs::Packet &pkt);
};

}
}

#endif
