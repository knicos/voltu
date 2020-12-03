/**
 * @file software_decoder.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_AUDIO_SOFTWARE_DECODER_HPP_
#define _FTL_AUDIO_SOFTWARE_DECODER_HPP_

#include <ftl/audio/decoder.hpp>

struct OpusMSDecoder;

namespace ftl {
namespace audio {

/**
 * Implement the OPUS decoder.
 */
class SoftwareDecoder : public ftl::audio::Decoder {
	public:
	SoftwareDecoder();
	~SoftwareDecoder();

	bool decode(const ftl::codecs::Packet &pkt, std::vector<float> &out) override;

	bool accepts(const ftl::codecs::Packet &) override;

	private:
	OpusMSDecoder *opus_decoder_;
	bool cur_stereo_;

	bool _decodeOpus(const ftl::codecs::Packet &pkt, std::vector<float> &out);
	bool _decodeRaw(const ftl::codecs::Packet &pkt, std::vector<float> &out);
	bool _createOpus(const ftl::codecs::Packet &pkt);
};

}
}

#endif
