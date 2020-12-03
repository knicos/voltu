/**
 * @file encoder.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_AUDIO_ENCODER_HPP_
#define _FTL_AUDIO_ENCODER_HPP_

#include <vector>
#include <ftl/codecs/packet.hpp>
#include <ftl/codecs/codecs.hpp>

namespace ftl {
namespace audio {

/**
 * Abstract base class for an audio encoder.
 */
class Encoder {
	public:
	Encoder() {};
	virtual ~Encoder() {};

	/**
	 * Encode one or more audio frames into a data packet.
	 * 
	 * @param in Audio frame data, must be a multiple of frame size.
	 * @param pkt `Packet` to populate with codec info and encoded data.
	 */
	virtual bool encode(const std::vector<float> &in, ftl::codecs::Packet &pkt)=0;

	virtual void reset() {}

	virtual bool supports(ftl::codecs::codec_t codec)=0;
};

}
}

#endif
