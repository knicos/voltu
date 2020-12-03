/**
 * @file decoder.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_AUDIO_DECODER_HPP_
#define _FTL_AUDIO_DECODER_HPP_

#include <vector>
#include <ftl/codecs/packet.hpp>
#include <ftl/codecs/codecs.hpp>

namespace ftl {
namespace audio {

/**
 * Abstract audio decoder class. Specific codec implementations inherit this
 * base class, the codec being determined from the `Packet` structure.
 */
class Decoder {
	public:
	Decoder() { };
	virtual ~Decoder() { };

	/**
	 * Convert an encoded `Packet` to a raw decoded audio frame(s). The output
	 * is always a multiple of a fixed frame size.
	 * 
	 * @param pkt Encoded packet structure.
	 * @param out Vector to populate with decoded audio frames.
	 */
	virtual bool decode(const ftl::codecs::Packet &pkt, std::vector<float> &out)=0;

	/**
	 * Check if a decoder instance can decode a packet.
	 */
	virtual bool accepts(const ftl::codecs::Packet &)=0;
};

}
}

#endif