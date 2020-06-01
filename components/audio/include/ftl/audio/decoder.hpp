#ifndef _FTL_AUDIO_DECODER_HPP_
#define _FTL_AUDIO_DECODER_HPP_

#include <vector>
#include <ftl/codecs/packet.hpp>
#include <ftl/codecs/codecs.hpp>

namespace ftl {
namespace audio {

class Decoder {
	public:
	Decoder() { };
	virtual ~Decoder() { };

	virtual bool decode(const ftl::codecs::Packet &pkt, std::vector<short> &out)=0;

	virtual bool accepts(const ftl::codecs::Packet &)=0;
};

}
}

#endif