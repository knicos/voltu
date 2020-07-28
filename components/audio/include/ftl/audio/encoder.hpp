#ifndef _FTL_AUDIO_ENCODER_HPP_
#define _FTL_AUDIO_ENCODER_HPP_

#include <vector>
#include <ftl/codecs/packet.hpp>
#include <ftl/codecs/codecs.hpp>

namespace ftl {
namespace audio {

class Encoder {
	public:
	Encoder() {};
	virtual ~Encoder() {};

	virtual bool encode(const std::vector<float> &in, ftl::codecs::Packet &pkt)=0;

	virtual void reset() {}

	virtual bool supports(ftl::codecs::codec_t codec)=0;
};

}
}

#endif
