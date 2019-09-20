#include <ftl/codecs/decoder.hpp>

#include <ftl/codecs/opencv_decoder.hpp>
#include <ftl/codecs/nvpipe_decoder.hpp>

using ftl::codecs::Decoder;
using ftl::codecs::codec_t;

Decoder *ftl::codecs::allocateDecoder(const ftl::codecs::Packet &pkt) {
	switch(pkt.codec) {
	case codec_t::JPG		:
	case codec_t::PNG		: return new ftl::codecs::OpenCVDecoder;
	case codec_t::HEVC		: return new ftl::codecs::NvPipeDecoder;
	}

	return nullptr;
}

/**
 * Release a decoder to be reused by some other stream.
 */
void ftl::codecs::free(Decoder *&e) {
	delete e;
	e = nullptr;
}
