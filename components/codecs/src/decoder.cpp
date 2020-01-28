#include <ftl/codecs/decoder.hpp>

#include <ftl/codecs/opencv_decoder.hpp>
#include <ftl/codecs/nvpipe_decoder.hpp>

using ftl::codecs::Decoder;
using ftl::codecs::codec_t;
using std::string;
using std::to_string;

ftl::codecs::StreamPacket::operator std::string() const {
	return string("[\n  timestamp=") + to_string(timestamp) + string(",\n  frameset=") +
		to_string(streamID) + string(",\n  frame=") + to_string(frame_number) +
		string(",\n  channel=") + to_string((int)channel) + string("\n]");
}

Decoder *ftl::codecs::allocateDecoder(const ftl::codecs::Packet &pkt) {
	switch(pkt.codec) {
	case codec_t::JPG		:
	case codec_t::PNG		: return new ftl::codecs::OpenCVDecoder;
	case codec_t::HEVC_LOSSLESS:
	case codec_t::H264_LOSSLESS:
	case codec_t::H264		:
	case codec_t::HEVC		: return new ftl::codecs::NvPipeDecoder;
	default					: return nullptr;
	}
}

/**
 * Release a decoder to be reused by some other stream.
 */
void ftl::codecs::free(Decoder *&e) {
	delete e;
	e = nullptr;
}
