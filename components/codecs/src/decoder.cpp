/**
 * @file decoder.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#include <ftl/codecs/decoder.hpp>

#include <ftl/codecs/opencv_decoder.hpp>
#include <ftl/codecs/nvidia_decoder.hpp>

using ftl::codecs::Decoder;
using ftl::codecs::codec_t;
using std::string;
using std::to_string;

/* Debug utility function to print stream packets */
ftl::codecs::StreamPacket::operator std::string() const {
	return string("[\n  timestamp=") + to_string(timestamp) + string(",\n  frameset=") +
		to_string(streamID) + string(",\n  frame=") + to_string(frame_number) +
		string(",\n  channel=") + to_string((int)channel) + string("\n]");
}

Decoder *ftl::codecs::allocateDecoder(const ftl::codecs::Packet &pkt) {
	// TODO: (nick) Add other decoder libraries and codecs.
	switch(pkt.codec) {
	case codec_t::JPG		:
	case codec_t::PNG		: return new ftl::codecs::OpenCVDecoder;
	case codec_t::HEVC_LOSSLESS:
	case codec_t::H264_LOSSLESS:
	case codec_t::H264		:
	case codec_t::HEVC		: return new ftl::codecs::NvidiaDecoder;
	default					: return nullptr;
	}
}

void ftl::codecs::free(Decoder *&e) {
	delete e;
	e = nullptr;
}
