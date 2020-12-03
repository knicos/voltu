/**
 * @file software_decoder.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#include <ftl/audio/software_decoder.hpp>
#include <ftl/config.h>

#ifdef HAVE_OPUS
#include <opus/opus_multistream.h>
#else
struct OpusMSDecoder {};
#endif

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

#define FRAME_SIZE 960

using ftl::audio::SoftwareDecoder;
using ftl::codecs::codec_t;

SoftwareDecoder::SoftwareDecoder() : opus_decoder_(nullptr) {

}

SoftwareDecoder::~SoftwareDecoder() {

}

bool SoftwareDecoder::_createOpus(const ftl::codecs::Packet &pkt) {
	#ifdef HAVE_OPUS
	bool stereo = pkt.flags & ftl::codecs::kFlagStereo;

	// If the current decoder matches, don't create a new one
	if (opus_decoder_ && stereo == cur_stereo_) return true;

	cur_stereo_ = stereo;

	// Existing decoder is invalid so destroy it first
	if (opus_decoder_) {
		opus_multistream_decoder_destroy(opus_decoder_);
		opus_decoder_ = nullptr;
	}

	int sample_rate = 48000;  // TODO: Allow it to be different

	int errcode = 0;
	int channels = (stereo) ? 2 : 1;
	const unsigned char mapping[2] = {0,1};
	opus_decoder_ = opus_multistream_decoder_create(sample_rate, channels, 1, channels-1, mapping, &errcode);

	if (errcode < 0) return false;

	LOG(INFO) << "Created OPUS decoder: " << sample_rate << ", " << channels;
	#endif
	return true;
}

bool SoftwareDecoder::decode(const ftl::codecs::Packet &pkt, std::vector<float> &out) {
	switch (pkt.codec) {
	case codec_t::OPUS		: return _decodeOpus(pkt, out);
	case codec_t::RAW		: return _decodeRaw(pkt, out);
	default: return false;
	}
}

bool SoftwareDecoder::_decodeOpus(const ftl::codecs::Packet &pkt, std::vector<float> &out) {
	#ifdef HAVE_OPUS
	if (!_createOpus(pkt)) return false;

	int channels = (cur_stereo_) ? 2 : 1;

	// FIXME: (nick) Find another way to allow for more than 10 audio frames
	out.resize(10*FRAME_SIZE*channels);

	const unsigned char *inptr = pkt.data.data();
	float *outptr = out.data();
	int count = 0;
	int frames = 0;

	for (size_t i=0; i<pkt.data.size(); ) {
		// First 2 bytes indicate encoded frame size
		const short *len = (const short*)inptr;
		if (*len == 0) break;
		if (frames == 10) break; // Max frames reached

		inptr += 2;
		i += (*len)+2;
		int samples = opus_multistream_decode_float(opus_decoder_, inptr, *len, outptr, FRAME_SIZE, 0);

		if (samples != FRAME_SIZE) {
			LOG(ERROR) << "Failed to Opus decode: " << samples;
			break;
		}

		inptr += *len;
		outptr += FRAME_SIZE*channels;
		count += samples;
		++frames;
	}

	// Shrink back down to fit actual data received.
	out.resize(count*channels);
	return true;

	#else
	LOG(WARNING) << "No Opus decoder installed";
	return false;
	#endif
}

bool SoftwareDecoder::_decodeRaw(const ftl::codecs::Packet &pkt, std::vector<float> &out) {
	size_t size = pkt.data.size()/sizeof(float);
	out.resize(size);
	auto *ptr = (float*)pkt.data.data();
	for (size_t i=0; i<size; i++) out.data()[i] = ptr[i];
	return true;
}

bool SoftwareDecoder::accepts(const ftl::codecs::Packet &) {
	// TODO: Implement if ever needed
	return false;
}
