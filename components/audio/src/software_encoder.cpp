#include <ftl/audio/software_encoder.hpp>
#include <ftl/config.h>

#ifdef HAVE_OPUS
#include <opus/opus_multistream.h>
#else
struct OpusMSEncoder {};
#endif

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::audio::SoftwareEncoder;
using ftl::codecs::codec_t;

#define FRAME_SIZE 960
#define MAX_PACKET_SIZE (3*2*FRAME_SIZE)

SoftwareEncoder::SoftwareEncoder() : ftl::audio::Encoder(), opus_encoder_(nullptr), cur_bitrate_(0) {

}

SoftwareEncoder::~SoftwareEncoder() {

}

bool SoftwareEncoder::encode(const std::vector<short> &in, ftl::codecs::Packet &pkt) {
	auto codec = (pkt.codec == codec_t::Any) ? codec_t::OPUS : pkt.codec;

	// Force RAW if no opus
	#ifndef HAVE_OPUS
	codec = codec_t::RAW;
	#endif

	pkt.codec = codec;

	switch (codec) {
	case codec_t::OPUS		: return _encodeOpus(in, pkt);
	case codec_t::RAW		: return _encodeRaw(in, pkt);
	default: return false;
	}
}

bool SoftwareEncoder::_createOpus(ftl::codecs::Packet &pkt) {
	#ifdef HAVE_OPUS
	bool stereo = pkt.flags & ftl::codecs::kFlagStereo;
	if (pkt.definition == cur_definition_ && stereo == cur_stereo_ && opus_encoder_) return true;

	cur_definition_ = pkt.definition;
	cur_stereo_ = stereo;

	if (opus_encoder_) {
		opus_multistream_encoder_destroy(opus_encoder_);
		opus_encoder_ = nullptr;
	}

	int sample_rate;
	switch (pkt.definition) {
	case ftl::codecs::definition_t::hz48000		: sample_rate = 48000; break;
	case ftl::codecs::definition_t::hz44100		: sample_rate = 44100; break;
	default: return false;
	}

	int errcode = 0;
	int channels = (stereo) ? 2 : 1;
	const unsigned char mapping[2] = {0,1};
	opus_encoder_ = opus_multistream_encoder_create(sample_rate, channels, 1, channels-1, mapping, OPUS_APPLICATION_VOIP, &errcode);

	if (errcode < 0) return false;
	LOG(INFO) << "Created OPUS encoder";
	#endif

	return true;
}

bool SoftwareEncoder::_encodeOpus(const std::vector<short> &in, ftl::codecs::Packet &pkt) {
	#ifdef HAVE_OPUS
	static const float MAX_BITRATE = 128000.0f;
	static const float MIN_BITRATE = 24000.0f;

	if (!_createOpus(pkt)) return false;

	if (pkt.bitrate != cur_bitrate_) {
		int bitrate = (MAX_BITRATE-MIN_BITRATE) * (float(pkt.bitrate)/255.0f) + MIN_BITRATE;
		if (!cur_stereo_) bitrate /= 2;
		int errcode = opus_multistream_encoder_ctl(opus_encoder_, OPUS_SET_BITRATE(bitrate));
		if (errcode < 0) return false;
		LOG(INFO) << "OPUS encoder: bitrate = " << bitrate;
		cur_bitrate_ = pkt.bitrate;
	}

	int channels = (cur_stereo_) ? 2 : 1;

	pkt.data.resize(MAX_PACKET_SIZE);
	int count = 0;
	int frames = 0;

	unsigned char *outptr = pkt.data.data();

	for (unsigned int i=0; i<in.size(); i+=channels*FRAME_SIZE) {
		short *len = (short*)outptr;
		outptr += 2;
		int nbBytes = opus_multistream_encode(opus_encoder_, &in.data()[i], FRAME_SIZE, outptr, MAX_PACKET_SIZE);
		if (nbBytes <= 0) return false;

		//if (nbBytes > 32000) LOG(WARNING) << "Packet exceeds size limit";

		*len = nbBytes;

		count += nbBytes+2;
		outptr += nbBytes;
		++frames;
	}

	pkt.data.resize(count);
	//LOG(INFO) << "Opus Encode = " << pkt.data.size() << ", " << frames;
	return true;

	#else
	return false;
	#endif
}

bool SoftwareEncoder::_encodeRaw(const std::vector<short> &in, ftl::codecs::Packet &pkt) {
	const unsigned char *ptr = (unsigned char*)in.data();
	pkt.data = std::move(std::vector<unsigned char>(ptr, ptr+in.size()*sizeof(short)));
	return true;
}

void SoftwareEncoder::reset() {

}

bool SoftwareEncoder::supports(ftl::codecs::codec_t codec) {
	return false;
}
