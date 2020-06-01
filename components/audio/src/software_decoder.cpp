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
	if (pkt.definition == cur_definition_ && stereo == cur_stereo_ && opus_decoder_) return true;

	cur_definition_ = pkt.definition;
	cur_stereo_ = stereo;

	if (opus_decoder_) {
		opus_multistream_decoder_destroy(opus_decoder_);
		opus_decoder_ = nullptr;
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
	opus_decoder_ = opus_multistream_decoder_create(sample_rate, channels, 1, channels-1, mapping, &errcode);

	if (errcode < 0) return false;

	LOG(INFO) << "Created OPUS decoder: " << sample_rate << ", " << channels;
	#endif
	return true;
}

bool SoftwareDecoder::decode(const ftl::codecs::Packet &pkt, std::vector<short> &out) {
	switch (pkt.codec) {
	case codec_t::OPUS		: return _decodeOpus(pkt, out);
	case codec_t::RAW		: return _decodeRaw(pkt, out);
	default: return false;
	}
}

bool SoftwareDecoder::_decodeOpus(const ftl::codecs::Packet &pkt, std::vector<short> &out) {
	#ifdef HAVE_OPUS
	if (!_createOpus(pkt)) return false;

	int channels = (cur_stereo_) ? 2 : 1;

	out.resize(10*FRAME_SIZE*channels);

	const unsigned char *inptr = pkt.data.data();
	short *outptr = out.data();
	int count = 0;
	int frames = 0;

	for (size_t i=0; i<pkt.data.size(); ) {
		const short *len = (const short*)inptr;
		inptr += 2;
		i += (*len)+2;
		int samples = opus_multistream_decode(opus_decoder_, inptr, *len, outptr, FRAME_SIZE, 0);

		if (samples != FRAME_SIZE) {
			LOG(ERROR) << "Failed to Opus decode: " << samples;
			//return false;
			break;
		}

		inptr += *len;
		outptr += FRAME_SIZE*channels;
		count += samples;
		++frames;
	}

	out.resize(count*channels);
	//LOG(INFO) << "Received " << frames << " Opus frames";
	return true;

	#else
	LOG(WARNING) << "No Opus decoder installed";
	return false;
	#endif
}

bool SoftwareDecoder::_decodeRaw(const ftl::codecs::Packet &pkt, std::vector<short> &out) {
	size_t size = pkt.data.size()/sizeof(short);
	out.resize(size);
	auto *ptr = (short*)pkt.data.data();
	for (size_t i=0; i<size; i++) out.data()[i] = ptr[i];
	return true;
}

bool SoftwareDecoder::accepts(const ftl::codecs::Packet &) {
	return false;
}
