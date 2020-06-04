#ifndef _FTL_CODECS_NVPIPE_DECODER_HPP_
#define _FTL_CODECS_NVPIPE_DECODER_HPP_

#include <ftl/codecs/decoder.hpp>
#include <ftl/threads.hpp>

//#include <NvPipe.h>

class NvDecoder;

namespace ftl {
namespace codecs {

class NvPipeDecoder : public ftl::codecs::Decoder {
	public:
	NvPipeDecoder();
	~NvPipeDecoder();

	bool decode(const ftl::codecs::Packet &pkt, cv::cuda::GpuMat &out) override;

	bool accepts(const ftl::codecs::Packet &pkt);

	private:
	NvDecoder *nv_decoder_;
	bool is_float_channel_;
	ftl::codecs::definition_t last_definition_;
	ftl::codecs::codec_t last_codec_;
	MUTEX mutex_;
	bool seen_iframe_;
	cv::cuda::GpuMat tmp_;
	int width_;
	int height_;
	int n_;

	bool _create(const ftl::codecs::Packet &pkt);
	uint8_t* _decode(const uint8_t* src, uint64_t srcSize);
	bool _checkIFrame(ftl::codecs::codec_t codec, const unsigned char *data, size_t size);
};

}
}

#endif  // _FTL_CODECS_NVPIPE_DECODER_HPP_
