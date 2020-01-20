#ifndef _FTL_CODECS_NVPIPE_DECODER_HPP_
#define _FTL_CODECS_NVPIPE_DECODER_HPP_

#include <ftl/codecs/decoder.hpp>
#include <ftl/threads.hpp>

#include <NvPipe.h>

namespace ftl {
namespace codecs {

class NvPipeDecoder : public ftl::codecs::Decoder {
	public:
	NvPipeDecoder();
	~NvPipeDecoder();

	bool decode(const ftl::codecs::Packet &pkt, cv::cuda::GpuMat &out) override;

	bool accepts(const ftl::codecs::Packet &pkt);

	private:
	NvPipe *nv_decoder_;
	bool is_float_channel_;
	ftl::codecs::definition_t last_definition_;
	ftl::codecs::codec_t last_codec_;
	MUTEX mutex_;
	bool seen_iframe_;
	cv::cuda::GpuMat tmp_;

	bool _checkIFrame(ftl::codecs::codec_t codec, const unsigned char *data, size_t size);
};

}
}

#endif  // _FTL_CODECS_NVPIPE_DECODER_HPP_
