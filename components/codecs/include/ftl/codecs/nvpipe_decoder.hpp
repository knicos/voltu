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

	bool decode(const ftl::codecs::Packet &pkt, cv::Mat &out);

	bool accepts(const ftl::codecs::Packet &pkt);

	private:
	NvPipe *nv_decoder_;
	bool is_float_channel_;
	ftl::codecs::definition_t last_definition_;
	MUTEX mutex_;
};

}
}

#endif  // _FTL_CODECS_NVPIPE_DECODER_HPP_
