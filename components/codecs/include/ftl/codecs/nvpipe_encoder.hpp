#ifndef _FTL_CODECS_NVPIPE_ENCODER_HPP_
#define _FTL_CODECS_NVPIPE_ENCODER_HPP_

#include <ftl/codecs/encoder.hpp>
#include <NvPipe.h>

namespace ftl {
namespace codecs {

class NvPipeEncoder : public ftl::codecs::Encoder {
	public:
	NvPipeEncoder(ftl::codecs::definition_t maxdef,
			ftl::codecs::definition_t mindef);
	~NvPipeEncoder();

	bool encode(const cv::cuda::GpuMat &in, ftl::codecs::Packet &pkt) override;

	//bool encode(const cv::cuda::GpuMat &in, std::vector<uint8_t> &out, bitrate_t bix, bool);

	void reset();

	bool supports(ftl::codecs::codec_t codec) override;

	static constexpr int kFlagRGB = 0x00000001;
	static constexpr int kFlagMappedDepth = 0x00000002;

	private:
	NvPipe *nvenc_;
	NvPipe_Codec codec_;
	NvPipe_Format format_;
	NvPipe_Compression compression_;
	uint8_t last_bitrate_;

	bool was_reset_;
	cv::cuda::GpuMat tmp_;
	cv::cuda::GpuMat tmp2_;

	bool _encoderMatch(const ftl::codecs::Packet &pkt, format_t fmt);
	bool _createEncoder(const ftl::codecs::Packet &pkt, format_t fmt);
	ftl::codecs::definition_t _verifiedDefinition(ftl::codecs::definition_t def, const cv::cuda::GpuMat &in);
};

}
}

#endif  // _FTL_CODECS_NVPIPE_ENCODER_HPP_
