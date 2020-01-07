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

	bool encode(const cv::cuda::GpuMat &in, ftl::codecs::preset_t preset,
			const std::function<void(const ftl::codecs::Packet&)> &cb) {
		return Encoder::encode(in, preset, cb);
	}

	bool encode(const cv::cuda::GpuMat &in, ftl::codecs::definition_t definition, ftl::codecs::bitrate_t bitrate,
			const std::function<void(const ftl::codecs::Packet&)>&) override;

	//bool encode(const cv::cuda::GpuMat &in, std::vector<uint8_t> &out, bitrate_t bix, bool);

	void reset();

	bool supports(ftl::codecs::codec_t codec) override;

	static constexpr int kFlagRGB = 0x00000001;
	static constexpr int kFlagMappedDepth = 0x00000002;

	private:
	NvPipe *nvenc_;
	definition_t current_definition_;
	bool is_float_channel_;
	bool was_reset_;
	ftl::codecs::codec_t preference_;
	ftl::codecs::codec_t current_codec_;
	cv::cuda::GpuMat tmp_;
	cv::cuda::GpuMat tmp2_;
	cv::cuda::Stream stream_;

	bool _encoderMatch(const cv::cuda::GpuMat &in, definition_t def);
	bool _createEncoder(const cv::cuda::GpuMat &in, definition_t def, bitrate_t rate);
	ftl::codecs::definition_t _verifiedDefinition(ftl::codecs::definition_t def, const cv::cuda::GpuMat &in);
};

}
}

#endif  // _FTL_CODECS_NVPIPE_ENCODER_HPP_
