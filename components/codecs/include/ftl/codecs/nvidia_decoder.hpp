/**
 * @file nvidia_decoder.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_CODECS_NVIDIA_DECODER_HPP_
#define _FTL_CODECS_NVIDIA_DECODER_HPP_

#include <ftl/codecs/decoder.hpp>
#include <ftl/threads.hpp>

class NvDecoder;  // From Nvidia Video SDK

namespace ftl {
namespace codecs {

/**
 * Use NVIDIA hardware decoder. Utilises Nvidia Video SDK 9.1.23 and supports
 * H.264 and HEVC. In principle this supports other codecs but we don't use them.
 */
class NvidiaDecoder : public ftl::codecs::Decoder {
	public:
	NvidiaDecoder();
	~NvidiaDecoder();

	bool decode(const ftl::codecs::Packet &pkt, cv::cuda::GpuMat &out) override;

	bool accepts(const ftl::codecs::Packet &pkt);

	private:
	NvDecoder *nv_decoder_;
	bool is_float_channel_;
	ftl::codecs::codec_t last_codec_;
	MUTEX mutex_;
	bool seen_iframe_;
	cv::cuda::GpuMat buffer_;
	int width_;
	int height_;
	int last_width_;
	int last_height_;
	int n_;

	bool _create(const ftl::codecs::Packet &pkt);
	uint8_t* _decode(const uint8_t* src, uint64_t srcSize);
	bool _checkIFrame(ftl::codecs::codec_t codec, const unsigned char *data, size_t size);
};

}  // namespace codecs
}  // namespace ftl

#endif  // _FTL_CODECS_NVIDIA_DECODER_HPP_
