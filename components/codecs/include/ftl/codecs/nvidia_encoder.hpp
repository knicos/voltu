/**
 * @file nvidia_encoder.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_CODECS_NVIDIA_ENCODER_HPP_
#define _FTL_CODECS_NVIDIA_ENCODER_HPP_

#include <ftl/codecs/encoder.hpp>

class NvEncoderCuda;  // From Nvidia Video SDK

namespace ftl {
namespace codecs {

/**
 * Uses Nvidia hardware encoder. Utilises Nvidia Video SDK 9.1.23 and supports
 * H.264 and HEVC.
 */
class NvidiaEncoder : public ftl::codecs::Encoder {
	public:
	NvidiaEncoder();
	~NvidiaEncoder();

	bool encode(const cv::cuda::GpuMat &in, ftl::codecs::Packet &pkt) override;

	void reset();

	bool supports(ftl::codecs::codec_t codec) override;

	struct Parameters {
		ftl::codecs::codec_t codec;
		bool is_float;
		uint32_t width;
		uint32_t height;
		uint8_t bitrate;

		inline uint32_t encodeWidth() const { return (is_float && !isLossy()) ? width*2 : width; }
		inline uint32_t encodeHeight() const { return height; }
		inline bool isLossy() const { return codec == ftl::codecs::codec_t::HEVC || codec == ftl::codecs::codec_t::H264; }

		inline bool operator==(const Parameters &p) const {
			return codec == p.codec && is_float == p.is_float && width == p.width &&
				height == p.height && bitrate == p.bitrate;
		}
	};

	private:
	NvEncoderCuda *nvenc_;
	ftl::codecs::codec_t codec_;
	Parameters params_;

	bool was_reset_;
	int64_t frame_count_ = 0;

	bool _createEncoder(const cv::cuda::GpuMat &in, const ftl::codecs::Packet &pkt);
	uint64_t _encode(uint8_t* dst, uint64_t dstSize, bool forceIFrame);
};

}
}

#endif  // _FTL_CODECS_NVIDIA_ENCODER_HPP_
