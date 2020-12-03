/**
 * @file opencv_decoder.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_CODECS_OPENCV_DECODER_HPP_
#define _FTL_CODECS_OPENCV_DECODER_HPP_

#include <ftl/codecs/decoder.hpp>

namespace ftl {
namespace codecs {

/**
 * Use OpenCV to do image decoding. This does not support video but uses images
 * instead, such as jpg or png, and is entirely CPU software based.
 */
class OpenCVDecoder : public ftl::codecs::Decoder {
	public:
	OpenCVDecoder();
	~OpenCVDecoder();

	bool decode(const ftl::codecs::Packet &pkt, cv::cuda::GpuMat &out) override;

	bool accepts(const ftl::codecs::Packet &pkt);

	private:
	//cv::Mat tmp_;  // TODO: Use this!?
};

}  // namespace codecs
}  // namespace ftl

#endif  // _FTL_CODECS_OPENCV_DECODER_HPP_
