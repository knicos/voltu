#ifndef _FTL_CODECS_OPENCV_DECODER_HPP_
#define _FTL_CODECS_OPENCV_DECODER_HPP_

#include <ftl/codecs/decoder.hpp>

namespace ftl {
namespace codecs {

class OpenCVDecoder : public ftl::codecs::Decoder {
	public:
	OpenCVDecoder();
	~OpenCVDecoder();

	bool decode(const ftl::codecs::Packet &pkt, cv::cuda::GpuMat &out) override;

	bool accepts(const ftl::codecs::Packet &pkt);

	private:
	cv::Mat tmp_;
};

}
}

#endif  // _FTL_CODECS_OPENCV_DECODER_HPP_
