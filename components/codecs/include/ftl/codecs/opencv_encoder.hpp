#ifndef _FTL_CODECS_OPENCV_ENCODER_HPP_
#define _FTL_CODECS_OPENCV_ENCODER_HPP_

#include <ftl/codecs/encoder.hpp>
#include <ftl/threads.hpp>
#include <condition_variable>

namespace ftl {
namespace codecs {

/**
 * Use OpenCV imencode to encode frames as images. This codec divides each
 * image into blocks and uses a thread per block to compress each one. It is
 * a simplistic codec that can function at high resolution but is best used for
 * low definition encoding due to the poor compressed data size.
 */
class OpenCVEncoder : public ftl::codecs::Encoder {
    public:
    OpenCVEncoder(ftl::codecs::definition_t maxdef,
			ftl::codecs::definition_t mindef);
    ~OpenCVEncoder();

	bool encode(const cv::cuda::GpuMat &in, ftl::codecs::preset_t preset,
			const std::function<void(const ftl::codecs::Packet&)> &cb) {
		return Encoder::encode(in, preset, cb);
	}

    bool encode(const cv::cuda::GpuMat &in, ftl::codecs::definition_t definition, ftl::codecs::bitrate_t bitrate,
			const std::function<void(const ftl::codecs::Packet&)>&) override;

	bool supports(ftl::codecs::codec_t codec) override;

    //bool encode(const cv::cuda::GpuMat &in, std::vector<uint8_t> &out, bitrate_t bix, bool);

	private:
	int chunk_count_;
	int chunk_dim_;
	ftl::codecs::preset_t current_preset_;
	ftl::codecs::definition_t current_definition_;
	std::atomic<int> jobs_;
	std::mutex job_mtx_;
	std::condition_variable job_cv_;
	cv::Mat tmp_;

	bool _encodeBlock(const cv::Mat &in, ftl::codecs::Packet &pkt, ftl::codecs::bitrate_t);
};

}
}

#endif  // _FTL_CODECS_OPENCV_ENCODER_HPP_
