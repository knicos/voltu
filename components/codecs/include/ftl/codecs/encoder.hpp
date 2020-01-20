#ifndef _FTL_CODECS_ENCODER_HPP_
#define _FTL_CODECS_ENCODER_HPP_

#include <ftl/cuda_util.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>

#include <ftl/codecs/codecs.hpp>
#include <ftl/codecs/packet.hpp>

namespace ftl {
namespace codecs {

static const unsigned int kVideoBufferSize = 10*1024*1024;

class Encoder;

enum class device_t {
	Any = 0,
	Hardware,
	Software,
	NVIDIA,
	x265,
	OpenCV
};

/**
 * Allocate a high quality encoder. This is a hardware encoding device if
 * available, or otherwise a CPU encoder. There are a restricted number of
 * high quality encoders due to system resources. A nullptr is returned if no
 * encoders are available, however, it will return a low quality encoder if no
 * high quality encoders are available.
 */
Encoder *allocateEncoder(
		ftl::codecs::definition_t maxdef=ftl::codecs::definition_t::HD1080,
		ftl::codecs::device_t dev=ftl::codecs::device_t::Any,
		ftl::codecs::codec_t codec=ftl::codecs::codec_t::Any);

/**
 * Release an encoder to be reused by some other stream.
 */
void free(Encoder *&e);

/**
 * Abstract encoder interface to be implemented. Anything implementing this will
 * convert an OpenCV Mat or GpuMat into a compressed byte array of some form.
 */
class Encoder {
	public:
	friend Encoder *allocateEncoder(ftl::codecs::definition_t,
			ftl::codecs::device_t, ftl::codecs::codec_t);
	friend void free(Encoder *&);

	public:
	Encoder(ftl::codecs::definition_t maxdef,
			ftl::codecs::definition_t mindef,
			ftl::codecs::device_t dev);
	virtual ~Encoder();

	/**
	 * Encode a frame given as an opencv gpumat. The packet structure should
	 * be filled before calling this function as the codec, definition and
	 * bitrate given in the packet are used as suggestions for the encoder. The
	 * encoder will modify the fields within the packet and will populate the
	 * data element.
	 * 
	 * @param in An OpenCV image of the frame to compress
	 * @param fmt Colour format used.
	 * @param pkt Partially filled packet to be encoded into.
	 * @return True if succeeded with encoding.
	 */
	virtual bool encode(const cv::cuda::GpuMat &in, ftl::codecs::Packet &pkt)=0;

	virtual void reset() {}

	virtual bool supports(ftl::codecs::codec_t codec)=0;

	cv::cuda::Stream &stream() { return stream_; }

	protected:
	bool available;
	const ftl::codecs::definition_t max_definition;
	const ftl::codecs::definition_t min_definition;
	const ftl::codecs::device_t device;
	cv::cuda::Stream stream_;
};

}
}

#endif  // _FTL_CODECS_ENCODER_HPP_
