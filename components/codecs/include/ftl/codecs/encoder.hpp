#ifndef _FTL_CODECS_ENCODER_HPP_
#define _FTL_CODECS_ENCODER_HPP_

#include <ftl/cuda_util.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>

#include <ftl/codecs/bitrates.hpp>
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
		ftl::codecs::device_t dev=ftl::codecs::device_t::Any);

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
			ftl::codecs::device_t);
    friend void free(Encoder *&);

    public:
    Encoder(ftl::codecs::definition_t maxdef,
			ftl::codecs::definition_t mindef,
			ftl::codecs::device_t dev);
    virtual ~Encoder();

	/**
	 * Wrapper encode to allow use of presets.
	 */
	virtual bool encode(const cv::Mat &in, ftl::codecs::preset_t preset,
			const std::function<void(const ftl::codecs::Packet&)> &cb);

	/**
	 * Encode a frame at specified preset and call a callback function for each
	 * block as it is encoded. The callback may be called only once with the
	 * entire frame or it may be called many times from many threads with
	 * partial blocks of a frame. Each packet should be sent over the network
	 * to all listening clients.
	 * 
	 * @param in An OpenCV image of the frame to compress
	 * @param preset Codec preset for resolution and quality
	 * @param iframe Full frame if true, else might be a delta frame
	 * @param cb Callback containing compressed data
	 * @return True if succeeded with encoding.
	 */
    virtual bool encode(
			const cv::Mat &in,
			ftl::codecs::definition_t definition,
			ftl::codecs::bitrate_t bitrate,
			const std::function<void(const ftl::codecs::Packet&)> &cb)=0;

	// TODO: Eventually, use GPU memory directly since some encoders can support this
    //virtual bool encode(const cv::cuda::GpuMat &in, std::vector<uint8_t> &out, bitrate_t bix, bool)=0;

	virtual void reset() {}

    protected:
    bool available;
	const ftl::codecs::definition_t max_definition;
	const ftl::codecs::definition_t min_definition;
	const ftl::codecs::device_t device;
};

}
}

#endif  // _FTL_CODECS_ENCODER_HPP_
