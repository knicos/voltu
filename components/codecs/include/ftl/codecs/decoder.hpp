#ifndef _FTL_CODECS_DECODER_HPP_
#define _FTL_CODECS_DECODER_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>

#include <ftl/codecs/packet.hpp>


namespace ftl {
namespace codecs {

class Decoder;

/**
 * Allocate a decoder that can handle the received packet.
 */
Decoder *allocateDecoder(const ftl::codecs::Packet &);

/**
 * Release a decoder to be reused by some other stream.
 */
void free(Decoder *&e);

/**
 * An abstract interface for a frame decoder. An implementation of this class
 * will take codec packets and reconstruct an image or partial image into a
 * specified OpenCV Mat. It is not the job of the decoder to check when a frame
 * is completed, the user of the decoder should check the packet block number
 * and total to ensure all packets are received for a given frame. The decoder
 * is threadsafe.
 */
class Decoder {
	public:
	Decoder() {};
	virtual ~Decoder() {};

	virtual bool decode(const ftl::codecs::Packet &pkt, cv::cuda::GpuMat &out)=0;

	virtual bool accepts(const ftl::codecs::Packet &)=0;
};

}
}

#endif  // _FTL_CODECS_DECODER_HPP_
