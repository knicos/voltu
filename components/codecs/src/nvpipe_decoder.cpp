#include <ftl/codecs/nvpipe_decoder.hpp>

#include <loguru.hpp>

#include <ftl/cuda_util.hpp>
#include <ftl/codecs/hevc.hpp>
#include <ftl/codecs/h264.hpp>
//#include <cuda_runtime.h>

#include <opencv2/core/cuda/common.hpp>

using ftl::codecs::NvPipeDecoder;

NvPipeDecoder::NvPipeDecoder() {
	nv_decoder_ = nullptr;
	seen_iframe_ = false;
}

NvPipeDecoder::~NvPipeDecoder() {
	if (nv_decoder_ != nullptr) {
		NvPipe_Destroy(nv_decoder_);
	}
}

bool NvPipeDecoder::decode(const ftl::codecs::Packet &pkt, cv::cuda::GpuMat &out) {
	cudaSetDevice(0);
	UNIQUE_LOCK(mutex_,lk);
	if (pkt.codec != codec_t::HEVC && pkt.codec != codec_t::H264) return false;
	bool is_float_frame = out.type() == CV_32F;

	// Is the previous decoder still valid for current resolution and type?
	if (nv_decoder_ != nullptr && (last_definition_ != pkt.definition || is_float_channel_ != is_float_frame)) {
		NvPipe_Destroy(nv_decoder_);
		nv_decoder_ = nullptr;
	}

	is_float_channel_ = is_float_frame;
	last_definition_ = pkt.definition;

	//LOG(INFO) << "DECODE RESOLUTION: (" << (int)pkt.definition << ") " << ftl::codecs::getWidth(pkt.definition) << "x" << ftl::codecs::getHeight(pkt.definition);

	// Build a decoder instance of the correct kind
	if (nv_decoder_ == nullptr) {
		nv_decoder_ = NvPipe_CreateDecoder(
				(is_float_frame) ? NVPIPE_UINT16 : NVPIPE_RGBA32,
				(pkt.codec == codec_t::HEVC) ? NVPIPE_HEVC : NVPIPE_H264,
				ftl::codecs::getWidth(pkt.definition),
				ftl::codecs::getHeight(pkt.definition));
		if (!nv_decoder_) {
			//LOG(INFO) << "Bitrate=" << (int)bitrate << " width=" << ABRController::getColourWidth(bitrate);
			LOG(FATAL) << "Could not create decoder: " << NvPipe_GetError(NULL);
		} else {
			DLOG(INFO) << "Decoder created";
		}

		seen_iframe_ = false;
	}
	
	// TODO: (Nick) Move to member variable to prevent re-creation
	tmp_.create(cv::Size(ftl::codecs::getWidth(pkt.definition),ftl::codecs::getHeight(pkt.definition)), (is_float_frame) ? CV_16U : CV_8UC4);

	// Check for an I-Frame
	if (pkt.codec == ftl::codecs::codec_t::HEVC) {
		if (ftl::codecs::hevc::isIFrame(pkt.data)) seen_iframe_ = true;
	} else if (pkt.codec == ftl::codecs::codec_t::H264) {
		if (ftl::codecs::h264::isIFrame(pkt.data)) seen_iframe_ = true;
	}

	// No I-Frame yet so don't attempt to decode P-Frames.
	if (!seen_iframe_) return false;

	int rc = NvPipe_Decode(nv_decoder_, pkt.data.data(), pkt.data.size(), tmp_.data, tmp_.cols, tmp_.rows, tmp_.step);
	if (rc == 0) LOG(ERROR) << "NvPipe decode error: " << NvPipe_GetError(nv_decoder_);

	if (is_float_frame) {
		// Is the received frame the same size as requested output?
		if (out.rows == ftl::codecs::getHeight(pkt.definition)) {
			tmp_.convertTo(out, CV_32FC1, 1.0f/1000.0f, stream_);
		} else {
			LOG(WARNING) << "Resizing decoded frame from " << tmp_.size() << " to " << out.size();
			// FIXME: This won't work on GPU
			tmp_.convertTo(tmp_, CV_32FC1, 1.0f/1000.0f, stream_);
			cv::cuda::resize(tmp_, out, out.size(), 0, 0, cv::INTER_NEAREST, stream_);
		}
	} else {
		// Is the received frame the same size as requested output?
		if (out.rows == ftl::codecs::getHeight(pkt.definition)) {
			// Flag 0x1 means frame is in RGB so needs conversion to BGR
			if (pkt.flags & 0x1) {
				cv::cuda::cvtColor(tmp_, out, cv::COLOR_RGBA2BGR, 0, stream_);
			} else {
				cv::cuda::cvtColor(tmp_, out, cv::COLOR_BGRA2BGR, 0, stream_);
			}
		} else {
			LOG(WARNING) << "Resizing decoded frame from " << tmp_.size() << " to " << out.size();
			// FIXME: This won't work on GPU, plus it allocates extra memory...
			// Flag 0x1 means frame is in RGB so needs conversion to BGR
			if (pkt.flags & 0x1) {
				cv::cuda::cvtColor(tmp_, tmp_, cv::COLOR_RGBA2BGR, 0, stream_);
			} else {
				cv::cuda::cvtColor(tmp_, tmp_, cv::COLOR_BGRA2BGR, 0, stream_);
			}
			cv::cuda::resize(tmp_, out, out.size(), 0.0, 0.0, cv::INTER_LINEAR, stream_);
		}
	}

	stream_.waitForCompletion();

	return rc > 0;
}

bool NvPipeDecoder::accepts(const ftl::codecs::Packet &pkt) {
	return pkt.codec == codec_t::HEVC || pkt.codec == codec_t::H264;
}
