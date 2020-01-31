#include <ftl/codecs/nvpipe_decoder.hpp>
#include <ftl/codecs/nvpipe_encoder.hpp>

#include <loguru.hpp>

#include <ftl/cuda_util.hpp>
#include <ftl/codecs/hevc.hpp>
#include <ftl/codecs/h264.hpp>
//#include <cuda_runtime.h>

#include <opencv2/core/cuda/common.hpp>

#include <ftl/codecs/depth_convert_cuda.hpp>

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

template <typename T>
static T readValue(const unsigned char **data) {
	const T *ptr = (const T*)(*data);
	*data += sizeof(T);
	return *ptr;
}

bool NvPipeDecoder::_checkIFrame(ftl::codecs::codec_t codec, const unsigned char *data, size_t size) {
	if (!seen_iframe_) {
		if (codec == ftl::codecs::codec_t::HEVC || codec == ftl::codecs::codec_t::HEVC_LOSSLESS) {
			if (ftl::codecs::hevc::isIFrame(data, size)) seen_iframe_ = true;
		} else if (codec == ftl::codecs::codec_t::H264 || codec == ftl::codecs::codec_t::H264_LOSSLESS) {
			if (ftl::codecs::h264::isIFrame(data, size)) seen_iframe_ = true;
		}
	}
	return seen_iframe_;
}

bool NvPipeDecoder::decode(const ftl::codecs::Packet &pkt, cv::cuda::GpuMat &out) {
	//cudaSetDevice(0);
	UNIQUE_LOCK(mutex_,lk);
	if (pkt.codec != codec_t::HEVC && pkt.codec != codec_t::H264 && pkt.codec != codec_t::HEVC_LOSSLESS && pkt.codec != codec_t::H264_LOSSLESS) return false;

	bool is_float_frame = pkt.flags & ftl::codecs::kFlagFloat;
	bool islossless = ((pkt.codec == ftl::codecs::codec_t::HEVC || pkt.codec == ftl::codecs::codec_t::H264) && is_float_frame &&
		!(pkt.flags & 0x2)) || pkt.codec == ftl::codecs::codec_t::HEVC_LOSSLESS || pkt.codec == ftl::codecs::codec_t::H264_LOSSLESS; 

	if (is_float_frame && !islossless && out.type() != CV_16UC4) {
		LOG(ERROR) << "Invalid buffer for lossy float frame";
		return false;
	}

	if (is_float_frame && islossless && out.type() != CV_16U) {
		LOG(ERROR) << "Invalid buffer for lossless float frame";
		return false;
	}

	if (!is_float_frame && out.type() != CV_8UC4) {
		LOG(ERROR) << "Invalid buffer for lossy colour frame: " << out.type();
		return false;
	}

	int width = ftl::codecs::getWidth(pkt.definition);
	int height = ftl::codecs::getHeight(pkt.definition);
	auto [tx,ty] = ftl::codecs::chooseTileConfig(pkt.frame_count);

	if (tx*width != out.cols || ty*height != out.rows) {
		LOG(ERROR) << "Received frame too large for output";
		return false;
	}

	// Is the previous decoder still valid for current resolution and type?
	if (nv_decoder_ != nullptr && (last_definition_ != pkt.definition || last_codec_ != pkt.codec || is_float_channel_ != is_float_frame)) {
		NvPipe_Destroy(nv_decoder_);
		nv_decoder_ = nullptr;
	}

	is_float_channel_ = is_float_frame;
	last_definition_ = pkt.definition;
	last_codec_ = pkt.codec;

	// Build a decoder instance of the correct kind
	if (nv_decoder_ == nullptr) {
		nv_decoder_ = NvPipe_CreateDecoder(
				(is_float_frame) ? (islossless) ? NVPIPE_UINT16 : NVPIPE_YUV64 : NVPIPE_RGBA32,
				(pkt.codec == codec_t::HEVC || pkt.codec == ftl::codecs::codec_t::HEVC_LOSSLESS) ? NVPIPE_HEVC : NVPIPE_H264,
				out.cols,
				out.rows);
		if (!nv_decoder_) {
			//LOG(INFO) << "Bitrate=" << (int)bitrate << " width=" << ABRController::getColourWidth(bitrate);
			LOG(FATAL) << "Could not create decoder: " << NvPipe_GetError(NULL);
		}

		seen_iframe_ = false;
	}
	
	//tmp_.create(cv::Size(ftl::codecs::getWidth(pkt.definition),ftl::codecs::getHeight(pkt.definition)), (!is_float_frame) ? CV_8UC4 : (islossless) ? CV_16U : CV_16UC4);

	// Final checks for validity
	if (pkt.data.size() == 0) { // || !ftl::codecs::hevc::validNAL(pkt.data)) {
		LOG(ERROR) << "Failed to decode packet";
		return false;
	}

	int rc = 0;
	if (pkt.flags & ftl::codecs::kFlagMultiple) {
		const unsigned char *ptr = pkt.data.data();
		const unsigned char *eptr = ptr+pkt.data.size();

		while (ptr < eptr) {
			int size = readValue<int>(&ptr);

			// Skip if still missing an IFrame.
			if (!_checkIFrame(pkt.codec, ptr, size)) {
				LOG(WARNING) << "P-Frame without I-Frame in decoder";
				ptr += size;
				if (ptr < eptr) continue;
				else return false;
			}

			rc = NvPipe_Decode(nv_decoder_, ptr, size, out.data, out.cols, out.rows, out.step);
			if (rc == 0) LOG(ERROR) << "NvPipe decode error: " << NvPipe_GetError(nv_decoder_);
			ptr += size;
		}

		//LOG(WARNING) << "Decode of multiple frames: " << count;
	} else {
		if (!_checkIFrame(pkt.codec, pkt.data.data(), pkt.data.size())) {
			LOG(WARNING) << "P-Frame without I-Frame in decoder: " << pkt.data.size();
			return false;
		}
		rc = NvPipe_Decode(nv_decoder_, pkt.data.data(), pkt.data.size(), out.data, out.cols, out.rows, out.step);
		if (rc == 0) LOG(ERROR) << "NvPipe decode error: " << NvPipe_GetError(nv_decoder_);
	}

	/*if (is_float_frame) {
		if (!islossless) {
			//cv::cuda::cvtColor(tmp_, tmp_, cv::COLOR_RGB2YUV, 4, stream_);

			ftl::cuda::vuya_to_depth(out, tmp_, 16.0f, stream_);
		} else {
			tmp_.convertTo(out, CV_32FC1, 1.0f/1000.0f, stream_);
		}
	} else {
		// Flag 0x1 means frame is in RGB so needs conversion to BGR
		if (pkt.flags & 0x1) {
			cv::cuda::cvtColor(tmp_, out, cv::COLOR_RGBA2BGRA, 0, stream_);
		}
	}*/

	//stream_.waitForCompletion();

	return rc > 0;
}

bool NvPipeDecoder::accepts(const ftl::codecs::Packet &pkt) {
	return pkt.codec == codec_t::HEVC || pkt.codec == codec_t::H264 || pkt.codec == codec_t::H264_LOSSLESS || pkt.codec == codec_t::HEVC_LOSSLESS;
}
