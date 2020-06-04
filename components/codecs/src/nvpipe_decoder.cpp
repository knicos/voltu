#include <ftl/codecs/nvpipe_decoder.hpp>
#include <ftl/codecs/nvpipe_encoder.hpp>
#include <ftl/exception.hpp>

#include <loguru.hpp>

#include <ftl/cuda_util.hpp>
#include <ftl/codecs/hevc.hpp>
#include <ftl/codecs/h264.hpp>
//#include <cuda_runtime.h>

#include <opencv2/core/cuda/common.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <ftl/codecs/depth_convert_cuda.hpp>

#include <opencv2/highgui.hpp>

#include "Video_Codec_SDK_9.1.23/Samples/NvCodec/NvDecoder/NvDecoder.h"
#include "../Utils/ColorSpace.h"

using ftl::codecs::NvPipeDecoder;
using ftl::codecs::codec_t;


NvPipeDecoder::NvPipeDecoder() {
	nv_decoder_ = nullptr;
	seen_iframe_ = false;
}

NvPipeDecoder::~NvPipeDecoder() {
	if (nv_decoder_ != nullptr) {
		delete nv_decoder_;
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

static inline std::string DecErrorCodeToString(CUresult code)
{
    const char* str = nullptr;
    cuGetErrorName(code, &str);

    if (str)
        return std::string(str);

    return "Unknown error code";
}

bool NvPipeDecoder::_create(const ftl::codecs::Packet &pkt) {
	bool is_float_frame = pkt.flags & ftl::codecs::kFlagFloat;

	// Check existing decoder is valid first and remove if not
	if (nv_decoder_ != nullptr && (last_definition_ != pkt.definition || last_codec_ != pkt.codec || is_float_channel_ != is_float_frame)) {
		delete nv_decoder_;
		nv_decoder_ = nullptr;
	}

	if (!nv_decoder_) {
		// Ensure we have a CUDA context
		LOG(INFO) << "Getting cuda context...";
        cudaSafeCall(cudaDeviceSynchronize());
		LOG(INFO) << "Have cuda context.";
        CUcontext cudaContext;
        cuCtxGetCurrent(&cudaContext);

		try {
			nv_decoder_ = new NvDecoder(cudaContext, true, (pkt.codec == codec_t::HEVC || pkt.codec == codec_t::HEVC_LOSSLESS) ? cudaVideoCodec_HEVC : cudaVideoCodec_H264, nullptr, true);
		} catch (NVDECException& e) {
			throw FTL_Error("Failed to create decoder (" << e.getErrorString() << ", error " << std::to_string(e.getErrorCode()) << " = " + DecErrorCodeToString(e.getErrorCode()) << ")");
		}

		seen_iframe_ = false;
	}

	return true;
}

uint8_t* NvPipeDecoder::_decode(const uint8_t* src, uint64_t srcSize) {
	int numFramesDecoded = 0;
	uint8_t **decodedFrames;
	int64_t *timeStamps;

	// From NvPipe
	try {
		// Some cuvid implementations have one frame latency. Refeed frame into pipeline in this case.
		const uint32_t DECODE_TRIES = 3;
		for (uint32_t i = 0; (i < DECODE_TRIES) && (numFramesDecoded <= 0); ++i)
			nv_decoder_->Decode(src, srcSize, &decodedFrames, &numFramesDecoded, CUVID_PKT_ENDOFPICTURE, &timeStamps, n_++, stream_);
	} catch (NVDECException& e) {
		throw FTL_Error("Decode failed (" << e.getErrorString() << ", error " << std::to_string(e.getErrorCode()) << " = " + DecErrorCodeToString(e.getErrorCode()) << ")");
	}

	if (numFramesDecoded <= 0) {
		throw FTL_Error("No frame decoded (Decoder expects encoded bitstream for a single complete frame. Accumulating partial data or combining multiple frames is not supported.)");
	}

	return decodedFrames[numFramesDecoded - 1];
}

bool NvPipeDecoder::decode(const ftl::codecs::Packet &pkt, cv::cuda::GpuMat &out) {
	//cudaSetDevice(0);
	UNIQUE_LOCK(mutex_,lk);
	if (pkt.codec != codec_t::HEVC && pkt.codec != codec_t::H264 && pkt.codec != codec_t::HEVC_LOSSLESS && pkt.codec != codec_t::H264_LOSSLESS) return false;

	bool is_float_frame = pkt.flags & ftl::codecs::kFlagFloat;
	bool islossless = ((pkt.codec == ftl::codecs::codec_t::HEVC || pkt.codec == ftl::codecs::codec_t::H264) && is_float_frame &&
		!(pkt.flags & 0x2)) || pkt.codec == ftl::codecs::codec_t::HEVC_LOSSLESS || pkt.codec == ftl::codecs::codec_t::H264_LOSSLESS; 

	/*if (is_float_frame && !islossless && out.type() != CV_16UC4) {
		LOG(ERROR) << "Invalid buffer for lossy float frame";
		return false;
	}*/

	if (is_float_frame && out.type() != CV_32F) {
		LOG(ERROR) << "Invalid buffer for float frame";
		return false;
	}

	if (!is_float_frame && out.type() != CV_8UC4) {
		LOG(ERROR) << "Invalid buffer for lossy colour frame: " << out.type();
		return false;
	}

	int width = ftl::codecs::getWidth(pkt.definition);
	int height = ftl::codecs::getHeight(pkt.definition);
	auto [tx,ty] = ftl::codecs::chooseTileConfig(pkt.frame_count);

	//int exp_height = (is_float_frame && !islossless) ? ty*height+((ty*height)/2) : ty*height;

	if (tx*width != out.cols || ty*height != out.rows) {
		LOG(ERROR) << "Received frame whose size does not match buffer";
		return false;
	}

	// Is the previous decoder still valid for current resolution and type?
	//if (nv_decoder_ != nullptr && (last_definition_ != pkt.definition || last_codec_ != pkt.codec || is_float_channel_ != is_float_frame)) {
	//	NvPipe_Destroy(nv_decoder_);
	//	nv_decoder_ = nullptr;
	//}

	// Create an appropriate NvDecoder instance
	width_ = tx*width;
	height_ = ty*height;
	//if (islossless && is_float_frame) width_ *= 2;  // 16bit = double width 8 bit
	_create(pkt);

	is_float_channel_ = is_float_frame;
	last_definition_ = pkt.definition;
	last_codec_ = pkt.codec;

	// Build a decoder instance of the correct kind
	/*if (nv_decoder_ == nullptr) {
		nv_decoder_ = NvPipe_CreateDecoder(
				(is_float_frame) ? (islossless) ? NVPIPE_UINT16 : NVPIPE_NV12_10bit : NVPIPE_RGBA32,
				(pkt.codec == codec_t::HEVC || pkt.codec == ftl::codecs::codec_t::HEVC_LOSSLESS) ? NVPIPE_HEVC : NVPIPE_H264,
				out.cols,
				out.rows);
		if (!nv_decoder_) {
			//LOG(INFO) << "Bitrate=" << (int)bitrate << " width=" << ABRController::getColourWidth(bitrate);
			LOG(FATAL) << "Could not create decoder: " << NvPipe_GetError(NULL);
		}

		seen_iframe_ = false;
	}*/
	
	//tmp_.create(cv::Size(ftl::codecs::getWidth(pkt.definition),ftl::codecs::getHeight(pkt.definition)), (!is_float_frame) ? CV_8UC4 : (islossless) ? CV_16U : CV_16UC4);

	// Final checks for validity
	if (pkt.data.size() == 0) { // || !ftl::codecs::hevc::validNAL(pkt.data)) {
		LOG(ERROR) << "Failed to decode packet";
		return false;
	}

	int rc = 0;
	uint8_t *decodedPtr = nullptr;

	if (pkt.flags & ftl::codecs::kFlagMultiple) {
		const unsigned char *ptr = pkt.data.data();
		const unsigned char *eptr = ptr+pkt.data.size();

		//LOG(WARNING) << "Decode of multiple frames";

		while (ptr < eptr) {
			int size = readValue<int>(&ptr);

			// Skip if still missing an IFrame.
			if (!_checkIFrame(pkt.codec, ptr, size)) {
				LOG(WARNING) << "P-Frame without I-Frame in decoder";
				ptr += size;
				if (ptr < eptr) continue;
				else return false;
			}

			decodedPtr = _decode(ptr, size);
			//rc = NvPipe_Decode(nv_decoder_, ptr, size, out.data, tx*width, ty*height, out.step);
			//if (rc == 0) LOG(ERROR) << "NvPipe decode error: " << NvPipe_GetError(nv_decoder_);
			ptr += size;
		}

		//LOG(WARNING) << "Decode of multiple frames: " << count;
	} else {
		if (!_checkIFrame(pkt.codec, pkt.data.data(), pkt.data.size())) {
			LOG(WARNING) << "P-Frame without I-Frame in decoder: " << pkt.data.size();
			return false;
		}
		decodedPtr = _decode(pkt.data.data(), pkt.data.size());
		//LOG(INFO) << "Decoded size = " << nv_decoder_->GetWidth() << "x" << nv_decoder_->GetHeight() << " - " << nv_decoder_->GetBPP();
		//rc = NvPipe_Decode(nv_decoder_, pkt.data.data(), pkt.data.size(), out.data, tx*width, ty*height, out.step);
		//if (rc == 0) LOG(ERROR) << "NvPipe decode error: " << NvPipe_GetError(nv_decoder_);
	}

	// OpenCV GpuMat for YCbCr 4:2:0
	cv::cuda::GpuMat surface;
	if (is_float_frame && !islossless) surface = cv::cuda::GpuMat(height_+height_/2, width_, CV_16U, decodedPtr, width_*2);
	else if (is_float_frame && islossless) surface = cv::cuda::GpuMat(height_+height_/2, width_*2, CV_8U, decodedPtr, width_*2);
	else surface = cv::cuda::GpuMat(height_+height_/2, width_, CV_8U, decodedPtr, width_);

	auto cvstream = cv::cuda::StreamAccessor::wrapStream(stream_);

	if (is_float_frame) {
		if (!islossless) {
			cv::cuda::GpuMat sroi = surface(cv::Rect(0,0,width_, height_));
			cv::cuda::GpuMat csroi = surface(cv::Rect(0,height_,width_, height_/2));
			//cv::cuda::cvtColor(tmp_, tmp_, cv::COLOR_RGB2YUV, 4, stream_);
			//LOG(INFO) << "Depth convert: " << out.cols << ", " << out.rows << ", " << out.type();
			//ftl::cuda::vuya_to_depth(out, tmp_, 16.0f, stream_);
			ftl::cuda::vuya_to_depth(out, sroi, csroi, 16.0f, cvstream);
		} else {
			//tmp_.convertTo(out, CV_32FC1, 1.0f/1000.0f, stream_);
			ftl::cuda::nv12_to_float(decodedPtr, width_*2, (float*)out.data, out.step1(), width_, height_, stream_);
		}
	} else {
		// Flag 0x1 means frame is in RGB so needs conversion to BGR
		if (pkt.flags & 0x1) {
			//cv::cuda::cvtColor(surface, out, cv::COLOR_YUV2BGRA_NV12, 0, cvstream);
			Nv12ToColor32<BGRA32>(decodedPtr, width_, out.data, out.step1(), width_, height_, 0, stream_);
		} else {
			//cv::cuda::cvtColor(surface, out, cv::COLOR_YUV2RGBA_NV12, 0, cvstream);
			Nv12ToColor32<RGBA32>(decodedPtr, width_, out.data, out.step1(), width_, height_, 0, stream_);
		}
	}

	//stream_.waitForCompletion();

	return true;
}

bool NvPipeDecoder::accepts(const ftl::codecs::Packet &pkt) {
	return pkt.codec == codec_t::HEVC || pkt.codec == codec_t::H264 || pkt.codec == codec_t::H264_LOSSLESS || pkt.codec == codec_t::HEVC_LOSSLESS;
}
