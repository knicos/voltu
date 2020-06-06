#include <ftl/codecs/nvidia_encoder.hpp>
#include <loguru.hpp>
#include <ftl/timer.hpp>
#include <ftl/codecs/codecs.hpp>
#include <ftl/cuda_util.hpp>
#include <ftl/exception.hpp>

#include <opencv2/core/cuda/common.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <opencv2/cudaimgproc.hpp>

#include <ftl/codecs/depth_convert_cuda.hpp>

#include "NvEncoder/NvEncoderCuda.h"

using ftl::codecs::NvidiaEncoder;
using ftl::codecs::bitrate_t;
using ftl::codecs::codec_t;
using ftl::codecs::definition_t;
using ftl::codecs::format_t;
using ftl::codecs::Packet;
using ftl::codecs::kFlagFloat;
using ftl::codecs::kFlagFlipRGB;
using ftl::codecs::kFlagMappedDepth;

static inline std::string EncErrorCodeToString(NVENCSTATUS code)
{
    std::vector<std::string> errors = {
        "NV_ENC_SUCCESS",
        "NV_ENC_ERR_NO_ENCODE_DEVICE",
        "NV_ENC_ERR_UNSUPPORTED_DEVICE",
        "NV_ENC_ERR_INVALID_ENCODERDEVICE",
        "NV_ENC_ERR_INVALID_DEVICE",
        "NV_ENC_ERR_DEVICE_NOT_EXIST",
        "NV_ENC_ERR_INVALID_PTR",
        "NV_ENC_ERR_INVALID_EVENT",
        "NV_ENC_ERR_INVALID_PARAM",
        "NV_ENC_ERR_INVALID_CALL",
        "NV_ENC_ERR_OUT_OF_MEMORY",
        "NV_ENC_ERR_ENCODER_NOT_INITIALIZED",
        "NV_ENC_ERR_UNSUPPORTED_PARAM",
        "NV_ENC_ERR_LOCK_BUSY",
        "NV_ENC_ERR_NOT_ENOUGH_BUFFER",
        "NV_ENC_ERR_INVALID_VERSION",
        "NV_ENC_ERR_MAP_FAILED",
        "NV_ENC_ERR_NEED_MORE_INPUT",
        "NV_ENC_ERR_ENCODER_BUSY",
        "NV_ENC_ERR_EVENT_NOT_REGISTERD",
        "NV_ENC_ERR_GENERIC",
        "NV_ENC_ERR_INCOMPATIBLE_CLIENT_KEY",
        "NV_ENC_ERR_UNIMPLEMENTED",
        "NV_ENC_ERR_RESOURCE_REGISTER_FAILED",
        "NV_ENC_ERR_RESOURCE_NOT_REGISTERED",
        "NV_ENC_ERR_RESOURCE_NOT_MAPPED"
    };

    if (code >= 0 && code < errors.size())
        return errors[code];

    return "Unknown error code";
}

NvidiaEncoder::NvidiaEncoder(definition_t maxdef,
			definition_t mindef) : Encoder(maxdef, mindef, ftl::codecs::device_t::Hardware) {
	nvenc_ = nullptr;
	was_reset_ = false;
}

NvidiaEncoder::~NvidiaEncoder() {
	if (nvenc_) {
		std::vector<std::vector<uint8_t>> tmp;
		nvenc_->EndEncode(tmp);
		nvenc_->DestroyEncoder();
		delete nvenc_;
		nvenc_ = nullptr;
	}
}

void NvidiaEncoder::reset() {
	was_reset_ = true;
}

bool NvidiaEncoder::supports(ftl::codecs::codec_t codec) {
	switch (codec) {
	case codec_t::H264_LOSSLESS:
	case codec_t::HEVC_LOSSLESS:
	case codec_t::H264:
	case codec_t::HEVC: return true;
	default: return false;
	}
}

/*
 * Create encoder params structure from packet and surface. Changes to these
 * require a recreation of the encoder.
 */
static ftl::codecs::NvidiaEncoder::Parameters generateParams(const cv::cuda::GpuMat &in, const ftl::codecs::Packet &pkt) {
	ftl::codecs::NvidiaEncoder::Parameters params;
	params.bitrate = pkt.bitrate;
	params.codec = pkt.codec;
	params.width = in.cols;
	params.height = in.rows;
	params.is_float = in.type() == CV_32F;
	return params;
}

static uint64_t calculateBitrate(int64_t pixels, float ratescale) {
	/*float bitrate = 1.0f;  // Megabits
	switch (def) {
	case definition_t::UHD4k	: bitrate = 40.0f; break;
	case definition_t::HTC_VIVE	: bitrate = 32.0f; break;
	case definition_t::HD1080	: bitrate = 12.0f; break;
	case definition_t::HD720	: bitrate = 8.0f; break;
	case definition_t::SD576	:
	case definition_t::SD480	: bitrate = 4.0f; break;
	case definition_t::LD360	: bitrate = 2.0f; break;
	default						: bitrate = 16.0f;
	}*/

	float bitrate = 8.0f * float(pixels);

	//bitrate *= 1000.0f*1000.0f;
	float minrate = 0.05f * bitrate;
	return uint64_t((bitrate - minrate)*ratescale + minrate);
}

/*
 * Check that codec configuration and surface data are all valid.
 */
static bool validate(const cv::cuda::GpuMat &in, ftl::codecs::Packet &pkt) {
	if (in.type() == CV_32F) pkt.flags |= kFlagFloat;
	else pkt.flags |= kFlagFlipRGB;

	// Remove unwanted flags
	if (in.type() == CV_32F && (pkt.flags & kFlagFlipRGB)) pkt.flags &= ~kFlagFlipRGB;
	if (in.type() == CV_8UC4 && (pkt.flags & kFlagFloat)) pkt.flags &= ~kFlagFloat;
	if (pkt.codec == codec_t::HEVC_LOSSLESS && (pkt.flags & kFlagMappedDepth)) pkt.flags &= ~kFlagMappedDepth;

	if (pkt.codec == codec_t::Any) pkt.codec = codec_t::HEVC;

	// Correct for mising flag
	if (pkt.codec == codec_t::HEVC && in.type() == CV_32F) {
		pkt.flags |= ftl::codecs::kFlagMappedDepth;
	}

	if (pkt.codec == codec_t::H264 && in.type() == CV_32F) {
		LOG(ERROR) << "Lossy compression not supported with H264 currently";
		return false;
	}

	if (pkt.frame_count == 0) {
		//pkt.definition = definition_t::Invalid;
		return false;
	}

	/*auto [tx,ty] = ftl::codecs::chooseTileConfig(pkt.frame_count);
	pkt.definition = (pkt.definition == definition_t::Any) ? ftl::codecs::findDefinition(in.cols/tx, in.rows/ty) : pkt.definition;
	if (pkt.definition == definition_t::Invalid || pkt.definition == definition_t::Any) {
		LOG(ERROR) << "Could not find appropriate definition";
		return false;
	}*/

	auto width = in.cols; //ftl::codecs::getWidth(pkt.definition)*tx;
	auto height = in.rows; //ftl::codecs::getHeight(pkt.definition)*ty;

	if (in.empty()) {
		LOG(WARNING) << "No data";
		return false;
	}

	if (in.type() != CV_32F && in.type() != CV_8UC4) {
		LOG(ERROR) << "Input type does not match given format";
		pkt.flags = 0;
		return false;
	}

	/*if (width != in.cols || height != in.rows) {
		// TODO: Resize if lower definition requested...
		LOG(ERROR) << "Input size does not match expected: " << in.cols << " != " << width;
		pkt.definition = definition_t::Invalid;
		return false;
	}*/

	return true;
}

bool NvidiaEncoder::encode(const cv::cuda::GpuMat &in, ftl::codecs::Packet &pkt) {
	//cudaSetDevice(0);

	if (pkt.codec != codec_t::Any && !supports(pkt.codec)) {
		pkt.codec = codec_t::Invalid;
		return false;
	}

	if (!validate(in, pkt)) return false;	
	if (!_createEncoder(in, pkt)) return false;

	const NvEncInputFrame* f = nvenc_->GetNextInputFrame();

	if (!params_.is_float) {
		cv::cuda::GpuMat surface(nvenc_->GetEncodeHeight(), nvenc_->GetEncodeWidth(), CV_8UC4, f->inputPtr, f->pitch);
		cv::cuda::cvtColor(in, surface, cv::COLOR_BGRA2RGBA, 0, stream_);
	} else if (params_.isLossy()) {
		ftl::cuda::depth_to_nv12_10(in, (ushort*)f->inputPtr, (ushort*)(((uchar*)f->inputPtr)+(nvenc_->GetEncodeHeight()*f->pitch)), f->pitch/2, 16.0f, stream_);
	} else {
		ftl::cuda::float_to_nv12_16bit((float*)in.data, in.step1(), (uchar*)f->inputPtr, f->pitch, nvenc_->GetEncodeWidth()/2, nvenc_->GetEncodeHeight(), cv::cuda::StreamAccessor::getStream(stream_));
	}

	// TODO: Use page locked memory?
	pkt.data.resize(ftl::codecs::kVideoBufferSize);

	// Make sure conversions complete...
	stream_.waitForCompletion();

	uint64_t cs = _encode(pkt.data.data(), pkt.data.size(), was_reset_);
	pkt.data.resize(cs);
	was_reset_ = false;

	if (cs == 0 || cs >= ftl::codecs::kVideoBufferSize) {
		LOG(ERROR) << "Could not encode video frame";
		return false;
	} else {
		return true;
	}
}

bool NvidiaEncoder::_createEncoder(const cv::cuda::GpuMat &in, const ftl::codecs::Packet &pkt) {
	Parameters params = generateParams(in, pkt);
	if (nvenc_ && (params == params_)) return true;

	uint64_t bitrate = calculateBitrate(in.cols*in.rows, float(pkt.bitrate)/255.0f) * pkt.frame_count;
	LOG(INFO) << "Calculated bitrate " << ((params.is_float) ? "(float)" : "(rgb)") << ": " << bitrate;
	
	params_ = params;

	const int fps = 1000/ftl::timer::getInterval();
	
	bool ish264 = pkt.codec == codec_t::H264 || pkt.codec == codec_t::H264_LOSSLESS;
	bool ishevc = !ish264;

	// Ensure we have a CUDA context
	cudaSafeCall(cudaDeviceSynchronize());
	CUcontext cudaContext;
	cuCtxGetCurrent(&cudaContext);    

	if (nvenc_) {
		LOG(INFO) << "Destroying old NVENC encoder";
		std::vector<std::vector<uint8_t>> tmp;
		nvenc_->EndEncode(tmp);
		nvenc_->DestroyEncoder();
		delete nvenc_;
		nvenc_ = nullptr;
	}

	// Create encoder
	try
	{
		NV_ENC_BUFFER_FORMAT bufferFormat;
		if (!params.is_float) bufferFormat = NV_ENC_BUFFER_FORMAT_ABGR;
		else if (!params.isLossy()) bufferFormat = NV_ENC_BUFFER_FORMAT_NV12;
		else bufferFormat = NV_ENC_BUFFER_FORMAT_YUV420_10BIT;

		nvenc_ = new NvEncoderCuda(cudaContext, params_.encodeWidth(), params_.encodeHeight(), bufferFormat, 0);

		NV_ENC_INITIALIZE_PARAMS initializeParams = { NV_ENC_INITIALIZE_PARAMS_VER };
		NV_ENC_CONFIG encodeConfig = { NV_ENC_CONFIG_VER };
		initializeParams.encodeConfig = &encodeConfig;

		GUID codecGUID = (ishevc) ? NV_ENC_CODEC_HEVC_GUID : NV_ENC_CODEC_H264_GUID;

		GUID presetGUID = NV_ENC_PRESET_LOW_LATENCY_HQ_GUID;
		if (!params.isLossy())
			presetGUID = NV_ENC_PRESET_LOSSLESS_DEFAULT_GUID; // NV_ENC_PRESET_LOSSLESS_HP_GUID

		nvenc_->CreateDefaultEncoderParams(&initializeParams, codecGUID, presetGUID);

		initializeParams.encodeWidth = params.encodeWidth();
		initializeParams.encodeHeight = params.encodeHeight();
		initializeParams.frameRateNum = fps;
		initializeParams.frameRateDen = 1;
		initializeParams.enablePTD = 1;

		encodeConfig.gopLength = NVENC_INFINITE_GOPLENGTH; // No B-frames
		encodeConfig.frameIntervalP = 1;

		if (ish264)
			encodeConfig.encodeCodecConfig.h264Config.idrPeriod = NVENC_INFINITE_GOPLENGTH;
		else {
			encodeConfig.encodeCodecConfig.hevcConfig.idrPeriod = NVENC_INFINITE_GOPLENGTH;

			if (params.is_float && params.isLossy()) {
				encodeConfig.encodeCodecConfig.hevcConfig.pixelBitDepthMinus8 = 2;  // For 10-bit colour
			}

			//if (this->compression == NVPIPE_LOSSY_10BIT_444 || this->compression == NVPIPE_LOSSY_8BIT_444) {
			//	encodeConfig.encodeCodecConfig.hevcConfig.chromaFormatIDC = 3;  // For Yuv444 (1 for 420)
			//}
		}

		if (params.isLossy())
		{
			encodeConfig.rcParams.averageBitRate = bitrate;
			encodeConfig.rcParams.rateControlMode = NV_ENC_PARAMS_RC_CBR_LOWDELAY_HQ;
			encodeConfig.rcParams.vbvBufferSize = encodeConfig.rcParams.averageBitRate * initializeParams.frameRateDen / initializeParams.frameRateNum; // bitrate / framerate = one frame
			encodeConfig.rcParams.maxBitRate = encodeConfig.rcParams.averageBitRate;
			encodeConfig.rcParams.vbvInitialDelay = encodeConfig.rcParams.vbvBufferSize;
		}

		nvenc_->CreateEncoder(&initializeParams);
	}
	catch (NVENCException& e)
	{
		throw FTL_Error("Failed to create encoder (" << e.getErrorString() << ", error " + std::to_string(e.getErrorCode()) << " = " + EncErrorCodeToString(e.getErrorCode()) << ")");
	}

	if (!nvenc_) {
		//LOG(ERROR) << "Could not create video encoder";
		return false;
	} else {
		LOG(INFO) << "NVENC encoder created";

		//nvenc_->SetIOCudaStreams(cv::cuda::StreamAccessor::getStream(stream_), cv::cuda::StreamAccessor::getStream(stream_));

		return true;
	}
}

uint64_t NvidiaEncoder::_encode(uint8_t* dst, uint64_t dstSize, bool forceIFrame) {
	std::vector<std::vector<uint8_t>> packets;

	try
	{
		if (forceIFrame)
		{
			NV_ENC_PIC_PARAMS params = {};
			params.encodePicFlags = NV_ENC_PIC_FLAG_FORCEIDR | NV_ENC_PIC_FLAG_OUTPUT_SPSPPS;

			nvenc_->EncodeFrame(packets, &params);
		}
		else
		{
			nvenc_->EncodeFrame(packets);
		}
	}
	catch (NVENCException& e)
	{
		throw FTL_Error("Encode failed (" << e.getErrorString() << ", error " << std::to_string(e.getErrorCode()) << " = " << EncErrorCodeToString(e.getErrorCode()) << ")");
	}

	// Copy output
	uint64_t size = 0;
	for (auto& p : packets)
	{
		if (size + p.size() <= dstSize)
		{
			memcpy(dst + size, p.data(), p.size());
			size += p.size();
		}
		else
		{
			throw FTL_Error("Encode output buffer overflow");
		}
	}

	return size;
}
