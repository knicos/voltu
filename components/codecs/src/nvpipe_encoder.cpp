#include <ftl/codecs/nvpipe_encoder.hpp>
#include <loguru.hpp>
#include <ftl/timer.hpp>
#include <ftl/codecs/codecs.hpp>
#include <ftl/cuda_util.hpp>

#include <opencv2/core/cuda/common.hpp>

#include <ftl/codecs/depth_convert_cuda.hpp>

using ftl::codecs::NvPipeEncoder;
using ftl::codecs::bitrate_t;
using ftl::codecs::codec_t;
using ftl::codecs::definition_t;
using ftl::codecs::format_t;
using ftl::codecs::Packet;

NvPipeEncoder::NvPipeEncoder(definition_t maxdef,
			definition_t mindef) : Encoder(maxdef, mindef, ftl::codecs::device_t::Hardware) {
	nvenc_ = nullptr;
	was_reset_ = false;
}

NvPipeEncoder::~NvPipeEncoder() {
	if (nvenc_) NvPipe_Destroy(nvenc_);
}

void NvPipeEncoder::reset() {
	was_reset_ = true;
}

bool NvPipeEncoder::supports(ftl::codecs::codec_t codec) {
	switch (codec) {
	case codec_t::H264_LOSSLESS:
	case codec_t::HEVC_LOSSLESS:
	case codec_t::H264:
	case codec_t::HEVC: return true;
	default: return false;
	}
}

/* Check preset resolution is not better than actual resolution. */
/*definition_t NvPipeEncoder::_verifiedDefinition(definition_t def, const cv::cuda::GpuMat &in) {
	int height = ftl::codecs::getHeight(def);

	while (height > in.rows) {
		def = static_cast<definition_t>(int(def)+1);
		height = ftl::codecs::getHeight(def);
	}

	return def;
}*/

static bool isLossy(codec_t c) {
	return !(c == codec_t::HEVC_LOSSLESS || c == codec_t::H264_LOSSLESS);
}

static bool sanityFormat(int type, ftl::codecs::format_t fmt) {
	switch(fmt) {
	case format_t::BGRA8	:
	case format_t::RGBA8	: return type == CV_8UC4;
	case format_t::VUYA16	: return type == CV_8UC4;
	case format_t::F32		: return type == CV_32F;
	case format_t::U16		: return type == CV_16U;
	}
	return false;
}

static ftl::codecs::format_t formatFromPacket(const ftl::codecs::Packet &pkt) {
	if (pkt.flags & ftl::codecs::kFlagFloat) {
		return (pkt.flags & ftl::codecs::kFlagMappedDepth) ? format_t::VUYA16 : format_t::U16;
	} else {
		return (pkt.flags & ftl::codecs::kFlagFlipRGB) ? format_t::BGRA8 : format_t::RGBA8;
	}
}

static uint64_t calculateBitrate(definition_t def, float ratescale) {
	float bitrate = 1.0f;  // Megabits
	switch (def) {
	case definition_t::UHD4k	: bitrate = 40.0f; break;
	case definition_t::HTC_VIVE	: bitrate = 32.0f; break;
	case definition_t::HD1080	: bitrate = 12.0f; break;
	case definition_t::HD720	: bitrate = 8.0f; break;
	case definition_t::SD576	:
	case definition_t::SD480	: bitrate = 4.0f; break;
	case definition_t::LD360	: bitrate = 2.0f; break;
	default						: bitrate = 16.0f;
	}

	bitrate *= 1000.0f*1000.0f;
	float minrate = 0.05f * bitrate;
	return uint64_t((bitrate - minrate)*ratescale + minrate);
}

bool NvPipeEncoder::encode(const cv::cuda::GpuMat &in, ftl::codecs::Packet &pkt) {
	//cudaSetDevice(0);

	if (pkt.codec != codec_t::Any && !supports(pkt.codec)) {
		pkt.codec = codec_t::Invalid;
		return false;
	}

	// Correct for mising flag
	if (pkt.codec == codec_t::HEVC && (pkt.flags & ftl::codecs::kFlagFloat) && in.type() == CV_8UC4) {
		pkt.flags |= ftl::codecs::kFlagMappedDepth;
	}

	ftl::codecs::format_t fmt = formatFromPacket(pkt);

	if (pkt.frame_count == 0) {
		pkt.definition = definition_t::Invalid;
		return false;
	}

	//bool is_stereo = pkt.flags & ftl::codecs::kFlagStereo;

	auto [tx,ty] = ftl::codecs::chooseTileConfig(pkt.frame_count);
	pkt.definition = (pkt.definition == definition_t::Any) ? ftl::codecs::findDefinition(in.cols/tx, in.rows/ty) : pkt.definition;
	if (pkt.definition == definition_t::Invalid || pkt.definition == definition_t::Any) {
		LOG(ERROR) << "Could not find appropriate definition";
		return false;
	}

	auto width = ftl::codecs::getWidth(pkt.definition);
	auto height = ftl::codecs::getHeight(pkt.definition);

	if (in.empty()) {
		LOG(WARNING) << "No data";
		return false;
	}

	if (!sanityFormat(in.type(), fmt)) {
		LOG(ERROR) << "Input type does not match given format";
		pkt.flags = 0;
		return false;
	}

	if (tx*width != in.cols || ty*height != in.rows) {
		// TODO: Resize if lower definition requested...
		LOG(ERROR) << "Input size does not match expected: " << in.cols << " != " << tx*width;
		pkt.definition = definition_t::Invalid;
		return false;
	}

	cv::cuda::GpuMat tmp;
	/*if (width != in.cols || height != in.rows) {
		LOG(WARNING) << "Mismatch resolution with encoding resolution";
		if (in.type() == CV_32F) {
			cv::cuda::resize(in, tmp_, cv::Size(width,height), 0.0, 0.0, cv::INTER_NEAREST, stream_);
		} else {
			cv::cuda::resize(in, tmp_, cv::Size(width,height), 0.0, 0.0, cv::INTER_LINEAR, stream_);
		}
		tmp = tmp_;
	} else {*/
		tmp = in;
	//}

	//LOG(INFO) << "Definition: " << ftl::codecs::getWidth(pkt.definition) << "x" << ftl::codecs::getHeight(pkt.definition);

	if (in.empty()) {
		LOG(ERROR) << "Missing data for Nvidia encoder";
		return false;
	}

	if (pkt.codec == codec_t::Any)
		pkt.codec = ((pkt.flags & ftl::codecs::kFlagFloat) && !(pkt.flags & ftl::codecs::kFlagMappedDepth)) ? codec_t::HEVC_LOSSLESS : codec_t::HEVC;

	if (!_createEncoder(pkt, fmt)) return false;

	// Doesn't seem to work
	/*if (isLossy(pkt.codec) && pkt.bitrate != last_bitrate_) {
		uint64_t bitrate = calculateBitrate(pkt.definition, float(pkt.bitrate)/255.0f) * pkt.frame_count;
		const int fps = 1000/ftl::timer::getInterval();
		LOG(INFO) << "Changing bitrate: " << bitrate;
		NvPipe_SetBitrate(nvenc_, bitrate, fps);
		last_bitrate_ = pkt.bitrate;
	}*/

	//LOG(INFO) << "NvPipe Encode: " << int(definition) << " " << in.cols;

	//pkt.flags = 0;

	//cv::Mat tmp;
	/*if (tmp.type() == CV_32F) {
		if (isLossy(pkt.codec)) {
			// Use special encoding transform
			tmp2_.create(tmp.size(), CV_8UC4);
			ftl::cuda::depth_to_vuya(tmp, tmp2_, 16.0f, stream_);
			pkt.flags |= NvPipeEncoder::kFlagMappedDepth;
		} else {
			tmp.convertTo(tmp2_, CV_16UC1, 1000, stream_);
		}
	} else if (tmp.type() == CV_8UC3) {
		cv::cuda::cvtColor(tmp, tmp2_, cv::COLOR_BGR2RGBA, 0, stream_);
	} else if (tmp.type() == CV_8UC4) {
		if (fmt == format_t::BGRA8) {
			cv::cuda::cvtColor(tmp, tmp2_, cv::COLOR_BGRA2RGBA, 0, stream_);
			pkt.flags |= NvPipeEncoder::kFlagRGB;
		} else if (fmt == format_t::VUYA16) {
			tmp2_ = tmp;
		}
	//} else if (tmp.type() == CV_16UC4) {

	} else {
		LOG(ERROR) << "Unsupported cv::Mat type in Nvidia encoder";
		return false;
	}*/

	// Make sure conversions complete...
	//stream_.waitForCompletion();

	//pkt.flags = NvPipeEncoder::kFlagRGB | NvPipeEncoder::kFlagMappedDepth;

	// TODO: Use page locked memory?
	pkt.data.resize(ftl::codecs::kVideoBufferSize);
	uint64_t cs = NvPipe_Encode(
		nvenc_,
		in.data,
		in.step,
		pkt.data.data(),
		ftl::codecs::kVideoBufferSize,
		in.cols,
		in.rows,
		was_reset_		// Force IFrame!
	);
	pkt.data.resize(cs);
	was_reset_ = false;

	if (cs == 0 || cs >= ftl::codecs::kVideoBufferSize) {
		LOG(ERROR) << "Could not encode video frame: " << NvPipe_GetError(nvenc_);
		return false;
	} else {
		return true;
	}
}

static NvPipe_Codec selectCodec(const Packet &pkt) {
	return (pkt.codec == codec_t::HEVC || pkt.codec == codec_t::HEVC_LOSSLESS) ? NVPIPE_HEVC : NVPIPE_H264;
}

static NvPipe_Compression selectCompression(const Packet &pkt, format_t fmt) {
	switch (fmt) {
	case format_t::BGRA8	:
	case format_t::RGBA8	: return NVPIPE_LOSSY;
	case format_t::F32		: return (isLossy(pkt.codec)) ? NVPIPE_LOSSY_10BIT_420 : NVPIPE_LOSSLESS;
	case format_t::VUYA16	: return NVPIPE_LOSSY_10BIT_420;  // FIXME: Check codec.
	case format_t::U16		: return NVPIPE_LOSSLESS;
	}
	return NVPIPE_LOSSY;
}

static NvPipe_Format selectFormat(const Packet &pkt, format_t fmt) {
	switch (fmt) {
	case format_t::BGRA8	:
	case format_t::RGBA8	: return NVPIPE_RGBA32;
	case format_t::F32		: return (isLossy(pkt.codec)) ? NVPIPE_YUV32 : NVPIPE_UINT16;
	case format_t::U16		: return NVPIPE_UINT16;
	case format_t::VUYA16	: return NVPIPE_YUV32;
	}
	return NVPIPE_RGBA32;
}

bool NvPipeEncoder::_encoderMatch(const ftl::codecs::Packet &pkt, format_t fmt) {
	return	compression_ == selectCompression(pkt, fmt) &&
			format_ == selectFormat(pkt, fmt) &&
			codec_ == selectCodec(pkt) && last_bitrate_ == pkt.bitrate;
}

bool NvPipeEncoder::_createEncoder(const ftl::codecs::Packet &pkt, format_t fmt) {
	if (_encoderMatch(pkt, fmt) && nvenc_) return true;

	uint64_t bitrate = calculateBitrate(pkt.definition, float(pkt.bitrate)/255.0f) * pkt.frame_count;
	//if (is_float_channel_) bitrate *= 2.0f;
	//LOG(INFO) << "Calculated bitrate: " << bitrate;
	
	format_ = selectFormat(pkt, fmt);
	compression_ = selectCompression(pkt, fmt);
	codec_ = selectCodec(pkt);
	last_bitrate_ = pkt.bitrate;

	if (nvenc_) NvPipe_Destroy(nvenc_);
	const int fps = 1000/ftl::timer::getInterval();
	nvenc_ = NvPipe_CreateEncoder(
		format_,
		codec_,
		compression_,
		bitrate,
		fps,				// FPS
		ftl::codecs::getWidth(pkt.definition),	// Output Width
		ftl::codecs::getHeight(pkt.definition)	// Output Height
	);

	if (!nvenc_) {
		LOG(ERROR) << "Could not create video encoder: " << NvPipe_GetError(NULL);
		return false;
	} else {
		LOG(INFO) << "NvPipe encoder created";
		return true;
	}
}
