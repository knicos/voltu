#include <ftl/codecs/nvpipe_encoder.hpp>
#include <loguru.hpp>
#include <ftl/timer.hpp>
#include <ftl/codecs/bitrates.hpp>
#include <ftl/cuda_util.hpp>

#include <opencv2/core/cuda/common.hpp>

using ftl::codecs::NvPipeEncoder;
using ftl::codecs::bitrate_t;
using ftl::codecs::codec_t;
using ftl::codecs::definition_t;
using ftl::codecs::preset_t;
using ftl::codecs::CodecPreset;
using ftl::codecs::Packet;

NvPipeEncoder::NvPipeEncoder(definition_t maxdef,
			definition_t mindef) : Encoder(maxdef, mindef, ftl::codecs::device_t::Hardware) {
	nvenc_ = nullptr;
	current_definition_ = definition_t::HD1080;
	is_float_channel_ = false;
	was_reset_ = false;
	preference_ = codec_t::Any;
}

NvPipeEncoder::~NvPipeEncoder() {
	if (nvenc_) NvPipe_Destroy(nvenc_);
}

void NvPipeEncoder::reset() {
	was_reset_ = true;
}

bool NvPipeEncoder::supports(ftl::codecs::codec_t codec) {
	switch (codec) {
	case codec_t::H264:
	case codec_t::HEVC: preference_ = codec; return true;
	default: return false;
	}
}

/* Check preset resolution is not better than actual resolution. */
definition_t NvPipeEncoder::_verifiedDefinition(definition_t def, const cv::Mat &in) {
	int height = ftl::codecs::getHeight(def);

	// FIXME: Make sure this can't go forever
	while (height > in.rows) {
		def = static_cast<definition_t>(int(def)+1);
		height = ftl::codecs::getHeight(def);
	}

	return def;
}

void scaleDownAndPad(cv::Mat &in, cv::Mat &out) {
	const auto isize = in.size();
	const auto osize = out.size();
	cv::Mat tmp;
	
	if (isize != osize) {
		double x_scale = ((double) isize.width) / osize.width;
		double y_scale = ((double) isize.height) / osize.height;
		double x_scalei = 1.0 / x_scale;
		double y_scalei = 1.0 / y_scale;

		if (x_scale > 1.0 || y_scale > 1.0) {
			if (x_scale > y_scale) {
				cv::resize(in, tmp, cv::Size(osize.width, osize.height * x_scalei));
			} else {
				cv::resize(in, tmp, cv::Size(osize.width * y_scalei, osize.height));
			}
		}
		else { tmp = in; }
		
		if (tmp.size().width < osize.width || tmp.size().height < osize.height) {
			tmp.copyTo(out(cv::Rect(cv::Point2i(0, 0), tmp.size())));
		}
		else { out = tmp; }
	}
}

bool NvPipeEncoder::encode(const cv::Mat &in, definition_t odefinition, bitrate_t bitrate, const std::function<void(const ftl::codecs::Packet&)> &cb) {
	cudaSetDevice(0);
	auto definition = odefinition; //_verifiedDefinition(odefinition, in);

	auto width = ftl::codecs::getWidth(definition);
	auto height = ftl::codecs::getHeight(definition);

	cv::Mat tmp;
	if (width != in.cols || height != in.rows) {
		LOG(WARNING) << "Mismatch resolution with encoding resolution";
		if (in.type() == CV_32F) {
			cv::resize(in, tmp, cv::Size(width,height), 0.0, 0.0, cv::INTER_NEAREST);
		} else {
			cv::resize(in, tmp, cv::Size(width,height));
		}
	} else {
		tmp = in;
	}

	//LOG(INFO) << "Definition: " << ftl::codecs::getWidth(definition) << "x" << ftl::codecs::getHeight(definition);

	if (in.empty()) {
		LOG(ERROR) << "Missing data for Nvidia encoder";
		return false;
	}
	if (!_createEncoder(tmp, definition, bitrate)) return false;

	//LOG(INFO) << "NvPipe Encode: " << int(definition) << " " << in.cols;

	//cv::Mat tmp;
	if (tmp.type() == CV_32F) {
		tmp.convertTo(tmp, CV_16UC1, 1000);
	} else if (tmp.type() == CV_8UC3) {
		cv::cvtColor(tmp, tmp, cv::COLOR_BGR2RGBA);
	} else if (tmp.type() == CV_8UC4) {
		cv::cvtColor(tmp, tmp, cv::COLOR_BGRA2RGBA);
	} else {
		LOG(ERROR) << "Unsupported cv::Mat type in Nvidia encoder";
		return false;
	}

	// scale/pad to fit output format
	//cv::Mat tmp2 = cv::Mat::zeros(getHeight(odefinition), getWidth(odefinition), tmp.type());
	//scaleDownAndPad(tmp, tmp2);
	//std::swap(tmp, tmp2);

	Packet pkt;
	pkt.codec = (preference_ == codec_t::Any) ? codec_t::HEVC : preference_;
	pkt.definition = definition;
	pkt.block_total = 1;
	pkt.block_number = 0;
	pkt.flags = NvPipeEncoder::kFlagRGB;

	pkt.data.resize(ftl::codecs::kVideoBufferSize);
	uint64_t cs = NvPipe_Encode(
		nvenc_,
		tmp.data,
		tmp.step,
		pkt.data.data(),
		ftl::codecs::kVideoBufferSize,
		tmp.cols,
		tmp.rows,
		was_reset_		// Force IFrame!
	);
	pkt.data.resize(cs);
	was_reset_ = false;

	if (cs == 0) {
		LOG(ERROR) << "Could not encode video frame: " << NvPipe_GetError(nvenc_);
		return false;
	} else {
		cb(pkt);
		return true;
	}
}

bool NvPipeEncoder::_encoderMatch(const cv::Mat &in, definition_t def) {
	return ((in.type() == CV_32F && is_float_channel_) ||
		((in.type() == CV_8UC3 || in.type() == CV_8UC4) && !is_float_channel_)) && current_definition_ == def;
}

static uint64_t calculateBitrate(definition_t def, bitrate_t rate) {
	float scale = 1.0f;
	switch (rate) {
	case bitrate_t::High		: break;
	case bitrate_t::Standard	: scale = 0.5f; break;
	case bitrate_t::Low			: scale = 0.25f; break;
	}

	float bitrate = 1.0f;  // Megabits
	switch (def) {
	case definition_t::UHD4k	: bitrate = 24.0f; break;
	case definition_t::HTC_VIVE	: bitrate = 16.0f; break;
	case definition_t::HD1080	: bitrate = 16.0f; break;
	case definition_t::HD720	: bitrate = 8.0f; break;
	case definition_t::SD576	:
	case definition_t::SD480	: bitrate = 4.0f; break;
	case definition_t::LD360	: bitrate = 2.0f; break;
	default						: bitrate = 8.0f;
	}

	return uint64_t(bitrate * 1000.0f * 1000.0f * scale);
}

bool NvPipeEncoder::_createEncoder(const cv::Mat &in, definition_t def, bitrate_t rate) {
	if (_encoderMatch(in, def) && nvenc_) return true;

	uint64_t bitrate = calculateBitrate(def, rate);
	LOG(INFO) << "Calculated bitrate: " << bitrate;

	if (in.type() == CV_32F) is_float_channel_ = true;
	else is_float_channel_ = false;
	current_definition_ = def;

	if (nvenc_) NvPipe_Destroy(nvenc_);
	const int fps = 1000/ftl::timer::getInterval();
	nvenc_ = NvPipe_CreateEncoder(
		(is_float_channel_) ? NVPIPE_UINT16 : NVPIPE_RGBA32,
		(preference_ == codec_t::Any || preference_ == codec_t::HEVC) ? NVPIPE_HEVC : NVPIPE_H264,
		(is_float_channel_) ? NVPIPE_LOSSLESS : NVPIPE_LOSSY,
		bitrate,
		fps,				// FPS
		ftl::codecs::getWidth(def),	// Output Width
		ftl::codecs::getHeight(def)	// Output Height
	);

	if (!nvenc_) {
		LOG(ERROR) << "Could not create video encoder: " << NvPipe_GetError(NULL);
		return false;
	} else {
		LOG(INFO) << "NvPipe encoder created";
		return true;
	}
}
