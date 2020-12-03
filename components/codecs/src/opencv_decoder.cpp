/**
 * @file opencv_decoder.cpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#include <ftl/codecs/opencv_decoder.hpp>

#include <opencv2/opencv.hpp>

#include <loguru.hpp>

using ftl::codecs::OpenCVDecoder;
using ftl::codecs::codec_t;

OpenCVDecoder::OpenCVDecoder() {

}

OpenCVDecoder::~OpenCVDecoder() {

}

bool OpenCVDecoder::accepts(const ftl::codecs::Packet &pkt) {
	return (pkt.codec == codec_t::JPG || pkt.codec == codec_t::PNG);
}

bool OpenCVDecoder::decode(const ftl::codecs::Packet &pkt, cv::cuda::GpuMat &out) { 
	// TODO: (nick) Move to member variables
	cv::Mat tmp2_, tmp_;

	// Decode in temporary buffers to prevent long locks
	cv::imdecode(pkt.data, cv::IMREAD_UNCHANGED, &tmp2_);

	if (tmp2_.type() == CV_8UC3) {
		cv::cvtColor(tmp2_, tmp_, cv::COLOR_RGB2BGRA);
	} else if (tmp2_.type() == CV_8U) {
		tmp_ = tmp2_;
	} else {
		if (pkt.flags & ftl::codecs::kFlagFlipRGB) {
			cv::cvtColor(tmp2_, tmp_, cv::COLOR_RGBA2BGRA);
		} else {
			tmp_ = tmp2_;
		}
	}

	if (!tmp_.empty() && tmp_.type() == CV_16U) {
		tmp_.convertTo(tmp_, CV_32FC1, 1.0f/1000.0f);
		out.upload(tmp_);
	} else if (!tmp_.empty() && tmp_.type() == CV_8UC4) {
		out.upload(tmp_);
	} else if (!tmp_.empty() && tmp_.type() == CV_16U) {
		out.upload(tmp_);
	} else if (!tmp_.empty() && tmp_.type() == CV_8UC1) {
		out.upload(tmp_);
	} else {
		// Silent ignore?
	}
	

	return true;
}
