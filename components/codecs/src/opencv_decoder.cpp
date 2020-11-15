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
	/*int chunk_dim = 1; //std::sqrt(pkt.frame_count);
	int chunk_width = out.cols / chunk_dim;
	int chunk_height = out.rows / chunk_dim;

	// Build chunk head
	int cx = 0; //(pkt.block_number % chunk_dim) * chunk_width;
	int cy = 0; //(pkt.block_number / chunk_dim) * chunk_height;
	cv::Rect roi(cx,cy,chunk_width,chunk_height);
	cv::cuda::GpuMat chunkHead = out(roi);*/

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

	// Apply colour correction to chunk
	//ftl::rgbd::colourCorrection(tmp_rgb, gamma_, temperature_);

	// TODO:(Nick) Decode directly into double buffer if no scaling
	// Can either check JPG/PNG headers or just use pkt definition.

	// Original size so just copy
	//if (tmp_.cols == chunkHead.cols) {
		if (!tmp_.empty() && tmp_.type() == CV_16U) {
			tmp_.convertTo(tmp_, CV_32FC1, 1.0f/1000.0f);
			out.upload(tmp_);
		} else if (!tmp_.empty() && tmp_.type() == CV_8UC4) {
			//tmp_.copyTo(chunkHead);
			out.upload(tmp_);
		} else if (!tmp_.empty() && tmp_.type() == CV_16U) {
			out.upload(tmp_);
		} else if (!tmp_.empty() && tmp_.type() == CV_8UC1) {
			out.upload(tmp_);
		} else {
			// Silent ignore?
		}
	// Downsized so needs a scale up
	/*} else {
		if (!tmp_.empty() && tmp_.type() == CV_16U && chunkHead.type() == CV_32F) {
			tmp_.convertTo(tmp_, CV_32FC1, 1.0f/1000.0f); //(16.0f*10.0f));
			cv::resize(tmp_, tmp_, chunkHead.size(), 0, 0, cv::INTER_NEAREST);
			chunkHead.upload(tmp_);
		} else if (!tmp_.empty() && tmp_.type() == CV_8UC4 && chunkHead.type() == CV_8UC4) {
			cv::resize(tmp_, tmp_, chunkHead.size());
			chunkHead.upload(tmp_);
		} else {
			// Silent ignore?
		}
	}*/

	return true;
}
