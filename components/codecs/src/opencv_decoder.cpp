#include <ftl/codecs/opencv_decoder.hpp>

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

	int chunk_dim = std::sqrt(pkt.block_total);
	int chunk_width = out.cols / chunk_dim;
	int chunk_height = out.rows / chunk_dim;

	// Build chunk head
	int cx = (pkt.block_number % chunk_dim) * chunk_width;
	int cy = (pkt.block_number / chunk_dim) * chunk_height;
	cv::Rect roi(cx,cy,chunk_width,chunk_height);
	cv::cuda::GpuMat chunkHead = out(roi);

	//LOG(INFO) << "DECODE JPEG " << (int)pkt.block_number << "/" << chunk_dim;

	// Decode in temporary buffers to prevent long locks
	cv::imdecode(pkt.data, cv::IMREAD_UNCHANGED, &tmp_);

	// Apply colour correction to chunk
	//ftl::rgbd::colourCorrection(tmp_rgb, gamma_, temperature_);


	// TODO:(Nick) Decode directly into double buffer if no scaling
	// Can either check JPG/PNG headers or just use pkt definition.

	// Original size so just copy
	if (tmp_.cols == chunkHead.cols) {
		if (!tmp_.empty() && tmp_.type() == CV_16U && chunkHead.type() == CV_32F) {
			tmp_.convertTo(tmp_, CV_32FC1, 1.0f/1000.0f);
			chunkHead.upload(tmp_);
		} else if (!tmp_.empty() && tmp_.type() == CV_8UC3 && chunkHead.type() == CV_8UC3) {
			//tmp_.copyTo(chunkHead);
			chunkHead.upload(tmp_);
		} else {
			// Silent ignore?
		}
	// Downsized so needs a scale up
	} else {
		if (!tmp_.empty() && tmp_.type() == CV_16U && chunkHead.type() == CV_32F) {
			tmp_.convertTo(tmp_, CV_32FC1, 1.0f/1000.0f); //(16.0f*10.0f));
			cv::resize(tmp_, tmp_, chunkHead.size(), 0, 0, cv::INTER_NEAREST);
			chunkHead.upload(tmp_);
		} else if (!tmp_.empty() && tmp_.type() == CV_8UC3 && chunkHead.type() == CV_8UC3) {
			cv::resize(tmp_, tmp_, chunkHead.size());
			chunkHead.upload(tmp_);
		} else {
			// Silent ignore?
		}
	}

	return true;
}
