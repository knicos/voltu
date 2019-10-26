#include <ftl/codecs/opencv_encoder.hpp>

#include <loguru.hpp>
#include <vector>

using ftl::codecs::definition_t;
using ftl::codecs::codec_t;
using ftl::codecs::bitrate_t;
using ftl::codecs::OpenCVEncoder;
using ftl::codecs::preset_t;
using ftl::codecs::CodecPreset;
using std::vector;

OpenCVEncoder::OpenCVEncoder(ftl::codecs::definition_t maxdef,
			ftl::codecs::definition_t mindef) : Encoder(maxdef, mindef, ftl::codecs::device_t::Software) {
	jobs_ = 0;
}

OpenCVEncoder::~OpenCVEncoder() {
    
}

bool OpenCVEncoder::supports(ftl::codecs::codec_t codec) {
	switch (codec) {
	case codec_t::JPG:
	case codec_t::PNG: return true;
	default: return false;
	}
}

bool OpenCVEncoder::encode(const cv::Mat &in, definition_t definition, bitrate_t bitrate, const std::function<void(const ftl::codecs::Packet&)> &cb) {
	cv::Mat tmp;
	bool is_colour = in.type() != CV_32F;
	current_definition_ = definition;

	// Scale down image to match requested definition...
	if (ftl::codecs::getHeight(current_definition_) < in.rows) {
		cv::resize(in, tmp, cv::Size(ftl::codecs::getWidth(current_definition_), ftl::codecs::getHeight(current_definition_)), 0, 0, (is_colour) ? 1 : cv::INTER_NEAREST);
	} else {
		tmp = in;
	}

	// Represent float at 16bit int
    if (!is_colour) {
		tmp.convertTo(tmp, CV_16UC1, 1000);
	}

	chunk_dim_ = (definition == definition_t::LD360) ? 1 : 4;
	chunk_count_ = chunk_dim_ * chunk_dim_;
	jobs_ = chunk_count_;

	for (int i=0; i<chunk_count_; ++i) {
		// Add chunk job to thread pool
		ftl::pool.push([this,i,&tmp,cb,is_colour,bitrate](int id) {
			ftl::codecs::Packet pkt;
			pkt.block_number = i;
			pkt.block_total = chunk_count_;
			pkt.definition = current_definition_;
			pkt.codec = (is_colour) ? codec_t::JPG : codec_t::PNG;

			try {
				_encodeBlock(tmp, pkt, bitrate);
			} catch(...) {
				LOG(ERROR) << "OpenCV encode block exception: " << i;
			}

			try {
				cb(pkt);
			} catch(...) {
				LOG(ERROR) << "OpenCV encoder callback exception";
			}

			std::unique_lock<std::mutex> lk(job_mtx_);
			--jobs_;
			if (jobs_ == 0) job_cv_.notify_one();
		});
	}

	std::unique_lock<std::mutex> lk(job_mtx_);
	job_cv_.wait_for(lk, std::chrono::seconds(20), [this]{ return jobs_ == 0; });
	if (jobs_ != 0) {
		LOG(FATAL) << "Deadlock detected (" << jobs_ << ")";
	}

	return true;
}

bool OpenCVEncoder::_encodeBlock(const cv::Mat &in, ftl::codecs::Packet &pkt, bitrate_t bitrate) {
	int chunk_width = in.cols / chunk_dim_;
	int chunk_height = in.rows / chunk_dim_;

	// Build chunk heads
	int cx = (pkt.block_number % chunk_dim_) * chunk_width;
	int cy = (pkt.block_number / chunk_dim_) * chunk_height;
	cv::Rect roi(cx,cy,chunk_width,chunk_height);
	cv::Mat chunkHead = in(roi);

	if (pkt.codec == codec_t::PNG) {
		vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 1};
		if (!cv::imencode(".png", chunkHead, pkt.data, params)) {
			LOG(ERROR) << "PNG Encoding error";
			return false;
		}
		return true;
	} else if (pkt.codec == codec_t::JPG) {
		int q = 95;

		switch (bitrate) {
		case bitrate_t::High		: q = 95; break;
		case bitrate_t::Standard	: q = 75; break;
		case bitrate_t::Low			: q = 50; break;
		}

		vector<int> params = {cv::IMWRITE_JPEG_QUALITY, q};
		cv::imencode(".jpg", chunkHead, pkt.data, params);
		return true;
	} else {
		LOG(ERROR) << "Bad channel configuration: imagetype=" << in.type(); 
	}

	return false;
}

