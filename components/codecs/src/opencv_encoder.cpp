#include <ftl/codecs/opencv_encoder.hpp>

#include <opencv2/opencv.hpp>

#include <loguru.hpp>
#include <vector>

using ftl::codecs::definition_t;
using ftl::codecs::codec_t;
using ftl::codecs::OpenCVEncoder;
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

bool OpenCVEncoder::encode(const cv::cuda::GpuMat &in, ftl::codecs::Packet &pkt) {
	bool is_colour = !(pkt.flags & ftl::codecs::kFlagFloat);

	if (is_colour && in.type() != CV_8UC4) return false;
	if (!is_colour && in.type() == CV_8UC4) {
		LOG(ERROR) << "OpenCV Encoder doesn't support lossy depth";
		return false;
	}

	pkt.definition = (pkt.definition == definition_t::Any) ? ftl::codecs::findDefinition(in.cols, in.rows) : pkt.definition;

	if (pkt.definition == definition_t::Invalid || pkt.definition == definition_t::Any) {
		LOG(ERROR) << "Invalid definition";
		return false;
	}

	// Ensure definition does not exceed max
	current_definition_ = pkt.definition; //((int)pkt.definition < (int)max_definition) ? max_definition : pkt.definition;

	in.download(tmp_);
	//CHECK(cv::Size(ftl::codecs::getWidth(definition), ftl::codecs::getHeight(definition)) == in.size()); 

	if (!is_colour) {
		tmp_.convertTo(tmp_, CV_16U, 1000.0f);
	}

	int width = ftl::codecs::getWidth(current_definition_);
	int height = ftl::codecs::getHeight(current_definition_);

	// Scale down image to match requested definition...
	/*if (ftl::codecs::getHeight(current_definition_) < in.rows) {
		cv::resize(tmp_, tmp_, cv::Size(ftl::codecs::getWidth(current_definition_), ftl::codecs::getHeight(current_definition_)), 0, 0, (is_colour) ? 1 : cv::INTER_NEAREST);
	} else {
		
	}*/
	if (width != in.cols || height != in.rows) {
		LOG(ERROR) << "Input does not match requested definition";
		return false;
	}

	if (pkt.codec == codec_t::Any) pkt.codec = (is_colour) ? codec_t::JPG : codec_t::PNG;

	//for (int i=0; i<chunk_count_; ++i) {
		// Add chunk job to thread pool
		//ftl::pool.push([this,i,cb,is_colour,bitrate](int id) {
			//ftl::codecs::Packet pkt;
			//pkt.bitrate = 0;
			//pkt.frame_count = 1;
			//pkt.definition = current_definition_;
			//pkt.codec = (is_colour) ? codec_t::JPG : codec_t::PNG;

			try {
				_encodeBlock(tmp_, pkt);
			} catch(...) {
				LOG(ERROR) << "OpenCV encode block exception: ";
			}

			//std::unique_lock<std::mutex> lk(job_mtx_);
			//--jobs_;
			//if (jobs_ == 0) job_cv_.notify_one();
		//});
	//}

	//std::unique_lock<std::mutex> lk(job_mtx_);
	//job_cv_.wait_for(lk, std::chrono::seconds(20), [this]{ return jobs_ == 0; });
	//if (jobs_ != 0) {
	//	LOG(FATAL) << "Deadlock detected (" << jobs_ << ")";
	//}

	return true;
}

bool OpenCVEncoder::_encodeBlock(const cv::Mat &in, ftl::codecs::Packet &pkt) {
	//int chunk_width = in.cols / 1;
	//int chunk_height = in.rows / 1;

	// Build chunk heads
	//int cx = 0; //(pkt.block_number % chunk_dim_) * chunk_width;
	//int cy = 0; //(pkt.block_number / chunk_dim_) * chunk_height;
	//cv::Rect roi(cx,cy,chunk_width,chunk_height);

	cv::Mat chunkHead = in;

	if (pkt.codec == codec_t::PNG) {
		vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 1};
		if (!cv::imencode(".png", chunkHead, pkt.data, params)) {
			LOG(ERROR) << "PNG Encoding error";
			return false;
		}
		return true;
	} else if (pkt.codec == codec_t::JPG) {
		int q = int((95.0f - 50.0f) * (float(pkt.bitrate)/255.0f) + 50.0f);

		vector<int> params = {cv::IMWRITE_JPEG_QUALITY, q};
		cv::imencode(".jpg", chunkHead, pkt.data, params);
		return true;
	} else {
		LOG(ERROR) << "Bad channel configuration: imagetype=" << in.type(); 
	}

	return false;
}

