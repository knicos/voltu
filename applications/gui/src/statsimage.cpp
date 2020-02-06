#include "statsimage.hpp"

using ftl::gui::StatisticsImage;

StatisticsImage::StatisticsImage(cv::Size size) :
	StatisticsImage(size, std::numeric_limits<float>::infinity()) {}

StatisticsImage::StatisticsImage(cv::Size size, float max_f) {
	size_ = size;
	n_ = 0.0f;
	data_ = cv::Mat(size, CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));

	// TODO
	if (!std::isinf(max_f)) {
		LOG(WARNING) << "TODO: max_f_ not used. Values calculated for all samples";
	}
}

void StatisticsImage::reset() {
	n_ = 0.0f;
	data_ = cv::Scalar(0.0, 0.0, 0.0);
}

void StatisticsImage::update(const cv::Mat &in) {
	DCHECK(in.type() == CV_32F);
	DCHECK(in.size() == size_);
	
	n_ = n_ + 1.0f;

	// Welford's Method
	for (int row = 0; row < in.rows; row++) {
		float* ptr_data = data_.ptr<float>(row);
		const float* ptr_in = in.ptr<float>(row);

		for (int col = 0; col < in.cols; col++, ptr_in++) {
			float x = *ptr_in;
			float &m = *ptr_data++;
			float &s = *ptr_data++;
			float &f = *ptr_data++;
			float m_prev = m;

			if (!ftl::rgbd::isValidDepth(x)) continue;

			f = f + 1.0f;
			m = m + (x - m) / f;
			s = s + (x - m) * (x - m_prev);
		}
	}
}

void StatisticsImage::getVariance(cv::Mat &out) {
	std::vector<cv::Mat> channels(3);
	cv::split(data_, channels);
	cv::divide(channels[1], channels[2], out);
}

void StatisticsImage::getStdDev(cv::Mat &out) {
	getVariance(out);
	cv::sqrt(out, out);
}

void StatisticsImage::getMean(cv::Mat &out) {
	std::vector<cv::Mat> channels(3);
	cv::split(data_, channels);
	out = channels[0];
}

void StatisticsImage::getValidRatio(cv::Mat &out) {
	std::vector<cv::Mat> channels(3);
	cv::split(data_, channels);
	cv::divide(channels[2], n_, out);
}
