/*
 * Copyright 2019 Nicolas Pope
 */

#include <ftl/synched.hpp>

using ftl::SyncSource;
using cv::Mat;

SyncSource::SyncSource() {
	channels_.push_back(Mat());
	channels_.push_back(Mat());
}

void SyncSource::addChannel(const std::string &c) {
}

void SyncSource::feed(int channel, cv::Mat &m, double ts) {
	if (channel > static_cast<int>(channels_.size())) return;
	channels_[channel] = m;
}

bool SyncSource::get(int channel, cv::Mat &m) {
	if (channel > static_cast<int>(channels_.size())) return false;
	m = channels_[channel];
	return true;
}

double SyncSource::latency() const {
	return 0.0;
}

