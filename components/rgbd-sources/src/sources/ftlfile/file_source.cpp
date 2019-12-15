#include "file_source.hpp"

#include <ftl/codecs/hevc.hpp>
#include <ftl/timer.hpp>

using ftl::rgbd::detail::FileSource;
using ftl::codecs::codec_t;
using ftl::codecs::Channel;

void FileSource::_createDecoder(int ix, const ftl::codecs::Packet &pkt) {
	if (decoders_[ix]) {
		if (!decoders_[ix]->accepts(pkt)) {
			ftl::codecs::free(decoders_[ix]);
		} else {
			return;
		}
	}

	DLOG(INFO) << "Create a decoder: " << ix;
	decoders_[ix] = ftl::codecs::allocateDecoder(pkt);
}

FileSource::FileSource(ftl::rgbd::Source *s, ftl::rgbd::Player *r, int sid) : ftl::rgbd::detail::Source(s) {
    reader_ = r;
	has_calibration_ = false;
	decoders_[0] = nullptr;
	decoders_[1] = nullptr;
	cache_read_ = -1;
	cache_write_ = 0;
	realtime_ = host_->value("realtime", true);
	timestamp_ = r->getStartTime();
	sourceid_ = sid;
	freeze_ = host_->value("freeze", false);
	have_frozen_ = false;

	host_->on("freeze", [this](const ftl::config::Event &e) {
		have_frozen_ = false;
		freeze_ = host_->value("freeze", false);
	});

    r->onPacket(sid, [this](const ftl::codecs::StreamPacket &spkt, ftl::codecs::Packet &pkt) {
		host_->notifyRaw(spkt, pkt);

		// Some channels are to be directly handled by the source object and
		// do not proceed to any subsequent step.
		// FIXME: Potential problem, these get processed at wrong time
		if (spkt.channel == Channel::Configuration) {
			std::tuple<std::string, std::string> cfg;
			auto unpacked = msgpack::unpack((const char*)pkt.data.data(), pkt.data.size());
			unpacked.get().convert(cfg);

			LOG(INFO) << "Config Received: " << std::get<1>(cfg);
			return;
		}
		else if (spkt.channel == Channel::Calibration) {
			_processCalibration(pkt);
			return;
		} else if (spkt.channel == Channel::Pose) {
			_processPose(pkt);
			return;
		}

		// FIXME: For bad and old FTL files where wrong channel is used
		if (pkt.codec == codec_t::POSE) {
			_processPose(pkt);
			return;
		} else if (pkt.codec == codec_t::CALIBRATION) {
			_processCalibration(pkt);
			return;
		}


		// TODO: Check I-Frames for H264
		if (pkt.codec == codec_t::HEVC) {
			if (ftl::codecs::hevc::isIFrame(pkt.data)) _removeChannel(spkt.channel);
		}
		cache_[cache_write_].emplace_back();
		auto &c = cache_[cache_write_].back();

		// TODO: Attempt to avoid this copy operation
		c.spkt = spkt;
		c.pkt = pkt;
    });
}

FileSource::~FileSource() {

}

void FileSource::_processPose(ftl::codecs::Packet &pkt) {
	LOG(INFO) << "Got POSE channel";
	if (pkt.codec == codec_t::POSE) {
		Eigen::Matrix4d p = Eigen::Map<Eigen::Matrix4d>((double*)pkt.data.data());
		host_->setPose(p);
	} else if (pkt.codec == codec_t::MSGPACK) {

	}
}

void FileSource::_processCalibration(ftl::codecs::Packet &pkt) {
	if (pkt.codec == codec_t::CALIBRATION) {
		ftl::rgbd::Camera *camera = (ftl::rgbd::Camera*)pkt.data.data();
		params_ = *camera;
		has_calibration_ = true;
	} else if (pkt.codec == codec_t::MSGPACK) {
		std::tuple<ftl::rgbd::Camera, ftl::codecs::Channel, ftl::rgbd::capability_t> params;
		auto unpacked = msgpack::unpack((const char*)pkt.data.data(), pkt.data.size());
		unpacked.get().convert(params);

		if (std::get<1>(params) == Channel::Left) {
			params_ = std::get<0>(params);
			capabilities_ = std::get<2>(params);
			has_calibration_ = true;

			LOG(INFO) << "Got Calibration channel: " << params_.width << "x" << params_.height;
		} else {
			//params_right_ = std::get<0>(params);
		}
	}
}

void FileSource::_removeChannel(ftl::codecs::Channel channel) {
	int c = 0;
	for (auto i=cache_[cache_write_].begin(); i != cache_[cache_write_].end(); ++i) {
		if ((*i).spkt.channel == channel) {
			++c;
			i = cache_[cache_write_].erase(i);
		}
	}
	DLOG(INFO) << "Skipped " << c << " packets";
}

bool FileSource::capture(int64_t ts) {
	if (realtime_) {
    	timestamp_ = ts;
	} else {
		timestamp_ += ftl::timer::getInterval();
	}
    return true;
}

bool FileSource::retrieve() {
	if (!have_frozen_ && !reader_->read(timestamp_)) {
		cache_write_ = -1;
	}
    return true;
}

void FileSource::swap() {
	if (have_frozen_) return;
	cache_read_ = cache_write_;
	cache_write_ = (cache_write_ == 0) ? 1 : 0;
}

bool FileSource::compute(int n, int b) {
	// Freeze frame requires a copy to be made each time...
	if (have_frozen_) {
		cv::cuda::GpuMat t1, t2;
		if (!rgb_.empty()) rgb_.copyTo(t1);
		if (!depth_.empty()) depth_.copyTo(t2);
		host_->notify(timestamp_, t1, t2);
		return true;
	}

	if (cache_read_ < 0) return false;
	if (cache_[cache_read_].size() == 0) return false;

	int64_t lastts = 0;
	int lastc = 0;

	// Go through previously read and cached frames in sequence
	// needs to be done due to P-Frames
	for (auto i=cache_[cache_read_].begin(); i!=cache_[cache_read_].end(); ++i) {
		auto &c = *i;

		// Check for verifying that both channels are received, ie. two frames
		// with the same timestamp.
		if (c.spkt.timestamp > lastts) {
			lastts = c.spkt.timestamp;
			lastc = 1;
		} else if (c.spkt.timestamp == lastts) {
			lastc++;
		}

		if (c.spkt.channel == Channel::Colour) {
			rgb_.create(cv::Size(ftl::codecs::getWidth(c.pkt.definition),ftl::codecs::getHeight(c.pkt.definition)), CV_8UC3);
			_createDecoder(0, c.pkt);

			try {
				decoders_[0]->decode(c.pkt, rgb_);
			} catch (std::exception &e) {
				LOG(INFO) << "Decoder exception: " << e.what();
			}
		} else if (host_->getChannel() == c.spkt.channel) {
			depth_.create(cv::Size(ftl::codecs::getWidth(c.pkt.definition),ftl::codecs::getHeight(c.pkt.definition)), CV_32F);
			_createDecoder(1, c.pkt);
			try {
				decoders_[1]->decode(c.pkt, depth_);
			} catch (std::exception &e) {
				LOG(INFO) << "Decoder exception: " << e.what();
			}
		}
	
		//_createDecoder((c.spkt.channel == Channel::Colour) ? 0 : 1, c.pkt);

		/*try {
			decoders_[(c.spkt.channel == Channel::Colour) ? 0 : 1]->decode(c.pkt, (c.spkt.channel == Channel::Colour) ? rgb_ : depth_);
		} catch (std::exception &e) {
			LOG(INFO) << "Decoder exception: " << e.what();
		}*/
	}

	// FIXME: Consider case of Channel::None
	if (lastc < 2) {
		LOG(ERROR) << "Channels not in sync (" << sourceid_ << "): " << lastts;
		return false;
	}

	cache_[cache_read_].clear();

	if (rgb_.empty() || depth_.empty()) return false;

	// Inform about a decoded frame pair
	if (freeze_ && !have_frozen_) {
		have_frozen_ = true;
	} else {
		host_->notify(timestamp_, rgb_, depth_);
	}
    return true;
}

bool FileSource::isReady() {
    return true;
}
