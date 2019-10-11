#include "file_source.hpp"

using ftl::rgbd::detail::FileSource;
using ftl::codecs::codec_t;

void FileSource::_createDecoder(int ix, const ftl::codecs::Packet &pkt) {
	if (decoders_[ix]) {
		if (!decoders_[ix]->accepts(pkt)) {
			ftl::codecs::free(decoders_[ix]);
		} else {
			return;
		}
	}

	LOG(INFO) << "Create a decoder: " << ix;
	decoders_[ix] = ftl::codecs::allocateDecoder(pkt);
}

FileSource::FileSource(ftl::rgbd::Source *s, ftl::codecs::Reader *r, int sid) : ftl::rgbd::detail::Source(s) {
    reader_ = r;
	has_calibration_ = false;
	decoders_[0] = nullptr;
	decoders_[1] = nullptr;
	cache_read_ = -1;
	cache_write_ = 0;

    r->onPacket(sid, [this](const ftl::codecs::StreamPacket &spkt, ftl::codecs::Packet &pkt) {
		if (pkt.codec == codec_t::POSE) {
			Eigen::Matrix4d p = Eigen::Map<Eigen::Matrix4d>((double*)pkt.data.data());
			host_->setPose(p);
		} else if (pkt.codec == codec_t::CALIBRATION) {
			ftl::rgbd::Camera *camera = (ftl::rgbd::Camera*)pkt.data.data();
            LOG(INFO) << "Have calibration: " << camera->fx;
			params_ = *camera;
			has_calibration_ = true;
		} else {
			cache_[cache_write_].emplace_back();
			auto &c = cache_[cache_write_].back();

			// TODO: Attempt to avoid this copy operation
			c.spkt = spkt;
			c.pkt = pkt;
		}
    });
}

FileSource::~FileSource() {

}

bool FileSource::capture(int64_t ts) {
    timestamp_ = ts;
    return true;
}

bool FileSource::retrieve() {
	if (!reader_->read(timestamp_)) {
		cache_write_ = -1;
	}
    return true;
}

void FileSource::swap() {
	cache_read_ = cache_write_;
	cache_write_ = (cache_write_ == 0) ? 1 : 0;
}

bool FileSource::compute(int n, int b) {
	if (cache_read_ < 0) return false;

	for (auto i=cache_[cache_read_].begin(); i!=cache_[cache_read_].end(); ++i) {
		auto &c = *i;

		if (c.spkt.channel == 0) {
			rgb_.create(cv::Size(ftl::codecs::getWidth(c.pkt.definition),ftl::codecs::getHeight(c.pkt.definition)), CV_8UC3);
		} else {
			depth_.create(cv::Size(ftl::codecs::getWidth(c.pkt.definition),ftl::codecs::getHeight(c.pkt.definition)), CV_32F);
		}
	
		_createDecoder(c.spkt.channel, c.pkt);

		try {
			decoders_[c.spkt.channel]->decode(c.pkt, (c.spkt.channel == 0) ? rgb_ : depth_);
		} catch (std::exception &e) {
			LOG(INFO) << "Decoder exception: " << e.what();
		}
	}

	cache_[cache_read_].clear();

	if (rgb_.empty() || depth_.empty()) return false;

	auto cb = host_->callback();
	if (cb) cb(timestamp_, rgb_, depth_);
    return true;
}

bool FileSource::isReady() {
    return true;
}
