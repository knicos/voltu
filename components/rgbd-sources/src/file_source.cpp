#include "file_source.hpp"

using ftl::rgbd::detail::FileSource;

FileSource::FileSource(ftl::rgbd::Source *s, ftl::codecs::Reader *r, int sid) : ftl::rgbd::detail::Source(s) {
    reader_ = r;
    r->onPacket(sid, [this](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
        LOG(INFO) << "PACKET RECEIVED " << spkt.streamID;
    });
}

FileSource::~FileSource() {

}

bool FileSource::capture(int64_t ts) {
    reader_->read(ts);
    return true;
}

bool FileSource::retrieve() {
    return true;
}

bool FileSource::compute(int n, int b) {
    return true;
}

bool FileSource::isReady() {
    return true;
}
