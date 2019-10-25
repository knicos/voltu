#include <ftl/codecs/writer.hpp>
#include <ftl/timer.hpp>
#include <loguru.hpp>

#include <tuple>

using ftl::codecs::Writer;

Writer::Writer(std::ostream &s) : stream_(&s), active_(false) {}

Writer::~Writer() {

}

bool Writer::begin() {
	ftl::codecs::Header h;
	//h.version = 2;
	(*stream_).write((const char*)&h, sizeof(h));

	ftl::codecs::IndexHeader ih;
	ih.reserved[0] = -1;
	(*stream_).write((const char*)&ih, sizeof(ih));

	// Capture current time to adjust timestamps
	timestart_ = ftl::timer::get_time();
	active_ = true;
	return true;
}

bool Writer::end() {
	active_ = false;
	return true;
}

bool Writer::write(const ftl::codecs::StreamPacket &s, const ftl::codecs::Packet &p) {
	if (!active_) return false;
	ftl::codecs::StreamPacket s2 = s;
	// Adjust timestamp relative to start of file.
	s2.timestamp -= timestart_;

	auto data = std::make_tuple(s2,p);
	msgpack::sbuffer buffer;
	msgpack::pack(buffer, data);
	(*stream_).write(buffer.data(), buffer.size());
	//buffer_.clear();
	return true;
}
