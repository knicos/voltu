#include <ftl/codecs/writer.hpp>
#include <ftl/timer.hpp>

#include <tuple>

using ftl::codecs::Writer;

Writer::Writer(std::ostream &s) : stream_(&s) {}

Writer::~Writer() {

}

bool Writer::begin() {
	ftl::codecs::Header h;
	h.version = 0;
	(*stream_).write((const char*)&h, sizeof(h));

	// Capture current time to adjust timestamps
	timestart_ = ftl::timer::get_time();

	return true;
}

bool Writer::end() {
	return true;
}

bool Writer::write(const ftl::codecs::StreamPacket &s, const ftl::codecs::Packet &p) {
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
