#include <loguru.hpp>
#include <ftl/codecs/reader.hpp>
#include <ftl/timer.hpp>

#include <tuple>

using ftl::codecs::Reader;
using ftl::codecs::StreamPacket;
using ftl::codecs::Packet;
using std::get;

Reader::Reader(std::istream &s) : stream_(&s), has_data_(false), playing_(false) {

}

Reader::~Reader() {

}

bool Reader::begin() {
	ftl::codecs::Header h;
	(*stream_).read((char*)&h, sizeof(h));
	if (h.magic[0] != 'F' || h.magic[1] != 'T' || h.magic[2] != 'L' || h.magic[3] != 'F') return false;

	// Capture current time to adjust timestamps
	timestart_ = (ftl::timer::get_time() / ftl::timer::getInterval()) * ftl::timer::getInterval();
	playing_ = true;

	return true;
}

bool Reader::read(int64_t ts, const std::function<void(const ftl::codecs::StreamPacket &, ftl::codecs::Packet &)> &f) {
	//UNIQUE_LOCK(mtx_, lk);
	std::unique_lock<std::mutex> lk(mtx_, std::defer_lock);
	if (!lk.try_lock()) return true;

	if (has_data_ && get<0>(data_).timestamp <= ts) {
		f(get<0>(data_), get<1>(data_));
		has_data_ = false;
	} else if (has_data_) {
		return false;
	}

	bool partial = false;

	while (playing_ && stream_->good() || buffer_.nonparsed_size() > 0) {
		if (buffer_.nonparsed_size() == 0 || (partial && buffer_.nonparsed_size() < 10000000)) {
			buffer_.reserve_buffer(10000000);
			stream_->read(buffer_.buffer(), buffer_.buffer_capacity());
			//if (stream_->bad()) return false;

			int bytes = stream_->gcount();
			if (bytes == 0) return false;
			buffer_.buffer_consumed(bytes);
			partial = false;
		}

		msgpack::object_handle msg;
		if (!buffer_.next(msg)) {
			partial = true;
			continue;
		}

		std::tuple<StreamPacket,Packet> data;
		msgpack::object obj = msg.get();
		try {
			obj.convert(data);
		} catch (std::exception &e) {
			LOG(INFO) << "Corrupt message: " << buffer_.nonparsed_size();
			//partial = true;
			//continue;
			return false;
		}

		// Adjust timestamp
		get<0>(data).timestamp += timestart_;

		if (get<0>(data).timestamp <= ts) {
			f(get<0>(data),get<1>(data));
		} else {
			data_ = data;
			has_data_ = true;
			return true;
		}
	}

	return false;
}

bool Reader::read(int64_t ts) {
	return read(ts, [this](const ftl::codecs::StreamPacket &spkt, ftl::codecs::Packet &pkt) {
		if (handlers_.size() > spkt.streamID && (bool)handlers_[spkt.streamID]) {
			handlers_[spkt.streamID](spkt, pkt);
		} else if (spkt.streamID == 255) {
			// Broadcast stream, send packets to every source handler.
			for (auto &h : handlers_) {
				h(spkt, pkt);
			}
		}
	});
}

void Reader::onPacket(int streamID, const std::function<void(const ftl::codecs::StreamPacket &, ftl::codecs::Packet &)> &f) {
	if (streamID >= handlers_.size()) handlers_.resize(streamID+1);
	handlers_[streamID] = f;
}

bool Reader::end() {
	playing_ = false;
	return true;
}
