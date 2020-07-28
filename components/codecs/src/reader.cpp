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

	if (h.version >= 2) {
		ftl::codecs::IndexHeader ih;
		(*stream_).read((char*)&ih, sizeof(ih));
	}

	version_ = h.version;
	LOG(INFO) << "FTL format version " << version_;

	// Capture current time to adjust timestamps
	timestart_ = (ftl::timer::get_time() / ftl::timer::getInterval()) * ftl::timer::getInterval();
	playing_ = true;

	return true;
}

/*static void printMsgpack(msgpack::object &obj) {
	switch (obj.type) {
	case msgpack::type::NIL: return;
	case msgpack::type::BOOLEAN: LOG(INFO) << "BOOL " << obj.as<bool>(); return;
	case msgpack::type::POSITIVE_INTEGER:
	case msgpack::type::NEGATIVE_INTEGER: LOG(INFO) << "INT " << obj.as<int>(); return;
	case msgpack::type::FLOAT32: LOG(INFO) << "FLOAT " << obj.as<float>(); return;
	case msgpack::type::FLOAT64: LOG(INFO) << "DOUBLE " << obj.as<double>(); return;
	case msgpack::type::STR: LOG(INFO) << "STRING " << obj.as<std::string>(); return;
	case msgpack::type::BIN: return;
	case msgpack::type::EXT: return;
	case msgpack::type::ARRAY: LOG(INFO) << "ARRAY: "; return;
	case msgpack::type::MAP: LOG(INFO) << "MAP: "; return;
	}
}*/

bool Reader::read(int64_t ts, const std::function<void(const ftl::codecs::StreamPacket &, ftl::codecs::Packet &)> &f) {
	#ifdef DEBUG_MUTEX
	UNIQUE_LOCK(mtx_, lk);
	#else
	std::unique_lock<std::mutex> lk(mtx_, std::defer_lock);
	if (!lk.try_lock()) return true;
	#endif

	// Check buffer first for frames already read
	for (auto i = data_.begin(); i != data_.end();) {
		if (get<0>(*i).timestamp <= ts) {
			f(get<0>(*i), get<1>(*i));
			i = data_.erase(i);
		} else {
			++i;
		}
	}

	bool partial = false;
	int64_t extended_ts = ts + 200;  // Buffer 200ms ahead

	while ((playing_ && stream_->good()) || buffer_.nonparsed_size() > 0) {
		if (buffer_.nonparsed_size() == 0 || (partial && buffer_.nonparsed_size() < 10000000)) {
			buffer_.reserve_buffer(10000000);
			stream_->read(buffer_.buffer(), buffer_.buffer_capacity());
			//if (stream_->bad()) return false;

			size_t bytes = static_cast<size_t>(stream_->gcount());
			if (bytes == 0) break;
			buffer_.buffer_consumed(bytes);
			partial = false;
		}

		msgpack::object_handle msg;
		if (!buffer_.next(msg)) {
			partial = true;
			continue;
		}

		//std::tuple<StreamPacket,Packet> data;
		msgpack::object obj = msg.get();

		//printMsgpack(obj);

		try {
			obj.convert(data_.emplace_back());
		} catch (std::exception &e) {
			LOG(INFO) << "Corrupt message: " << buffer_.nonparsed_size() << " - " << e.what();
			//partial = true;
			//continue;
			return false;
		}

		auto &data = data_.back();

		// Adjust timestamp
		get<0>(data).timestamp += timestart_;

		// Fix to clear flags for version 2.
		if (version_ <= 2) {
			get<1>(data).flags = 0;
		}

		// TODO: Need to read ahead a few frames because there may be a
		// smaller timestamp after this one... requires a buffer. Ideally this
		// should be resolved during the write process.
		if (get<0>(data).timestamp <= ts) {
			f(get<0>(data),get<1>(data));
			data_.pop_back();
		} else if (get<0>(data).timestamp > extended_ts) {
			//data_ = data;
			//has_data_ = true;
			return true;
		}
	}

	return data_.size() > 0;
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
	if (static_cast<unsigned int>(streamID) >= handlers_.size()) handlers_.resize(streamID+1);
	handlers_[streamID] = f;
}

bool Reader::end() {
	playing_ = false;
	return true;
}
