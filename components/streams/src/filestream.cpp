#include <fstream>
#include <ftl/streams/filestream.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::stream::File;
using ftl::codecs::StreamPacket;
using ftl::codecs::Packet;
using std::get;
using ftl::codecs::Channel;

File::File(nlohmann::json &config) : Stream(config), ostream_(nullptr), istream_(nullptr), active_(false) {
	mode_ = Mode::Read;
	jobs_ = 0;
	checked_ = false;
	save_data_ = value("save_data", false);

	on("save_data", [this](const ftl::config::Event &e) {
		save_data_ = value("save_data", false);
	});
}

File::File(nlohmann::json &config, std::ifstream *is) : Stream(config), ostream_(nullptr), istream_(is), active_(false) {
	mode_ = Mode::Read;
	jobs_ = 0;
	checked_ = false;
	save_data_ = false;
}

File::File(nlohmann::json &config, std::ofstream *os) : Stream(config), ostream_(os), istream_(nullptr), active_(false) {
	mode_ = Mode::Write;
	jobs_ = 0;
	checked_ = false;
	save_data_ = value("save_data", false);

	on("save_data", [this](const ftl::config::Event &e) {
		save_data_ = value("save_data", false);
	});
}

File::~File() {
	end();
}

bool File::_checkFile() {
	if (!_open()) return false;

	LOG(INFO) << "FTL format version " << version_;

	// Read some packets to identify frame rate.
	int count = 10;
	int64_t ts = -1000;
	int min_ts_diff = 1000;
	first_ts_ = 10000000000000ll;

	while (count > 0) {
		std::tuple<ftl::codecs::StreamPacket,ftl::codecs::Packet> data;
		if (!readPacket(data)) {
			break;
		}

		auto &spkt = std::get<0>(data);
		//auto &pkt = std::get<1>(data);

		if (spkt.timestamp < first_ts_) first_ts_ = spkt.timestamp;

		//LOG(INFO) << "TIMESTAMP: " << spkt.timestamp;

		if (spkt.timestamp > 0 && int(spkt.channel) < 32) {
			if (spkt.timestamp > ts) {
				--count;
				auto d = spkt.timestamp - ts;
				if (d < min_ts_diff && d > 0) {
					min_ts_diff = d;
				}
				ts = spkt.timestamp;
			}
		}
	}

	buffer_in_.reset();
	buffer_in_.remove_nonparsed_buffer();

	checked_ = true;

	is_video_ = count < 9;

	LOG(INFO) << " -- Frame rate = " << (1000 / min_ts_diff);
	if (!is_video_) LOG(INFO) << " -- Static image";
	interval_ = min_ts_diff;
	return true;
}

bool File::onPacket(const std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> &f) {
	cb_ = f;
	return true;
}

bool File::post(const ftl::codecs::StreamPacket &s, const ftl::codecs::Packet &p) {
	if (!active_) return false;
	if (mode_ != Mode::Write) {
		LOG(WARNING) << "Cannot write to read-only ftl file";
		return false;
	}

	//LOG(INFO) << "WRITE: " << s.timestamp << " " << (int)s.channel << " " << p.data.size();

	// Don't write dummy packets to files.
	if (p.data.size() == 0) return true;

	available(s.streamID) += s.channel;

	// Discard all data channel packets for now
	// TODO: Allow saving of data channels once formats have solidified.
	if (!save_data_ && static_cast<int>(s.channel) >= static_cast<int>(ftl::codecs::Channel::Data)) return true;

	ftl::codecs::StreamPacket s2 = s;
	// Adjust timestamp relative to start of file.
	//s2.timestamp -= timestart_;

	auto data = std::make_tuple(s2,p);
	msgpack::sbuffer buffer;
	msgpack::pack(buffer, data);

	UNIQUE_LOCK(mutex_, lk);
	ostream_->write(buffer.data(), buffer.size());
	return ostream_->good();
}

bool File::readPacket(std::tuple<ftl::codecs::StreamPacket,ftl::codecs::Packet> &data) {
	bool partial = false;

	while ((istream_->good()) || buffer_in_.nonparsed_size() > 0u) {
		if (buffer_in_.nonparsed_size() == 0 || (partial && buffer_in_.nonparsed_size() < 10000000)) {
			buffer_in_.reserve_buffer(10000000);
			istream_->read(buffer_in_.buffer(), buffer_in_.buffer_capacity());
			//if (stream_->bad()) return false;

			int bytes = istream_->gcount();
			if (bytes == 0) return false;
			buffer_in_.buffer_consumed(bytes);
			partial = false;
		}

		msgpack::object_handle msg;
		if (!buffer_in_.next(msg)) {
			partial = true;
			continue;
		}

		msgpack::object obj = msg.get();

		try {
			obj.convert(data);
		} catch (std::exception &e) {
			LOG(INFO) << "Corrupt message: " << buffer_in_.nonparsed_size() << " - " << e.what();
			//active_ = false;
			return false;
		}

		// Fix to clear flags for version 2.
		if (version_ <= 2) {
			std::get<1>(data).flags = 0;
		}
		if (version_ < 4) {
			std::get<0>(data).frame_number = std::get<0>(data).streamID;
			std::get<0>(data).streamID = 0;
			if (isFloatChannel(std::get<0>(data).channel)) std::get<1>(data).flags |= ftl::codecs::kFlagFloat;

			auto codec = std::get<1>(data).codec;
			if (codec == ftl::codecs::codec_t::HEVC) std::get<1>(data).codec = ftl::codecs::codec_t::HEVC_LOSSLESS;
		}
		std::get<0>(data).version = 4;

		return true;
	}

	return false;
}

bool File::tick(int64_t ts) {
	if (!active_) return false;
	if (mode_ != Mode::Read) {
		LOG(ERROR) << "Cannot read from a write only file";
		return false;
	}

	#ifdef DEBUG_MUTEX
	UNIQUE_LOCK(mutex_, lk);
	#else
	std::unique_lock<std::mutex> lk(mutex_, std::defer_lock);
	if (!lk.try_lock()) return true;
	#endif

	if (jobs_ > 0) {
		//LOG(ERROR) << "STILL HAS JOBS";
		return true;
	}

	// Check buffer first for frames already read
	{
		UNIQUE_LOCK(data_mutex_, dlk);
		for (auto i = data_.begin(); i != data_.end(); ++i) {
			if (std::get<0>(*i).timestamp <= timestamp_) {
				++jobs_;
				std::get<0>(*i).timestamp = ts;
				ftl::pool.push([this,i](int id) {
					auto &spkt = std::get<0>(*i);
					auto &pkt = std::get<1>(*i);

					try {
						if (cb_) cb_(spkt, pkt);
					} catch (std::exception &e) {
						LOG(ERROR) << "Exception in packet callback: " << e.what();
					}
					//LOG(INFO) << "ERASE: " << spkt.timestamp << ", " << spkt.frameNumber() << ", " << (int)spkt.channel;
					UNIQUE_LOCK(data_mutex_, dlk);
					data_.erase(i);
					--jobs_;
				});
			}
		}
	}

	int64_t extended_ts = timestamp_ + 200;  // Buffer 200ms ahead

	while ((active_ && istream_->good()) || buffer_in_.nonparsed_size() > 0u) {
		UNIQUE_LOCK(data_mutex_, dlk);
		auto &data = data_.emplace_back();
		dlk.unlock();

		bool res = readPacket(data);
		if (!res) {
			UNIQUE_LOCK(data_mutex_, dlk);
			data_.pop_back();
			break;
		}

		// Adjust timestamp
		// FIXME: A potential bug where multiple times are merged into one?
		std::get<0>(data).timestamp = (((std::get<0>(data).timestamp) - first_ts_) / interval_) * interval_ + timestart_;
		std::get<0>(data).hint_capability = (is_video_) ? 0 : ftl::codecs::kStreamCap_Static;

		// Maintain availability of channels.
		available(0) += std::get<0>(data).channel;

		// This should only occur for first few frames, generally otherwise
		// the data buffer is already several frames ahead so is processed
		// above. Hence, no need to bother parallelising this bit.
		if (std::get<0>(data).timestamp <= timestamp_) {
			std::get<0>(data).timestamp = ts;
			if (cb_) {
				dlk.lock();
				try {
					cb_(std::get<0>(data),std::get<1>(data));
				} catch (std::exception &e) {
					LOG(ERROR) << "Exception in packet callback: " << e.what();
				}
				data_.pop_back();
			}
		} else if (std::get<0>(data).timestamp > extended_ts) {
			break;
		}
	}

	timestamp_ += interval_;

	if (data_.size() == 0 && value("looping", true)) {
		buffer_in_.reset();
		buffer_in_.remove_nonparsed_buffer();
		_open();

		timestart_ = (ftl::timer::get_time() / ftl::timer::getInterval()) * ftl::timer::getInterval();
		timestamp_ = timestart_;
		return true;
	}

	return data_.size() > 0;
}

bool File::_open() {
	if (istream_ && istream_->is_open()) {
		istream_->clear();
        istream_->seekg(0);
	} else {
		if (!istream_) istream_ = new std::ifstream;
		istream_->open(*get<std::string>("filename"), std::ifstream::in | std::ifstream::binary);

		if (!istream_->good()) {
			LOG(ERROR) << "Could not open file: " << *get<std::string>("filename");
			return false;
		}
	}

	ftl::codecs::Header h;
	(*istream_).read((char*)&h, sizeof(h));
	if (h.magic[0] != 'F' || h.magic[1] != 'T' || h.magic[2] != 'L' || h.magic[3] != 'F') return false;

	if (h.version >= 2) {
		ftl::codecs::IndexHeader ih;
		(*istream_).read((char*)&ih, sizeof(ih));
	}

	version_ = h.version;
	return true;
}

bool File::run() {
	timer_ = ftl::timer::add(ftl::timer::kTimerMain, [this](int64_t ts) {
		tick(ts);
		return active_;
	});
	return true;
}

bool File::begin(bool dorun) {
	if (mode_ == Mode::Read) {
		if (!checked_) _checkFile();
		_open();

		// Capture current time to adjust timestamps
		timestart_ = (ftl::timer::get_time() / ftl::timer::getInterval()) * ftl::timer::getInterval();
		active_ = true;
		//interval_ = 40;
		timestamp_ = timestart_;

		tick(timestart_); // Do some now!
		if (dorun) run();
	} else if (mode_ == Mode::Write) {
		if (!ostream_) ostream_ = new std::ofstream;
		ostream_->open(*get<std::string>("filename"), std::ifstream::out | std::ifstream::binary);

		if (!ostream_->good()) {
			LOG(ERROR) << "Could not open file: '" << *get<std::string>("filename") << "'";
			return false;
		}

		ftl::codecs::Header h;
		//h.version = 2;
		(*ostream_).write((const char*)&h, sizeof(h));

		ftl::codecs::IndexHeader ih;
		ih.reserved[0] = -1;
		(*ostream_).write((const char*)&ih, sizeof(ih));

		// Capture current time to adjust timestamps
		timestart_ = ftl::timer::get_time();
		active_ = true;
		interval_ = ftl::timer::getInterval();
		timestamp_ = timestart_;
	}

	return true;
}

bool File::end() {
	UNIQUE_LOCK(mutex_, lk);
	if (!active_) return false;
	active_ = false;
	timer_.cancel();

	if (mode_ == Mode::Read) {
		if (istream_) {
			istream_->close();
			delete istream_;
			istream_ = nullptr;
		}
	} else if (mode_ == Mode::Write) {
		if (ostream_) {
			ostream_->close();
			delete ostream_;
			ostream_ = nullptr;
		}
	}
	return true;
}

void File::reset() {
	UNIQUE_LOCK(mutex_, lk);

	// TODO: Find a better solution
	while (jobs_ > 0) std::this_thread::sleep_for(std::chrono::milliseconds(2));

	data_.clear();
	buffer_in_.reset();
	buffer_in_.remove_nonparsed_buffer();
	_open();

	timestart_ = (ftl::timer::get_time() / ftl::timer::getInterval()) * ftl::timer::getInterval();
	timestamp_ = timestart_;
}

bool File::active() {
	return active_;
}
