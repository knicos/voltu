#include <fstream>
#include <ftl/streams/filestream.hpp>
#include <ftl/timer.hpp>

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

	on("save_data", [this]() {
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

	on("save_data", [this]() {
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
	int count = 1000;
	int64_t ts = -1000;
	int min_ts_diff = 1000;
	first_ts_ = 10000000000000ll;

	std::unordered_set<ftl::codecs::codec_t> codecs_found;

	while (count > 0) {
		std::tuple<ftl::codecs::StreamPacket,ftl::codecs::Packet> data;
		if (!readPacket(data)) {
			break;
		}

		auto &spkt = std::get<0>(data);
		auto &pkt = std::get<1>(data);

		auto &fsdata = framesets_[spkt.streamID];

		codecs_found.emplace(pkt.codec);

		if (fsdata.first_ts < 0) fsdata.first_ts = spkt.timestamp;

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
	if (!is_video_) {
		LOG(INFO) << " -- Static image";
		set("looping", false);
	}

	std::string codec_str = "";
	for (auto c : codecs_found) {
		codec_str += std::string(" ") + std::to_string(int(c));
	}
	LOG(INFO) << " -- Codecs:" << codec_str;

	interval_ = min_ts_diff;
	for (auto &f : framesets_) {
		f.second.interval = interval_;
	}
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
			// Older versions have a different SPKT structure.
			if (version_ < 5) {
				std::tuple<ftl::codecs::StreamPacketV4, ftl::codecs::Packet> datav4;
				obj.convert(datav4);

				auto &spkt = std::get<0>(data);
				auto &spktv4 = std::get<0>(datav4);
				spkt.version = 4;
				spkt.streamID = spktv4.streamID;
				spkt.channel = spktv4.channel;
				spkt.frame_number = spktv4.frame_number;
				spkt.timestamp = spktv4.timestamp;
				spkt.flags = 0;

				std::get<1>(data) = std::move(std::get<1>(datav4));
			} else {
				obj.convert(data);
			}
		} catch (std::exception &e) {
			LOG(INFO) << "Corrupt message: " << buffer_in_.nonparsed_size() << " - " << e.what();
			//active_ = false;
			return false;
		}

		// Correct for older version differences.
		_patchPackets(std::get<0>(data), std::get<1>(data));

		return true;
	}

	return false;
}

void File::_patchPackets(ftl::codecs::StreamPacket &spkt, ftl::codecs::Packet &pkt) {
	// Fix to clear flags for version 2.
	if (version_ <= 2) {
		pkt.flags = 0;
	}
	if (version_ < 4) {
		spkt.frame_number = spkt.streamID;
		spkt.streamID = 0;
		if (isFloatChannel(spkt.channel)) pkt.flags |= ftl::codecs::kFlagFloat;

		auto codec = pkt.codec;
		if (codec == ftl::codecs::codec_t::HEVC) pkt.codec = ftl::codecs::codec_t::HEVC_LOSSLESS;
	}

	spkt.version = ftl::codecs::kCurrentFTLVersion;

	// Fix for flags corruption
	if (pkt.data.size() == 0) {
		pkt.flags = 0;
	}
}

bool File::tick(int64_t ts) {
	if (!active_) return false;
	if (mode_ != Mode::Read) {
		LOG(ERROR) << "Cannot read from a write only file";
		return false;
	}

	// Skip if paused
	if (value("paused", false)) return true;

	#ifdef DEBUG_MUTEX
	UNIQUE_LOCK(mutex_, lk);
	#else
	std::unique_lock<std::mutex> lk(mutex_, std::defer_lock);
	if (!lk.try_lock()) return true;
	#endif

	if (jobs_ > 0) {
		return true;
	}

	bool has_data = false;

	// Check buffer first for frames already read
	{
		//UNIQUE_LOCK(data_mutex_, dlk);
		if (data_.size() > 0) has_data = true;
		
		/*if (needs_endframe_) {
			// Reset packet counts
			for (auto &p : packet_counts_) p = 0;
		}*/

		size_t complete_count = 0;

		for (auto i = data_.begin(); i != data_.end(); ) {
			auto &fsdata = framesets_[std::get<0>(*i).streamID];
			if (fsdata.timestamp == 0) fsdata.timestamp = std::get<0>(*i).timestamp;

			// Limit to file framerate
			if (std::get<0>(*i).timestamp > ts) {
				break;
			}

			if (std::get<0>(*i).timestamp < fsdata.timestamp) {
				LOG(WARNING) << "Received old packet: " << std::get<0>(*i).timestamp << " vs " << fsdata.timestamp << " ( channel = " << int(std::get<0>(*i).channel) << " )";
				i = data_.erase(i);
				continue;
			}

			if (std::get<0>(*i).timestamp <= fsdata.timestamp) {
				auto &spkt = std::get<0>(*i);
				auto &pkt = std::get<1>(*i);

				//LOG(INFO) << "PACKET: " << spkt.timestamp << ", " << fsdata.timestamp << ", " << int(spkt.streamID) << ", " << int(spkt.channel);


				++jobs_;
				//spkt.timestamp = ts;

				if (spkt.channel == Channel::EndFrame) {
					fsdata.needs_endframe = false;
				}

				if (fsdata.needs_endframe) {
					if (spkt.frame_number < 255) {
						fsdata.frame_count = std::max(fsdata.frame_count, static_cast<size_t>(spkt.frame_number + pkt.frame_count));
						while (fsdata.packet_counts.size() < fsdata.frame_count) fsdata.packet_counts.push_back(0);
						++fsdata.packet_counts[spkt.frame_number];
					} else {
						// Add frameset packets to frame 0 counts
						fsdata.frame_count = std::max(fsdata.frame_count, size_t(1));
						while (fsdata.packet_counts.size() < fsdata.frame_count) fsdata.packet_counts.push_back(0);
						++fsdata.packet_counts[0];
					}
				}

				auto j = i;
				++i;

				ftl::pool.push([this,i=j](int id) {
					auto &spkt = std::get<0>(*i);
					auto &pkt = std::get<1>(*i);

					spkt.localTimestamp = spkt.timestamp;

					try {
						cb_.trigger(spkt, pkt);
					} catch (const ftl::exception &e) {
						LOG(ERROR) << "Exception in packet callback: " << e.what() << e.trace();
					} catch (std::exception &e) {
						LOG(ERROR) << "Exception in packet callback: " << e.what();
					}
					//LOG(INFO) << "ERASE: " << spkt.timestamp << ", " << spkt.frameNumber() << ", " << (int)spkt.channel;
					UNIQUE_LOCK(data_mutex_, dlk);
					data_.erase(i);
					--jobs_;
				});
			} else {
				++complete_count;

				if (fsdata.needs_endframe) {
					// Send final frame packet.
					StreamPacket spkt;
					spkt.timestamp = fsdata.timestamp;
					spkt.streamID = std::get<0>(*i).streamID;
					spkt.flags = 0;
					spkt.channel = Channel::EndFrame;

					//LOG(INFO) << "Send EndFrame: " << spkt.timestamp << ", " << int(spkt.streamID);

					Packet pkt;
					pkt.bitrate = 255;
					pkt.codec = ftl::codecs::codec_t::Invalid;
					pkt.packet_count = 1;
					pkt.frame_count = 1;

					for (size_t i=0; i<fsdata.frame_count; ++i) {
						spkt.frame_number = i;
						pkt.packet_count = fsdata.packet_counts[i]+1;
						fsdata.packet_counts[i] = 0;

						try {
							cb_.trigger(spkt, pkt);
						} catch (const ftl::exception &e) {
							LOG(ERROR) << "Exception in packet callback: " << e.what() << e.trace();
						} catch (std::exception &e) {
							LOG(ERROR) << "Exception in packet callback: " << e.what();
						}
					}
				} else {
				}

				fsdata.timestamp = std::get<0>(*i).timestamp; //fsdata.interval;
				if (complete_count == framesets_.size()) break;
			}
		}
	}

	int64_t max_ts = std::numeric_limits<int64_t>::min();
	for (auto &fsd : framesets_) max_ts = std::max(max_ts, (fsd.second.timestamp <= 0) ? timestart_ : fsd.second.timestamp);
	int64_t extended_ts = max_ts + 200;  // Buffer 200ms ahead

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

		auto &fsdata = framesets_[std::get<0>(data).streamID];

		if (fsdata.first_ts < 0) LOG(WARNING) << "Bad first timestamp " << fsdata.first_ts << ", " << std::get<0>(data).timestamp;

		// Adjust timestamp
		// FIXME: A potential bug where multiple times are merged into one?
		std::get<0>(data).timestamp = (((std::get<0>(data).timestamp) - fsdata.first_ts)) + timestart_;
		std::get<0>(data).hint_capability = ((is_video_) ? 0 : ftl::codecs::kStreamCap_Static) | ftl::codecs::kStreamCap_Recorded;

		// Maintain availability of channels.
		available(0) += std::get<0>(data).channel;

		// This should only occur for first few frames, generally otherwise
		// the data buffer is already several frames ahead so is processed
		// above. Hence, no need to bother parallelising this bit.
		/*if (!is_video_ && std::get<0>(data).timestamp <= timestamp_) {
			std::get<0>(data).timestamp = ts;
			//if (cb_) {
				dlk.lock();
				try {
					LOG(INFO) << "EARLY TRIGGER: " << std::get<0>(data).timestamp << " - " << int(std::get<0>(data).channel);
					cb_.trigger(std::get<0>(data),std::get<1>(data));
				} catch (std::exception &e) {
					LOG(ERROR) << "Exception in packet callback: " << e.what();
				}
				data_.pop_back();
			//}
		}*/
		//if (version_ < 5 && lastData) {
			// For versions < 5, add completed flag to previous data
		//	std::get<0>(*lastData).flags |= ftl::codecs::kFlagCompleted;
		//}

		if (std::get<0>(data).timestamp > extended_ts) {
			break;
		}
	}

	//if (has_data) {
	//	for (auto &fsd : framesets_) fsd.second.timestamp += interval_;
	//}

	// Force send end frames for static files
	if (data_.size() == 0 && !is_video_) {
		for (auto &fsix : framesets_) {
			auto &fsdata = fsix.second;
			if (fsdata.needs_endframe) {
				fsdata.needs_endframe = false;
				// Send final frame packet.
				StreamPacket spkt;
				spkt.timestamp = fsdata.timestamp;
				spkt.streamID = fsix.first;
				spkt.flags = 0;
				spkt.channel = Channel::EndFrame;

				//LOG(INFO) << "Send EndFrame: " << spkt.timestamp << ", " << int(spkt.streamID);

				Packet pkt;
				pkt.bitrate = 255;
				pkt.codec = ftl::codecs::codec_t::Invalid;
				pkt.packet_count = 1;
				pkt.frame_count = 1;

				for (size_t i=0; i<fsdata.frame_count; ++i) {
					spkt.frame_number = i;
					pkt.packet_count = fsdata.packet_counts[i]+1;
					fsdata.packet_counts[i] = 0;

					try {
						cb_.trigger(spkt, pkt);
					} catch (const ftl::exception &e) {
						LOG(ERROR) << "Exception in packet callback: " << e.what() << e.trace();
					} catch (std::exception &e) {
						LOG(ERROR) << "Exception in packet callback: " << e.what();
					}
				}
			}
		}
	}

	if (data_.size() == 0 && value("looping", true)) {
		buffer_in_.reset();
		buffer_in_.remove_nonparsed_buffer();
		_open();

		timestart_ = ftl::timer::get_time(); // (ftl::timer::get_time() / ftl::timer::getInterval()) * ftl::timer::getInterval();
		//timestamp_ = timestart_;
		for (auto &fsd : framesets_) fsd.second.timestamp = 0;
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
	if (active_) return true;
	if (mode_ == Mode::Read) {
		if (!checked_) _checkFile();
		_open();

		// Capture current time to adjust timestamps
		timestart_ = ftl::timer::get_time(); //(ftl::timer::get_time() / ftl::timer::getInterval()) * ftl::timer::getInterval();
		active_ = true;
		//interval_ = 40;
		//timestamp_ = timestart_;
		//for (auto &fsd : framesets_) fsd.second.timestamp = timestart_;

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
		//timestamp_ = timestart_;

		//for (auto &fsd : framesets_) fsd.second.timestamp = timestart_;
	}

	return true;
}

bool File::end() {
	if (!active_) return false;
	active_ = false;
	
	timer_.cancel();

	UNIQUE_LOCK(mutex_, lk);

	while (jobs_ > 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

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
	/*UNIQUE_LOCK(mutex_, lk);

	// TODO: Find a better solution
	while (jobs_ > 0) std::this_thread::sleep_for(std::chrono::milliseconds(2));

	data_.clear();
	buffer_in_.reset();
	buffer_in_.remove_nonparsed_buffer();
	_open();

	timestart_ = (ftl::timer::get_time() / ftl::timer::getInterval()) * ftl::timer::getInterval();
	//timestamp_ = timestart_;
	for (auto &fsd : framesets_) fsd.second.timestamp = timestart_;*/
}

bool File::active() {
	return active_;
}
