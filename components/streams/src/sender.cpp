#include <ftl/streams/sender.hpp>
#include <ftl/codecs/depth_convert_cuda.hpp>
#include <ftl/profiler.hpp>
#include <ftl/audio/software_encoder.hpp>

#include <opencv2/cudaimgproc.hpp>

#include <ftl/streams/injectors.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::stream::Sender;
using ftl::codecs::StreamPacket;
using ftl::codecs::Packet;
using ftl::codecs::Channels;
using ftl::codecs::Channel;
using ftl::codecs::definition_t;
using ftl::codecs::device_t;
using ftl::codecs::codec_t;
using ftl::codecs::format_t;
using ftl::stream::injectCalibration;
using ftl::stream::injectPose;
using ftl::stream::injectConfig;

Sender::Sender(nlohmann::json &config) : ftl::Configurable(config), stream_(nullptr) {
	do_inject_.test_and_set();
	do_reinject_.test_and_set();
	iframe_ = 1;
	add_iframes_ = value("iframes", 50);
	timestamp_ = -1;

	on("iframes", [this]() {
		add_iframes_ = value("iframes", 50);
	});

	on("bitrate_timeout", bitrate_timeout_, 10000);
}

Sender::~Sender() {
	// Delete all encoders
	for (auto c : state_) {
		if (c.second.encoder[0]) ftl::codecs::free(c.second.encoder[0]);
		if (c.second.encoder[1]) ftl::codecs::free(c.second.encoder[1]);
	}
}

void Sender::setStream(ftl::stream::Stream*s) {
	//if (stream_) stream_->onPacket(nullptr);
	stream_ = s;
	handle_ = stream_->onPacket([this](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		if (pkt.data.size() > 0 || !(spkt.flags & ftl::codecs::kFlagRequest)) return true;

		if (int(spkt.channel) < 32) {
			auto now = ftl::timer::get_time();

			// Update the min bitrate selection
			UNIQUE_LOCK(bitrate_mtx_, lk);
			if (bitrate_map_.size() > 0) {
				while (bitrate_map_.size() > 0 && (now - bitrate_map_.begin()->second.first > bitrate_timeout_ || (bitrate_map_.begin()->second.second == spkt.hint_peerid && pkt.bitrate != bitrate_map_.begin()->first))) {
					LOG(INFO) << "Remove bitrate " << int(bitrate_map_.begin()->first);
					bitrate_map_.erase(bitrate_map_.begin());
				}
			}
			bitrate_map_[pkt.bitrate] = std::make_pair(now, spkt.hint_peerid);
		}

		//if (state_cb_) state_cb_(spkt.channel, spkt.streamID, spkt.frame_number);
		if (reqcb_) reqcb_(spkt,pkt);

		// Inject state packets
		if ((spkt.hint_capability & ftl::codecs::kStreamCap_NewConnection) || (spkt.flags & ftl::codecs::kFlagReset)) do_inject_.clear();

		return true;
	});
}

uint8_t Sender::_getMinBitrate() {
	SHARED_LOCK(bitrate_mtx_, lk);
	if (bitrate_map_.size() > 0) return bitrate_map_.begin()->first;
	else return 255;
}

void Sender::onRequest(const ftl::stream::StreamCallback &cb) {
	reqcb_ = cb;
}

ftl::audio::Encoder *Sender::_getAudioEncoder(int fsid, int sid, ftl::codecs::Channel c, ftl::codecs::Packet &pkt) {
	int id = (fsid << 8) + sid;
	auto i = audio_state_.find(id);
	if (i == audio_state_.end()) {
		audio_state_[id] = {nullptr};
	}

	auto &state = audio_state_[id];
	if (state.encoder == nullptr) {
		state.encoder = new ftl::audio::SoftwareEncoder();
	}
	return state.encoder;
}

template <typename T>
static void writeValue(std::vector<unsigned char> &data, T value) {
	unsigned char *pvalue_start = (unsigned char*)&value;
	data.insert(data.end(), pvalue_start, pvalue_start+sizeof(T));
}

void Sender::_sendPersistent(ftl::rgbd::FrameSet &fs) {
	std::unordered_set<ftl::codecs::Channel> persist_chan;

	for (auto &frame : fs.frames) {
		auto *session = frame.parent();
		if (session) {
			auto chans = session->channels();
			persist_chan.insert(chans.begin(), chans.end());
		}
	}

	for (auto c : persist_chan) {
		post(fs, c);
	}
}

void Sender::fakePost(ftl::data::FrameSet &fs, ftl::codecs::Channel c) {
	if (!stream_) return;

	for (size_t i=0; i<fs.frames.size(); ++i) {
		auto &frame = fs.frames[i];
		if (frame.hasOwn(c)) ++frame.packet_tx;
		
	}
}

bool Sender::_checkNeedsIFrame(int64_t ts, bool injecting) {
	int mspf = ftl::timer::getInterval();

	if (injecting) injection_timestamp_ = ts+2*mspf;

	// Add an iframe at the requested frequency.
	//if (add_iframes_ > 0 && ts != timestamp_) iframe_ = (iframe_+1) % add_iframes_;
	//if (iframe_ == 0) injection_timestamp_ = ts+mspf;

	// FIXME: This may need to be atomic in some cases?
	//if (ts > timestamp_) timestamp_ = ts;
	return injection_timestamp_ >= ts;
}

void Sender::_send(ftl::rgbd::FrameSet &fs, ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
	/*int ccount = 0;
	for (size_t i=0; i<fs.frames.size(); ++i) ccount += fs.frames[i].changed().size();
	fs.flush_count += fs.frames.size();

	if (ccount == fs.flush_count) {
		spkt.flags = ftl::codecs::kFlagCompleted;
	}*/

	if (spkt.frame_number == 255) ++fs.frames[0].packet_tx;
	else if (spkt.frame_number < fs.frames.size()) ++fs.frames[spkt.frame_number].packet_tx;
	stream_->post(spkt, pkt);
}

void Sender::post(ftl::data::FrameSet &fs, ftl::codecs::Channel c, bool noencode) {
	if (!stream_) return;

	// Send quick message for this special channel.
	if (c == Channel::EndFrame) {
		if (timestamp_ >= fs.timestamp()) {
			LOG(WARNING) << "Sending old frame! " << fs.timestamp() << " vs " << timestamp_ << " (size = " << fs.frames[0].packet_tx+1 << ")"; 
		}
		timestamp_ = fs.timestamp();

		StreamPacket spkt;
		spkt.version = 5;
		spkt.timestamp = fs.timestamp();
		spkt.localTimestamp = fs.localTimestamp;
		spkt.streamID = fs.frameset();
		//spkt.frame_number = 0;
		spkt.channel = c;
		spkt.flags = ftl::codecs::kFlagCompleted;

		ftl::codecs::Packet pkt;
		pkt.frame_count = 1; //fs.frames.size();
		pkt.codec = codec_t::Invalid;

		for (size_t i=0; i<fs.frames.size(); ++i) {
			spkt.frame_number = i;
			pkt.packet_count = static_cast<uint8_t>(fs.frames[i].packet_tx+1);  // FIXME: 255 limit currently
			_send(fs, spkt, pkt);
		}

		if (injection_timestamp_ >= spkt.timestamp) {
			do_reinject_.clear();
		}
		return;
	}

	std::unordered_set<ftl::codecs::Channel> selected;
	if (stream_->size() > 0) selected = stream_->selected(fs.frameset());

	bool do_inject = !do_inject_.test_and_set();
	bool do_iframe = _checkNeedsIFrame(fs.timestamp(), do_inject);
	if (!do_reinject_.test_and_set()) {
		do_inject = true;
	}

	FTL_Profile("SenderPost", 0.02);

	bool available = false;
	bool needs_encoding = true;

	int valid_frames = 0;
	//int ccount = 0;
	int forward_count = 0;

	if (do_inject) {
		_sendPersistent(fs);
	}

	for (size_t i=0; i<fs.frames.size(); ++i) {
		auto &frame = fs.frames[i];
		if (!frame.has(c)) continue;

		++valid_frames;
		//++fs.flush_count;

		//ccount += frame.changed().size();

		if (selected.find(c) != selected.end() || (int)c >= 32) {
			// FIXME: Sends high res colour, but receive end currently broken
			//auto cc = (c == Channel::Colour && frame.hasChannel(Channel::ColourHighRes)) ? Channel::ColourHighRes : c;
			auto cc = c;

			// Check if there are existing encoded packets
			const auto &packets = frame.getEncoded(cc);
			if (packets.size() > 0) {
				if (packets.size() == 1) {
					
				} else {
					// PROBLEMS
					LOG(WARNING) << "Multi packet send! - Channel = " << int(c) << ", count = " << packets.size();
				}
				forward_count += packets.back().frame_count;
			}
		} else {
			needs_encoding = false;
			available = true;
		}
	}

	//bool last_flush = ccount == fs.flush_count;

	// Don't do anything if channel not in any frames.
	if (valid_frames == 0) return;

	// Can we just forward existing encoding?
	// TODO: Test this code!
	if (forward_count == valid_frames) {
		needs_encoding = false;

		for (size_t i=0; i<fs.frames.size(); ++i) {
			auto &frame = fs.frames[i];
			if (!frame.has(c)) continue;

			const auto &packets = frame.getEncoded(c);
			//if (packets.size() == 1) {
				StreamPacket spkt;
				spkt.version = 5;
				spkt.timestamp = fs.timestamp();
				spkt.localTimestamp = fs.localTimestamp;
				spkt.streamID = fs.frameset(); //fs.id;
				spkt.frame_number = i;
				spkt.channel = c;
				//spkt.flags = (last_flush) ? ftl::codecs::kFlagCompleted : 0;

				//stream_->post(spkt, packets.back());
				_send(fs, spkt, packets.back());
			//} else if (packets.size() > 1) {
				// PROBLEMS
			//}
		}
	}

	// Don't transmit if noencode and needs encoding
	if (needs_encoding && noencode) {
		needs_encoding = false;
		available = true;
	}

	if (available) {
		// Not selected so send an empty packet...
		StreamPacket spkt;
		spkt.version = 5;
		spkt.timestamp = fs.timestamp();
		spkt.localTimestamp = fs.localTimestamp;
		spkt.streamID = fs.frameset();
		spkt.frame_number = 0;
		spkt.channel = c;
		//spkt.flags = (last_flush) ? ftl::codecs::kFlagCompleted : 0;

		Packet pkt;
		pkt.codec = codec_t::Any;
		pkt.frame_count = 1;
		pkt.bitrate = 0;
		pkt.flags = 0;
		//stream_->post(spkt, pkt);
		_send(fs, spkt, pkt);
	}

	if (needs_encoding) {
		_encodeChannel(fs, c, do_iframe);
	}
}

void Sender::forceAvailable(ftl::data::FrameSet &fs, ftl::codecs::Channel c) {
	StreamPacket spkt;
	spkt.version = 5;
	spkt.timestamp = fs.timestamp();
	spkt.localTimestamp = fs.localTimestamp;
	spkt.streamID = fs.frameset();
	spkt.frame_number = 0;
	spkt.channel = c;

	Packet pkt;
	pkt.codec = codec_t::Any;
	pkt.frame_count = 1;
	pkt.bitrate = 0;
	pkt.flags = 0;
	stream_->post(spkt, pkt);
}

void Sender::post(ftl::data::Frame &frame, ftl::codecs::Channel c) {
	if (!stream_) return;

	FTL_Profile("SenderPost", 0.02);

	bool available = false;
	bool needs_encoding = true;

		// FIXME: Allow data channel selection rather than always send
		/*for (auto c : frame.getDataChannels()) {
			StreamPacket spkt;
			spkt.version = 4;
			spkt.timestamp = fs.timestamp;
			spkt.streamID = 0; //fs.id;
			spkt.frame_number = i;
			spkt.channel = c;

			ftl::codecs::Packet pkt;
			pkt.codec = ftl::codecs::codec_t::MSGPACK;
			pkt.frame_count = 1;
			pkt.flags = 0;
			pkt.bitrate = 0;
			pkt.data = frame.getRawData(c);
			stream_->post(spkt, pkt);
		}*/

		//for (auto ic : frame.changed()) {
			//auto c = ic.first;
			if (true) { //if (selected.has(c)) {
				// FIXME: Sends high res colour, but receive end currently broken
				//auto cc = (c == Channel::Colour && frame.hasChannel(Channel::ColourHighRes)) ? Channel::ColourHighRes : c;
				auto cc = c;

				StreamPacket spkt;
				spkt.version = 5;
				spkt.timestamp = frame.timestamp();
				spkt.localTimestamp = spkt.timestamp;
				spkt.streamID = frame.frameset(); //fs.id;
				spkt.frame_number = frame.source();
				spkt.channel = c;

				// Check if there are existing encoded packets
				const auto &packets = frame.getEncoded(cc);
				if (packets.size() > 0) {
					needs_encoding = false;
					if (packets.size() > 1) {
						LOG(WARNING) << "Multi-packet send: " << (int)cc;
						ftl::codecs::Packet pkt;
						//mergeNALUnits(packets, pkt);
						//stream_->post(spkt, pkt);
					} else {
						// Send existing encoding instead of re-encoding
						//for (auto &pkt : packets) {
						stream_->post(spkt, packets.front());
						//}
					}
				}
			} else {
				available = true;
			}
		//}

	if (available) {
		// Not selected so send an empty packet...
		StreamPacket spkt;
		spkt.version = 5;
		spkt.timestamp = frame.timestamp();
		spkt.localTimestamp = spkt.timestamp;
		spkt.streamID = frame.frameset();
		spkt.frame_number = 0;
		spkt.channel = c;

		Packet pkt;
		pkt.codec = codec_t::Any;
		pkt.frame_count = 1;
		pkt.bitrate = 0;
		stream_->post(spkt, pkt);
	}

	if (needs_encoding) {
		// TODO: One thread per channel.
		_encodeChannel(frame, c, false);
	}

	//do_inject_ = false;
}

void Sender::resetEncoders(uint32_t fsid) {
	LOG(INFO) << "Reset encoders for " << fsid;
	for (auto &t : state_) {
		if ((t.first >> 16) == static_cast<int>(fsid)) {
			if (t.second.encoder[0]) {
				// Remove unwanted encoder
				ftl::codecs::free(t.second.encoder[0]);
				t.second.encoder[0] = nullptr;
				if (t.second.encoder[1]) {
					ftl::codecs::free(t.second.encoder[1]);
					t.second.encoder[1] = nullptr;
				}
				LOG(INFO) << "Removing encoder for channel " << (t.first & 0xFF);
			}
		}
	}
}

void Sender::setActiveEncoders(uint32_t fsid, const std::unordered_set<Channel> &ec) {
	for (auto &t : state_) {
		if ((t.first >> 16) == static_cast<int>(fsid)) {
			if (t.second.encoder[0] && ec.count(static_cast<Channel>(t.first & 0xFF)) == 0) {
				// Remove unwanted encoder
				ftl::codecs::free(t.second.encoder[0]);
				t.second.encoder[0] = nullptr;
				if (t.second.encoder[1]) {
					ftl::codecs::free(t.second.encoder[1]);
					t.second.encoder[1] = nullptr;
				}
				LOG(INFO) << "Removing encoder for channel " << (t.first & 0xFF);
			}
		}
	}
}

void Sender::_encodeVideoChannel(ftl::data::FrameSet &fs, Channel c, bool reset) {
	bool isfloat = ftl::codecs::type(c) == CV_32F;

	bool lossless = (isfloat) ? value("lossless_float", false) : value("lossless_colour", false);
	int max_bitrate = std::max(0, std::min(255, value("bitrate", 64)));
	int bitrate = std::min(static_cast<uint8_t>(max_bitrate), _getMinBitrate());
	if (isfloat) bitrate = std::min(255, int(float(bitrate)*value("bitrate_float_scale", 1.5f)));

	//int min_bitrate = std::max(0, std::min(255, value("min_bitrate", 0)));  // TODO: Use this
	codec_t codec = static_cast<codec_t>(
		(isfloat) ?	value("codec_float", static_cast<int>(codec_t::Any)) :
					value("codec_colour", static_cast<int>(codec_t::Any)));

	device_t device = static_cast<device_t>(value("encoder_device", static_cast<int>(device_t::Any)));

	if (codec == codec_t::Any) {
		codec = (lossless) ? codec_t::HEVC_LOSSLESS : codec_t::HEVC;
	}

	// TODO: Support high res
	bool is_stereo = value("stereo", false) && c == Channel::Colour && fs.firstFrame().hasChannel(Channel::Colour2);

	uint32_t offset = 0;
	while (offset < fs.frames.size()) {
		Channel cc = c;
		//if ((cc == Channel::Colour) && fs.firstFrame().hasChannel(Channel::ColourHighRes)) {
		//	cc = Channel::ColourHighRes;
		//}
		
		//if ((cc == Channel::Right) && fs.firstFrame().hasChannel(Channel::RightHighRes)) {
		//	cc = Channel::RightHighRes;
		//	fs.frames[offset].upload(cc);
		//}

		if (!fs.frames[offset].hasChannel(cc)) {
			offset++;
			continue;
		}

		StreamPacket spkt;
		spkt.version = 5;
		spkt.timestamp = fs.timestamp();
		spkt.localTimestamp = fs.localTimestamp;
		spkt.streamID = fs.frameset();
		spkt.frame_number = offset;
		spkt.channel = c;

		auto &tile = _getTile(fs.id(), cc);

		ftl::codecs::Encoder *enc = tile.encoder[(offset==0)?0:1];
		if (!enc) {
			enc = ftl::codecs::allocateEncoder(
				definition_t::HD1080, device, codec);
			tile.encoder[(offset==0)?0:1] = enc;
		}

		if (!enc) {
			LOG(ERROR) << "No encoder";
			return;
		}
		if (enc->device() == device_t::OpenCV) {
			LOG(WARNING) << "Software encoder for " << ftl::codecs::name(c);
		}

		// Upload if in host memory
		for (auto &f : fs.frames) {
			if (!fs.hasFrame(f.source())) continue;

			// FIXME:
			//if (f.isCPU(c)) {
			//	f.upload(Channels<0>(cc), cv::cuda::StreamAccessor::getStream(enc->stream()));
			//}
		}

		int count = _generateTiles(fs, offset, cc, enc->stream(), is_stereo);
		if (count <= 0) {
			LOG(ERROR) << "Could not generate tiles.";
			break;
		}

		//cudaSafeCall(cudaStreamSynchronize(enc->stream()));
		//enc->stream().waitForCompletion();

		if (enc) {
			if (reset) enc->reset();

			try {
				ftl::codecs::Packet pkt;
				pkt.frame_count = count;
				pkt.codec = codec;
				pkt.bitrate = bitrate;
				pkt.flags = 0;

				// In the event of partial frames, add a flag to indicate that
				//if (static_cast<size_t>(fs.count) < fs.frames.size()) pkt.flags |= ftl::codecs::kFlagPartial;

				// Choose correct region of interest into the surface.
				cv::Rect roi = _generateROI(fs, cc, offset, is_stereo);
				cv::cuda::GpuMat sroi = tile.surface(roi);

				FTL_Profile("Encoder",0.02);

				if (enc->encode(sroi, pkt)) {
					//stream_->post(spkt, pkt);
					_send(fs, spkt, pkt);

					/*cv::Mat tmp;
					tile.surface.download(tmp);
					cv::imshow("Test", tmp);
					cv::waitKey(1);*/
				} else {
					LOG(ERROR) << "Encoding failed for channel " << (int)cc;
				}
			} catch (std::exception &e) {
				LOG(ERROR) << "Exception in encoder: " << e.what();
			}
		} else {
			LOG(ERROR) << "No encoder";
		}

		offset += count;
	}
}

void Sender::_encodeAudioChannel(ftl::data::FrameSet &fs, Channel c, bool reset) {
	
	// TODO: combine into multiple opus streams
	for (size_t i=0; i<fs.frames.size(); ++i) {
		if (!fs.frames[i].hasChannel(c)) continue;

		const auto &listdata = fs.frames[i].get<std::list<ftl::audio::Audio>>(c);

		//auto &settings = fs.frames[i].getSettings();

		StreamPacket spkt;
		spkt.version = 5;
		spkt.timestamp = fs.timestamp();
		spkt.localTimestamp = fs.localTimestamp;
		spkt.streamID = fs.frameset();
		spkt.frame_number = i;
		spkt.channel = c;
		//spkt.flags = (last_flush) ? ftl::codecs::kFlagCompleted : 0;

		ftl::codecs::Packet pkt;
		pkt.codec = ftl::codecs::codec_t::OPUS;
		pkt.frame_count = 1;
		pkt.flags = (c == Channel::AudioStereo) ? ftl::codecs::kFlagStereo : 0;
		pkt.bitrate = 180;

		// Find encoder here ...
		ftl::audio::Encoder *enc = _getAudioEncoder(fs.frameset(), i, c, pkt);

		// Do encoding into pkt.data
		if (!enc) {
			LOG(ERROR) << "Could not find audio encoder";
			return;
		}
		
		for (auto &data : listdata) {
			if (!enc->encode(data.data(), pkt)) {
				LOG(ERROR) << "Could not encode audio";
				return;
			}
		}

		_send(fs, spkt, pkt);
		//stream_->post(spkt, pkt);
	}
}

void Sender::_encodeDataChannel(ftl::data::FrameSet &fs, Channel c, bool reset) {
	int i=0;

	// TODO: Pack all frames into a single packet
	for (auto &f : fs.frames) {
		StreamPacket spkt;
		spkt.version = 5;
		spkt.timestamp = fs.timestamp();
		spkt.localTimestamp = fs.localTimestamp;
		spkt.streamID = fs.frameset();
		spkt.frame_number = i++;
		spkt.channel = c;
		//spkt.flags = (last_flush) ? ftl::codecs::kFlagCompleted : 0;

		ftl::codecs::Packet pkt;
		pkt.frame_count = 1;
		pkt.codec = codec_t::MSGPACK;
		pkt.bitrate = 255;
		pkt.flags = 0;
		
		auto encoder = ftl::data::getTypeEncoder(f.type(c));
		if (encoder) {
			if (encoder(f, c, pkt.data)) {
				//stream_->post(spkt, pkt);
				_send(fs, spkt, pkt);
			}
		} else {
			LOG(WARNING) << "Missing msgpack encoder";
		}
	}
}

void Sender::_encodeDataChannel(ftl::data::Frame &f, Channel c, bool reset) {
	StreamPacket spkt;
	spkt.version = 5;
	spkt.timestamp = f.timestamp();
	spkt.localTimestamp = spkt.timestamp;
	spkt.streamID = f.frameset();
	spkt.frame_number = f.source();
	spkt.channel = c;

	ftl::codecs::Packet pkt;
	pkt.frame_count = 1;
	pkt.codec = codec_t::MSGPACK;
	pkt.bitrate = 255;
	pkt.flags = 0;
	
	auto encoder = ftl::data::getTypeEncoder(f.type(c));
	if (encoder) {
		if (encoder(f, c, pkt.data)) {
			stream_->post(spkt, pkt);
		}
	} else {
		LOG(WARNING) << "Missing msgpack encoder";
	}
}

void Sender::_encodeChannel(ftl::data::FrameSet &fs, Channel c, bool reset) {
	int ic = int(c);

	if (ic < 32) {
		_encodeVideoChannel(fs, c, reset);
	} else if (ic < 64) {
		_encodeAudioChannel(fs, c, reset);
	} else {
		_encodeDataChannel(fs, c, reset);
	}
}

void Sender::_encodeChannel(ftl::data::Frame &frame, Channel c, bool reset) {
	int ic = int(c);

	if (ic < 32) {
		//_encodeVideoChannel(frame, c, reset);
	} else if (ic < 64) {
		//_encodeAudioChannel(frame, c, reset);
	} else {
		_encodeDataChannel(frame, c, reset);
	}
}

cv::Rect Sender::_generateROI(const ftl::rgbd::FrameSet &fs, ftl::codecs::Channel c, int offset, bool stereo) {
	const ftl::data::Frame &cframe = fs.firstFrame();
	int rwidth = cframe.get<cv::cuda::GpuMat>(c).cols;
	if (stereo) rwidth *= 2;
	int rheight = cframe.get<cv::cuda::GpuMat>(c).rows;
	auto [tx,ty] = ftl::codecs::chooseTileConfig(fs.frames.size()-offset);
	return cv::Rect(0, 0, tx*rwidth, ty*rheight);
}

Sender::EncodingState &Sender::_getTile(int fsid, Channel c) {
	int id = (fsid << 8) | (int)c;

	{
		SHARED_LOCK(mutex_, lk);
		auto i = state_.find(id);
		if (i != state_.end()) {
			return (*i).second;
		}
	}

	UNIQUE_LOCK(mutex_, lk);
	state_[id] = {
		uint8_t(255),
		{nullptr,nullptr},
		cv::cuda::GpuMat(),
		0
	};
	return state_[id];
}

float Sender::_selectFloatMax(Channel c) {
	switch (c) {
	case Channel::Depth		: return 16.0f;
	default					: return 1.0f;
	}
}

int Sender::_generateTiles(const ftl::rgbd::FrameSet &fs, int offset, Channel c, cv::cuda::Stream &stream, bool stereo) {
	auto &surface = _getTile(fs.id(), c);

	const ftl::data::Frame *cframe = nullptr; //&fs.frames[offset];

	const auto &m = fs.firstFrame().get<cv::cuda::GpuMat>(c);

	// Choose tile configuration and allocate memory
	auto [tx,ty] = ftl::codecs::chooseTileConfig(fs.frames.size());
	int rwidth = (stereo) ? m.cols*2 : m.cols;
	int rheight = m.rows;
	int width = tx * rwidth;
	int height = ty * rheight;
	int tilecount = tx*ty;
	uint32_t count = 0;

	int cvtype = m.type();

	surface.surface.create(height, width, cvtype);

	// Loop over tiles with ROI mats and do colour conversions.
	while (tilecount > 0 && count+offset < fs.frames.size()) {
		if (fs.hasFrame(offset+count)) {
			cframe = &fs.frames[offset+count];
			auto &m = cframe->get<cv::cuda::GpuMat>(c);
			cv::Rect roi((count % tx)*rwidth, (count / tx)*rheight, (stereo) ? rwidth/2 : rwidth, rheight);
			cv::cuda::GpuMat sroi = surface.surface(roi);

			m.copyTo(sroi, stream);
		} else {
			cv::Rect roi((count % tx)*rwidth, (count / tx)*rheight, rwidth, rheight);
			cv::cuda::GpuMat sroi = surface.surface(roi);
			sroi.setTo(cv::Scalar(0));
		}

		++count;
		--tilecount;
	}

	return count;
}
