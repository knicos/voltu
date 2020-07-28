#include <ftl/streams/receiver.hpp>
#include <ftl/codecs/depth_convert_cuda.hpp>
#include <ftl/profiler.hpp>
#include <ftl/audio/software_decoder.hpp>
#include <ftl/rgbd/capabilities.hpp>

#include <opencv2/cudaimgproc.hpp>
#include <opencv2/highgui.hpp>

#include <ftl/streams/parsers.hpp>
#include <ftl/streams/injectors.hpp>

#define LOGURU_REPLACE_GLOG 1
#include <loguru.hpp>

using ftl::stream::Receiver;
using ftl::stream::Stream;
using ftl::codecs::StreamPacket;
using ftl::codecs::Packet;
using ftl::codecs::Channel;
using std::string;
using ftl::stream::parseCalibration;
using ftl::stream::parsePose;
using ftl::stream::parseConfig;
using ftl::stream::injectCalibration;
using ftl::stream::injectPose;
using ftl::rgbd::Capability;

Receiver::Receiver(nlohmann::json &config, ftl::data::Pool *p) : ftl::Configurable(config), stream_(nullptr), pool_(p) {
	timestamp_ = 0;
	second_channel_ = Channel::Depth;
	frame_mask_ = value("frame_mask", 0xFFFFFFFFu);

	//size_t bsize = value("frameset_buffer_size", 3);
	/*for (size_t i=0; i<ftl::stream::kMaxStreams; ++i) {
		builder_[i].setID(i);
		builder_[i].setBufferSize(bsize);
	}*/

	on("frameset_buffer_size", [this]() {
		size_t bsize = value("frameset_buffer_size", 3);
		for (auto &i : builders_) {
			i.second->setBufferSize(bsize);
		}
	});

	on("frame_mask", [this]() {
		frame_mask_ = value("frame_mask", 0xFFFFFFFFu);
	});
}

Receiver::~Receiver() {
	//if (stream_) {
	//	stream_->onPacket(nullptr);
	//}

	//builder_[0].onFrameSet(nullptr);
}

void Receiver::loopback(ftl::data::Frame &f, ftl::codecs::Channel c) {
	auto &build = builder(f.frameset());
	auto fs = build.get(f.timestamp(), f.source());
	fs->frames[f.source()].informChange(c, build.changeType(), f.getAnyMutable(c));
	//f.remove(c);
}

ftl::streams::BaseBuilder &Receiver::builder(uint32_t id) {
	auto i = builders_.find(id);
	if (i == builders_.end()) {
		auto fb = new ftl::streams::ForeignBuilder();
		builders_[id] = std::shared_ptr<ftl::streams::BaseBuilder>(fb);
		auto &b = builders_[id];
		b->setID(id);
		b->setPool(pool_);
		fb->setBufferSize(value("frameset_buffer_size", 3));
		handles_[id] = std::move(fb->onFrameSet([this](const ftl::data::FrameSetPtr& fs) {
			callback_.trigger(fs);
			return true;
		}));
		return *b;
	} else {
		return *(i->second);
	}
}

void Receiver::removeBuilder(uint32_t id) {
	UNIQUE_LOCK(mutex_, lk);
	auto i = builders_.find(id);
	if (i != builders_.end()) {
		handles_.erase(id);
		builders_.erase(i);
	}
}

void Receiver::registerBuilder(const std::shared_ptr<ftl::streams::BaseBuilder> &b) {
	auto i = builders_.find(b->id());
	if (i != builders_.end()) throw FTL_Error("Builder already exists");
	builders_[b->id()] = b;
	handles_[b->id()] = std::move(b->onFrameSet([this](const ftl::data::FrameSetPtr& fs) {
		callback_.trigger(fs);
		return true;
	}));
}

//void Receiver::onAudio(const ftl::audio::FrameSet::Callback &cb) {
//	audio_cb_ = cb;
//}

/*void Receiver::_processConfig(InternalStates &frame, const ftl::codecs::Packet &pkt) {
	std::string cfg;
	auto unpacked = msgpack::unpack((const char*)pkt.data.data(), pkt.data.size());
	unpacked.get().convert(cfg);

	LOG(INFO) << "Config Received: " << cfg;
	// TODO: This needs to be put in safer / better location
	//host_->set(std::get<0>(cfg), nlohmann::json::parse(std::get<1>(cfg)));
}*/

void Receiver::_createDecoder(InternalVideoStates &frame, int chan, const ftl::codecs::Packet &pkt) {
	UNIQUE_LOCK(frame.mutex,lk);
	auto *decoder = frame.decoders[chan];
	if (decoder) {
		if (!decoder->accepts(pkt)) {
			//UNIQUE_LOCK(mutex_,lk);
			ftl::codecs::free(frame.decoders[chan]);
		} else {
			return;
		}
	}

	//UNIQUE_LOCK(mutex_,lk);
	frame.decoders[chan] = ftl::codecs::allocateDecoder(pkt);
}

Receiver::InternalVideoStates::InternalVideoStates() {
	for (int i=0; i<32; ++i) decoders[i] = nullptr;
	timestamp = -1;
}

Receiver::InternalVideoStates &Receiver::_getVideoFrame(const StreamPacket &spkt, int ix) {
	uint32_t fn = spkt.frameNumber()+ix;

	UNIQUE_LOCK(mutex_, lk);
	while (video_frames_[spkt.streamID].size() <= fn) {
		//frames_.resize(spkt.frameNumber()+1);
		video_frames_[spkt.streamID].push_back(new InternalVideoStates);
		//video_frames_[spkt.streamID][video_frames_[spkt.streamID].size()-1]->state.set("name",std::string("Source ")+std::to_string(fn+1));
	}
	auto &f = *video_frames_[spkt.streamID][fn];
	//if (!f.frame.origin()) f.frame.setOrigin(&f.state);
	return f;
}

Receiver::InternalAudioStates::InternalAudioStates() : decoder(nullptr) {

}

Receiver::InternalAudioStates &Receiver::_getAudioFrame(const StreamPacket &spkt, int ix) {
	uint32_t fn = spkt.frameNumber()+ix;

	UNIQUE_LOCK(mutex_, lk);
	while (audio_frames_[spkt.streamID].size() <= fn) {
		//frames_.resize(spkt.frameNumber()+1);
		audio_frames_[spkt.streamID].push_back(new InternalAudioStates);
		//audio_frames_[spkt.streamID][audio_frames_[spkt.streamID].size()-1]->state.set("name",std::string("Source ")+std::to_string(fn+1));
	}
	auto &f = *audio_frames_[spkt.streamID][fn];
	//if (!f.frame.origin()) f.frame.setOrigin(&f.state);
	return f;
}

void Receiver::_processData(const StreamPacket &spkt, const Packet &pkt) {
	auto &build = builder(spkt.streamID);
	auto fs = build.get(spkt.timestamp, spkt.frame_number);
	auto &f = (spkt.frame_number == 255) ? **fs : fs->frames[spkt.frame_number];

	// Remove LIVE capability if stream hints it is recorded
	if (spkt.channel == Channel::Capabilities && (spkt.hint_capability & ftl::codecs::kStreamCap_Recorded)) {
		std::any data;
		ftl::data::decode_type<std::unordered_set<Capability>>(data, pkt.data);

		auto &cap = *std::any_cast<std::unordered_set<Capability>>(&data);
		if (cap.count(Capability::LIVE)) {
			cap.erase(Capability::LIVE);
		}
		cap.emplace(Capability::STREAMED);

		f.informChange(spkt.channel, build.changeType(), data);
	} else if (spkt.channel == Channel::Pose && pkt.codec == ftl::codecs::codec_t::POSE) {
		// TODO: Remove this eventually, it allows old FTL files to work
		std::any data;
		auto &pose = data.emplace<Eigen::Matrix4d>();
		pose = Eigen::Map<Eigen::Matrix4d>((double*)pkt.data.data());
		f.informChange(spkt.channel, build.changeType(), data);
	} else {
		f.informChange(spkt.channel, build.changeType(), pkt);
	}

	if (spkt.channel == Channel::Calibration) {
		const auto &calibration = std::get<0>(f.get<std::tuple<ftl::rgbd::Camera, ftl::codecs::Channel, int>>(Channel::Calibration));
		InternalVideoStates &ividstate = _getVideoFrame(spkt);
		ividstate.width = calibration.width;
		ividstate.height = calibration.height;
	}

	// TODO: Adjust metadata also for recorded streams

	if (spkt.flags & ftl::codecs::kFlagCompleted) {
		//UNIQUE_LOCK(vidstate.mutex, lk);
		timestamp_ = spkt.timestamp;
		fs->completed(spkt.frame_number);
	}

	fs->localTimestamp = spkt.localTimestamp;

	/*const auto *cs = stream_;
	const auto sel = stream_->selected(spkt.frameSetID()) & cs->available(spkt.frameSetID());

	if (f.hasAll(sel)) {
		timestamp_ = spkt.timestamp;
		fs->completed(spkt.frame_number);
	}*/
}

ftl::audio::Decoder *Receiver::_createAudioDecoder(InternalAudioStates &frame, const ftl::codecs::Packet &pkt) {
	if (!frame.decoder) frame.decoder = new ftl::audio::SoftwareDecoder();
	return frame.decoder;
}

void Receiver::_processAudio(const StreamPacket &spkt, const Packet &pkt) {
	// Audio Data
	InternalAudioStates &state = _getAudioFrame(spkt);

	//frame.frame.reset();
	state.timestamp = spkt.timestamp;

	auto &build = builder(spkt.streamID);
	auto fs = build.get(spkt.timestamp, spkt.frame_number+pkt.frame_count-1);
	auto &frame = fs->frames[0];

	auto &audiolist = frame.createChange<std::list<ftl::audio::Audio>>(spkt.channel, build.changeType(), pkt);
	auto &audio = audiolist.emplace_back();

	//size_t size = pkt.data.size()/sizeof(short);
	//audio.data().resize(size);
	//auto *ptr = (short*)pkt.data.data();
	//for (size_t i=0; i<size; i++) audio.data()[i] = ptr[i];

	ftl::audio::Decoder *dec = _createAudioDecoder(state, pkt);
	if (!dec) {
		LOG(ERROR) << "Could get an audio decoder";
		return;
	}
	if (!dec->decode(pkt, audio.data())) {
		LOG(ERROR) << "Audio decode failed";
		return;
	}

	if (spkt.flags & ftl::codecs::kFlagCompleted) {
		//UNIQUE_LOCK(vidstate.mutex, lk);
		timestamp_ = spkt.timestamp;
		fs->completed(spkt.frame_number);
	}

	fs->localTimestamp = spkt.localTimestamp;

	// Generate settings from packet data
	/*ftl::audio::AudioSettings settings;
	settings.channels = (spkt.channel == Channel::AudioStereo) ? 2 : 1;
	settings.frame_size = 960;

	switch (pkt.definition) {
	case definition_t::hz48000		: settings.sample_rate = 48000; break;
	case definition_t::hz44100		: settings.sample_rate = 44100; break;
	default: settings.sample_rate = 48000; break;
	}*/

	//frame.state.setLeft(settings);
	//frame.frame.setOrigin(&frame.state);

	/*if (audio_cb_) {
		// Create an audio frameset wrapper.
		ftl::audio::FrameSet fs;
		fs.id = 0;
		fs.timestamp = frame.timestamp;
		//fs.originClockDelta;
		fs.count = 1;
		//fs.stale = false;
		fs.clear(ftl::data::FSFlag::STALE);
		frame.frame.swapTo(fs.frames.emplace_back());

		audio_cb_(fs);
	}*/
}

namespace sgm {
	namespace details {
		void median_filter(const uint16_t* d_src, uint16_t* d_dst, int width, int height, int pitch, cudaStream_t stream);
	}
}

void Receiver::_processVideo(const StreamPacket &spkt, const Packet &pkt) {
	FTL_Profile("VideoPacket", 0.02);

	//const ftl::codecs::Channel rchan = spkt.channel;
	const unsigned int channum = (unsigned int)spkt.channel;
	InternalVideoStates &ividstate = _getVideoFrame(spkt);

	auto [tx,ty] = ftl::codecs::chooseTileConfig(pkt.frame_count);

	// Get the frameset
	auto &build = builder(spkt.streamID);
	auto fs = build.get(spkt.timestamp, spkt.frame_number+pkt.frame_count-1);  // TODO: This is a hack

	//if (!fs->frames[spkt.frame_number].has(Channel::Calibration)) {
	//	LOG(WARNING) << "No calibration, skipping frame: " << spkt.timestamp;
	//	return;
	//}

	//const auto &calibration = std::get<0>(fs->frames[spkt.frame_number].get<std::tuple<ftl::rgbd::Camera, ftl::codecs::Channel, int>>(Channel::Calibration));
	int width = ividstate.width; //calibration.width;
	int height = ividstate.height; //calibration.height;

	if (width == 0 || height == 0) {
		// Attempt to retry the decode later
		// Make a copy of the packets into a thread job
		// FIXME: Check that thread pool does not explode
		if (spkt.retry_count < 10) {
			LOG(WARNING) << "No calibration, retrying: " << spkt.timestamp;
			ftl::pool.push([this, spkt, pkt](int id) mutable {
				++const_cast<StreamPacket&>(spkt).retry_count;
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				_processVideo(spkt, pkt);
			});
		} else {
			LOG(WARNING) << "No calibration, failed frame: " << spkt.timestamp;
		}
		return;
	}

	//LOG(INFO) << " CODEC = " << (int)pkt.codec << " " << (int)pkt.flags << " " << (int)spkt.channel;
	//LOG(INFO) << "Decode surface: " << (width*tx) << "x" << (height*ty);

	auto &surface = ividstate.surface[static_cast<int>(spkt.channel)];

	// Allocate a decode surface, this is a tiled image to be split later
	int cvtype = ftl::codecs::type(spkt.channel);
	if (cvtype == CV_32F) {
		//cvtype = CV_16U; //(pkt.flags & 0x2) ? CV_16UC4 : CV_16U;
		//if (pkt.flags & 0x2) sheight += sheight/2;
	}

	//surface.create(height*ty, width*tx, ((isFloatChannel(spkt.channel)) ? ((pkt.flags & 0x2) ? CV_16UC4 : CV_16U) : CV_8UC4));
	surface.create(height*ty, width*tx, cvtype);

	bool is_static = ividstate.decoders[channum] && (spkt.hint_capability & ftl::codecs::kStreamCap_Static);

	// Find or create the decoder
	_createDecoder(ividstate, channum, pkt);
	auto *decoder = ividstate.decoders[channum];
	if (!decoder) {
		LOG(ERROR) << "No frame decoder available";
		return;
	}

	// Do the actual decode into the surface buffer
	if (!is_static) {
		try {
			FTL_Profile("Decode", 0.015);
			if (!decoder->decode(pkt, surface)) {
				LOG(ERROR) << "Decode failed on channel " << (int)spkt.channel;
				return;
			}
		} catch (std::exception &e) {
			LOG(ERROR) << "Decode failed for " << spkt.timestamp << ": " << e.what();
			return;
		}
	}

	auto cvstream = cv::cuda::StreamAccessor::wrapStream(decoder->stream());

	/*if (spkt.channel == Channel::Depth && (pkt.flags & 0x2)) {
	cv::Mat tmp;
	surface.download(tmp);
	cv::imshow("Test", tmp);
	cv::waitKey(1);
	}*/

	// Mark a frameset as being partial
	if (pkt.flags & ftl::codecs::kFlagPartial) {
		fs->markPartial();
	}

	// Now split the tiles from surface into frames, doing colour conversions
	// at the same time.
	// Note: Done in reverse to allocate correct number of frames first time round
	for (int i=pkt.frame_count-1; i>=0; --i) {
		//InternalVideoStates &vidstate = _getVideoFrame(spkt,i);
		auto &frame = fs->frames[spkt.frame_number+i];

		//if (!frame.origin()) frame.setOrigin(&vidstate.state);

		if (frame.hasChannel(spkt.channel)) {
			// FIXME: Is this a corruption in recording or in playback?
			// Seems to occur in same place in ftl file, one channel is missing
			LOG(WARNING) << "Previous frame not complete: " << spkt.timestamp;
		}

		// Add channel to frame and allocate memory if required
		const cv::Size size = cv::Size(width, height);
		auto &buf = frame.createChange<ftl::rgbd::VideoFrame>(spkt.channel, build.changeType(), pkt).createGPU();
		buf.create(size, ftl::codecs::type(spkt.channel)); //(isFloatChannel(rchan) ? CV_32FC1 : CV_8UC4));

		cv::Rect roi((i % tx)*width, (i / tx)*height, width, height);
		cv::cuda::GpuMat sroi = surface(roi);
		sroi.copyTo(buf, cvstream);
	}

	// Must ensure all processing is finished before completing a frame.
	cudaSafeCall(cudaStreamSynchronize(decoder->stream()));

	for (int i=0; i<pkt.frame_count; ++i) {
		InternalVideoStates &vidstate = _getVideoFrame(spkt,i);
		//auto &frame = builder(spkt.streamID).get(spkt.timestamp, spkt.frame_number+i);
		//auto &frame = fs->frames[spkt.frame_number+i];

		/*if (spkt.version < 5) {
			const auto *cs = stream_;
			const auto sel = stream_->selected(spkt.frameSetID()) & cs->available(spkt.frameSetID());

			UNIQUE_LOCK(vidstate.mutex, lk);
			if (frame.availableAll(sel)) {
				timestamp_ = spkt.timestamp;
				fs->completed(spkt.frame_number+i);
			}
		}*/

		if (spkt.flags & ftl::codecs::kFlagCompleted) {
			UNIQUE_LOCK(vidstate.mutex, lk);
			timestamp_ = spkt.timestamp;
			fs->completed(spkt.frame_number+i);
		}
	}

	fs->localTimestamp = spkt.localTimestamp;
}

void Receiver::processPackets(const StreamPacket &spkt, const Packet &pkt) {
	const unsigned int channum = (unsigned int)spkt.channel;

	// No data packet means channel is only available.
	if (pkt.data.size() == 0) {
		if (spkt.streamID < 255 && !(spkt.flags & ftl::codecs::kFlagRequest)) {
			// Get the frameset
			auto fs = builder(spkt.streamID).get(spkt.timestamp, spkt.frame_number+pkt.frame_count-1);
			const auto *cs = stream_;
			const auto sel = stream_->selected(spkt.frameSetID()) & cs->available(spkt.frameSetID());

			for (auto &frame : fs->frames) {
				//LOG(INFO) << "MARK " << frame.source() << " " << (int)spkt.channel;
				frame.markAvailable(spkt.channel);

				if (spkt.flags & ftl::codecs::kFlagCompleted) {
					//UNIQUE_LOCK(vidstate.mutex, lk);  // FIXME: Should have a lock here...
					timestamp_ = spkt.timestamp;
					fs->completed(frame.source());
				}

				//if (frame.availableAll(sel)) {
					//LOG(INFO) << "FRAME COMPLETED " << frame.source();
				//	fs->completed(frame.source());
				//}
			}

			fs->localTimestamp = spkt.localTimestamp;
		}
		return;
	}

	//LOG(INFO) << "PACKET: " << spkt.timestamp << ", " << (int)spkt.channel << ", " << (int)pkt.codec << ", " << (int)pkt.definition;

	// TODO: Allow for multiple framesets
	//if (spkt.frameSetID() > 0) LOG(INFO) << "Frameset " << spkt.frameSetID() << " received: " << (int)spkt.channel;
	if (spkt.frameSetID() >= ftl::stream::kMaxStreams) return;

	// Frameset level data channels
	if (spkt.frameNumber() == 255 && pkt.data.size() > 0) {
		_processData(spkt,pkt);
		return;
	}

	// Too many frames, so ignore.
	//if (spkt.frameNumber() >= value("max_frames",32)) return;
	if (spkt.frameNumber() >= 32 || ((1 << spkt.frameNumber()) & frame_mask_) == 0) return;


	if (channum >= 64) {
		_processData(spkt,pkt);
	} else if (channum >= 32 && channum < 64) {
		_processAudio(spkt,pkt);
	} else {
		_processVideo(spkt,pkt);
	}
}

void Receiver::setStream(ftl::stream::Stream *s) {
	handle_.cancel();
	stream_ = s;

	handle_ = s->onPacket([this](const StreamPacket &spkt, const Packet &pkt) {
		processPackets(spkt, pkt);
		return true;
	});
}

ftl::Handle Receiver::onFrameSet(const std::function<bool(const ftl::data::FrameSetPtr&)> &cb) {
	//for (auto &b : builders_)
	//	b.second.onFrameSet(cb);
	return callback_.on(cb);
}

