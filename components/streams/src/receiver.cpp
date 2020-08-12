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

	on("frameset_buffer_size", [this]() {
		size_t bsize = value("frameset_buffer_size", 0);
		for (auto &i : builders_) {
			i.second->setBufferSize(bsize);
		}
	});
}

Receiver::~Receiver() {
}

void Receiver::loopback(ftl::data::Frame &f, ftl::codecs::Channel c) {
	auto &build = builder(f.frameset());
	auto fs = build.get(f.timestamp(), f.source());
	if (fs) fs->frames[f.source()].informChange(c, build.changeType(), f.getAnyMutable(c));
}

ftl::streams::BaseBuilder &Receiver::builder(uint32_t id) {
	auto i = builders_.find(id);
	if (i == builders_.end()) {
		auto fb = new ftl::streams::ForeignBuilder();
		builders_[id] = std::shared_ptr<ftl::streams::BaseBuilder>(fb);
		auto &b = builders_[id];
		b->setID(id);
		b->setPool(pool_);
		fb->setBufferSize(value("frameset_buffer_size", 0));
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

void Receiver::_createDecoder(InternalVideoStates &frame, int chan, const ftl::codecs::Packet &pkt) {
	UNIQUE_LOCK(frame.mutex,lk);
	auto *decoder = frame.decoders[chan];
	if (decoder) {
		if (!decoder->accepts(pkt)) {
			ftl::codecs::free(frame.decoders[chan]);
		} else {
			return;
		}
	}

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
		video_frames_[spkt.streamID].push_back(new InternalVideoStates);
	}

	auto &f = *video_frames_[spkt.streamID][fn];
	return f;
}

Receiver::InternalAudioStates::InternalAudioStates() : decoder(nullptr) {

}

Receiver::InternalAudioStates &Receiver::_getAudioFrame(const StreamPacket &spkt, int ix) {
	uint32_t fn = spkt.frameNumber()+ix;

	UNIQUE_LOCK(mutex_, lk);
	while (audio_frames_[spkt.streamID].size() <= fn) {
		audio_frames_[spkt.streamID].push_back(new InternalAudioStates);
	}

	auto &f = *audio_frames_[spkt.streamID][fn];
	return f;
}

void Receiver::_processData(const StreamPacket &spkt, const Packet &pkt) {
	auto &build = builder(spkt.streamID);
	auto fs = build.get(spkt.timestamp, spkt.frame_number);

	if (fs) {
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

		fs->localTimestamp = spkt.localTimestamp;
		_finishPacket(fs, spkt.frame_number);

	// Still need to get the calibration data even if frameset is lost.
	} else if (spkt.channel == Channel::Calibration) {
		//LOG(WARNING) << "Calibration being missed in data";
		InternalVideoStates &ividstate = _getVideoFrame(spkt);
		std::any tany;
		ftl::data::decode_type<std::tuple<ftl::rgbd::Camera, ftl::codecs::Channel, int>>(tany, pkt.data);
		auto *cal = std::any_cast<std::tuple<ftl::rgbd::Camera, ftl::codecs::Channel, int>>(&tany);
		if (cal) {
			auto &calibration = std::get<0>(*cal);
			ividstate.width = calibration.width;
			ividstate.height = calibration.height;
		}
	}
}

ftl::audio::Decoder *Receiver::_createAudioDecoder(InternalAudioStates &frame, const ftl::codecs::Packet &pkt) {
	if (!frame.decoder) frame.decoder = new ftl::audio::SoftwareDecoder();
	return frame.decoder;
}

void Receiver::_processAudio(const StreamPacket &spkt, const Packet &pkt) {
	// Audio Data
	InternalAudioStates &state = _getAudioFrame(spkt);

	state.timestamp = spkt.timestamp;

	auto &build = builder(spkt.streamID);
	auto fs = build.get(spkt.timestamp, spkt.frame_number+pkt.frame_count-1);

	if (fs) {
		auto &frame = fs->frames[spkt.frame_number];

		auto &audiolist = frame.createChange<std::list<ftl::audio::Audio>>(spkt.channel, build.changeType(), pkt);
		auto &audio = audiolist.emplace_back();

		ftl::audio::Decoder *dec = _createAudioDecoder(state, pkt);
		if (!dec) {
			LOG(ERROR) << "Could get an audio decoder";
			return;
		}
		if (!dec->decode(pkt, audio.data())) {
			LOG(ERROR) << "Audio decode failed";
			return;
		}

		fs->localTimestamp = spkt.localTimestamp;
		_finishPacket(fs, spkt.frame_number);
	} else {
		LOG(WARNING) << "Audio data being lost";
	}
}

namespace sgm {
	namespace details {
		void median_filter(const uint16_t* d_src, uint16_t* d_dst, int width, int height, int pitch, cudaStream_t stream);
	}
}

void Receiver::_processVideo(const StreamPacket &spkt, const Packet &pkt) {
	FTL_Profile("VideoPacket", 0.02);

	const unsigned int channum = (unsigned int)spkt.channel;
	InternalVideoStates &ividstate = _getVideoFrame(spkt);

	auto [tx,ty] = ftl::codecs::chooseTileConfig(pkt.frame_count);

	// Get the frameset
	auto &build = builder(spkt.streamID);
	auto fs = build.get(spkt.timestamp, spkt.frame_number+pkt.frame_count-1);

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

	auto &surface = ividstate.surface[static_cast<int>(spkt.channel)];

	// Allocate a decode surface, this is a tiled image to be split later
	int cvtype = ftl::codecs::type(spkt.channel);
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

	if (!fs) {
		LOG(WARNING) << "Dropping a video frame";
		return;
	}

	auto cvstream = cv::cuda::StreamAccessor::wrapStream(decoder->stream());

	// Mark a frameset as being partial
	if (pkt.flags & ftl::codecs::kFlagPartial) {
		fs->markPartial();
	}

	// Now split the tiles from surface into frames, doing colour conversions
	// at the same time.
	// Note: Done in reverse to allocate correct number of frames first time round
	// FIXME: Don't do this copy for single tiles
	for (int i=pkt.frame_count-1; i>=0; --i) {
		//InternalVideoStates &vidstate = _getVideoFrame(spkt,i);
		auto &frame = fs->frames[spkt.frame_number+i];

		//if (!frame.origin()) frame.setOrigin(&vidstate.state);

		if (frame.hasChannel(spkt.channel)) {
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

	fs->localTimestamp = spkt.localTimestamp;

	_finishPacket(fs, spkt.frame_number);
}

void Receiver::_finishPacket(ftl::streams::LockedFrameSet &fs, size_t fix) {
	if (fix >= fs->frames.size()) fix = 0;

	auto &frame = fs->frames[fix];
	++frame.packet_rx;

	if (frame.packet_tx > 0 && frame.packet_tx == frame.packet_rx) {
		fs->completed(fix);
		if (fs->isComplete()) {
			//LOG(INFO) << "COMPLETE: " << fs->timestamp() << ", " << fix;
			timestamp_ = fs->timestamp();
		}
		frame.packet_tx = 0;
		frame.packet_rx = 0;
	}
}

void Receiver::processPackets(const StreamPacket &spkt, const Packet &pkt) {
	const unsigned int channum = (unsigned int)spkt.channel;

	if (spkt.channel == Channel::EndFrame) {
		auto fs = builder(spkt.streamID).get(spkt.timestamp, spkt.frame_number+pkt.frame_count-1);

		if (fs) {
			fs->frames[spkt.frame_number].packet_tx = static_cast<int>(pkt.packet_count);
			//LOG(INFO) << "EXPECTED " << fs->frames[spkt.frame_number].packet_tx << " for " << int(spkt.frame_number);
			_finishPacket(fs, spkt.frame_number);
		}
		return;
	}

	// No data packet means channel is only available.
	if (pkt.data.size() == 0) {
		if (spkt.streamID < 255 && !(spkt.flags & ftl::codecs::kFlagRequest)) {
			// Get the frameset
			auto fs = builder(spkt.streamID).get(spkt.timestamp, spkt.frame_number+pkt.frame_count-1);

			if (fs) {
				const auto *cs = stream_;
				const auto sel = stream_->selected(spkt.frameSetID()) & cs->available(spkt.frameSetID());

				fs->localTimestamp = spkt.localTimestamp;

				for (auto &frame : fs->frames) {
					frame.markAvailable(spkt.channel);
				}
				_finishPacket(fs, spkt.frame_number);
			}
		}
		return;
	}

	if (spkt.frameSetID() >= ftl::stream::kMaxStreams) return;

	// Frameset level data channels
	if (spkt.frameNumber() == 255 && pkt.data.size() > 0) {
		_processData(spkt,pkt);
		return;
	}

	// Too many frames, so ignore.
	if (spkt.frameNumber() >= 32) return;


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
	return callback_.on(cb);
}

