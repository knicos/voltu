#include <ftl/streams/receiver.hpp>
#include <ftl/codecs/depth_convert_cuda.hpp>
#include <ftl/profiler.hpp>

#include <opencv2/cudaimgproc.hpp>

#include "parsers.hpp"
#include "injectors.hpp"

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
using ftl::codecs::definition_t;

Receiver::Receiver(nlohmann::json &config) : ftl::Configurable(config), stream_(nullptr) {
	timestamp_ = 0;
	second_channel_ = Channel::Depth;

	size_t bsize = value("frameset_buffer_size", 3);
	for (int i=0; i<ftl::stream::kMaxStreams; ++i) {
		builder_[i].setID(i);
		builder_[i].setBufferSize(bsize);
	}

	on("frameset_buffer_size", [this](const ftl::config::Event &e) {
		size_t bsize = value("frameset_buffer_size", 3);
		for (int i=0; i<ftl::stream::kMaxStreams; ++i) {
			builder_[i].setBufferSize(bsize);
		}
	});
}

Receiver::~Receiver() {
	//if (stream_) {
	//	stream_->onPacket(nullptr);
	//}

	builder_[0].onFrameSet(nullptr);
}

void Receiver::onAudio(const ftl::audio::FrameSet::Callback &cb) {
	audio_cb_ = cb;
}

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
		video_frames_[spkt.streamID][video_frames_[spkt.streamID].size()-1]->state.set("name",std::string("Source ")+std::to_string(fn+1));
	}
	auto &f = *video_frames_[spkt.streamID][fn];
	if (!f.frame.origin()) f.frame.setOrigin(&f.state);
	return f;
}

Receiver::InternalAudioStates::InternalAudioStates() {
	
}

Receiver::InternalAudioStates &Receiver::_getAudioFrame(const StreamPacket &spkt, int ix) {
	uint32_t fn = spkt.frameNumber()+ix;

	UNIQUE_LOCK(mutex_, lk);
	while (audio_frames_[spkt.streamID].size() <= fn) {
		//frames_.resize(spkt.frameNumber()+1);
		audio_frames_[spkt.streamID].push_back(new InternalAudioStates);
		audio_frames_[spkt.streamID][audio_frames_[spkt.streamID].size()-1]->state.set("name",std::string("Source ")+std::to_string(fn+1));
	}
	auto &f = *audio_frames_[spkt.streamID][fn];
	//if (!f.frame.origin()) f.frame.setOrigin(&f.state);
	return f;
}

void Receiver::_processState(const StreamPacket &spkt, const Packet &pkt) {
	for (int i=0; i<pkt.frame_count; ++i) {
		InternalVideoStates &frame = _getVideoFrame(spkt,i);

		// Deal with the special channels...
		switch (spkt.channel) {
		case Channel::Configuration		: ftl::config::parseJSON(frame.state.getConfig(), parseConfig(pkt)); break;
		case Channel::Calibration		: frame.state.getLeft() = parseCalibration(pkt); break;
		case Channel::Calibration2		: frame.state.getRight() = parseCalibration(pkt); break;
		//case Channel::Pose				: frame.state.getPose() = parsePose(pkt); break;
		case Channel::Pose				: frame.state.setPose(parsePose(pkt)); break;
		default: break;
		}
	}
}

void Receiver::_processData(const StreamPacket &spkt, const Packet &pkt) {
	//InternalVideoStates &frame = _getVideoFrame(spkt);
	if (spkt.frameNumber() == 255) {
		auto *fs = builder_[spkt.streamID].get(spkt.timestamp);
		if (fs) {
			fs->createRawData(spkt.channel, pkt.data);
		}
	} else {
		auto &frame = builder_[spkt.streamID].get(spkt.timestamp, spkt.frame_number);
		frame.createRawData(spkt.channel, pkt.data);
	}
}

void Receiver::_processAudio(const StreamPacket &spkt, const Packet &pkt) {
	// Audio Data
	InternalAudioStates &frame = _getAudioFrame(spkt);

	frame.frame.reset();
	frame.timestamp = spkt.timestamp;
	auto &audio = frame.frame.create<ftl::audio::Audio>(spkt.channel);
	size_t size = pkt.data.size()/sizeof(short);
	audio.data().resize(size);
	auto *ptr = (short*)pkt.data.data();
	for (size_t i=0; i<size; i++) audio.data()[i] = ptr[i];

	// Generate settings from packet data
	ftl::audio::AudioSettings settings;
	settings.channels = (spkt.channel == Channel::AudioStereo) ? 2 : 1;
	settings.frame_size = 256;
	
	switch (pkt.definition) {
	case definition_t::hz48000		: settings.sample_rate = 48000; break;
	case definition_t::hz44100		: settings.sample_rate = 44100; break;
	default: settings.sample_rate = 48000; break;
	}

	frame.state.setLeft(settings);
	frame.frame.setOrigin(&frame.state);

	if (audio_cb_) {
		// Create an audio frameset wrapper.
		ftl::audio::FrameSet fs;
		fs.id = 0;
		fs.timestamp = frame.timestamp;
		fs.originClockDelta;
		fs.count = 1;
		//fs.stale = false;
		fs.clear(ftl::data::FSFlag::STALE);
		frame.frame.swapTo(fs.frames.emplace_back());

		audio_cb_(fs);
	}
}

void Receiver::_processVideo(const StreamPacket &spkt, const Packet &pkt) {
	FTL_Profile("VideoPacket", 0.02);

	const ftl::codecs::Channel rchan = spkt.channel;
	const unsigned int channum = (unsigned int)spkt.channel;
	InternalVideoStates &ividstate = _getVideoFrame(spkt);

	auto [tx,ty] = ftl::codecs::chooseTileConfig(pkt.frame_count);
	int width = ftl::codecs::getWidth(pkt.definition);
	int height = ftl::codecs::getHeight(pkt.definition);

	//LOG(INFO) << " CODEC = " << (int)pkt.codec << " " << (int)pkt.flags << " " << (int)spkt.channel;
	//LOG(INFO) << "Decode surface: " << (width*tx) << "x" << (height*ty);

	auto &surface = ividstate.surface[static_cast<int>(spkt.channel)];

	// Allocate a decode surface, this is a tiled image to be split later
	surface.create(height*ty, width*tx, ((isFloatChannel(spkt.channel)) ? ((pkt.flags & 0x2) ? CV_16UC4 : CV_16U) : CV_8UC4));

	// Find or create the decoder
	_createDecoder(ividstate, channum, pkt);
	auto *decoder = ividstate.decoders[channum];
	if (!decoder) {
		LOG(ERROR) << "No frame decoder available";
		return;
	}

	// Do the actual decode into the surface buffer
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

	auto cvstream = cv::cuda::StreamAccessor::wrapStream(decoder->stream());

	/*if (spkt.channel == Channel::Depth && (pkt.flags & 0x2)) {
	cv::Mat tmp;
	surface.download(tmp);
	cv::imshow("Test", tmp);
	cv::waitKey(1);
	}*/

	bool apply_Y_filter = value("apply_Y_filter", true);

	// Mark a frameset as being partial
	if (pkt.flags & ftl::codecs::kFlagPartial) {
		builder_[spkt.streamID].markPartial(spkt.timestamp);
	}

	// Now split the tiles from surface into frames, doing colour conversions
	// at the same time.
	// Note: Done in reverse to allocate correct number of frames first time round
	for (int i=pkt.frame_count-1; i>=0; --i) {
		InternalVideoStates &vidstate = _getVideoFrame(spkt,i);
		auto &frame = builder_[spkt.streamID].get(spkt.timestamp, spkt.frame_number+i);

		if (!frame.origin()) frame.setOrigin(&vidstate.state);

		if (frame.hasChannel(spkt.channel)) {
			// FIXME: Is this a corruption in recording or in playback?
			// Seems to occur in same place in ftl file, one channel is missing
			LOG(WARNING) << "Previous frame not complete: " << spkt.timestamp;
		}

		{
			// This ensures that if previous frames are unfinished then they
			// are discarded.
			/*UNIQUE_LOCK(vidstate.mutex, lk);
			if (frame.timestamp != spkt.timestamp && frame.timestamp != -1) {
				frame.frame.reset();
				frame.completed.clear();
				LOG(WARNING) << "Frames out-of-phase by: " << spkt.timestamp - frame.timestamp;
			}
			frame.timestamp = spkt.timestamp;*/
		}

		// Add channel to frame and allocate memory if required
		const cv::Size size = cv::Size(width, height);
		frame.getBuffer<cv::cuda::GpuMat>(spkt.channel).create(size, (isFloatChannel(rchan) ? CV_32FC1 : CV_8UC4));

		cv::Rect roi((i % tx)*width, (i / tx)*height, width, height);
		cv::cuda::GpuMat sroi = surface(roi);
		
		// Do colour conversion
		if (isFloatChannel(rchan) && (pkt.flags & 0x2)) {
			// Smooth Y channel around discontinuities
			// Lerp the uv channels / smooth over a small kernal size.

			if (value("apply_median", false)) {
				cv::Mat tmp;
				sroi.download(tmp);
				cv::medianBlur(tmp, tmp, 5);
				sroi.upload(tmp);
			}

			if (apply_Y_filter) ftl::cuda::smooth_y(sroi, cvstream);
			ftl::cuda::vuya_to_depth(frame.getBuffer<cv::cuda::GpuMat>(spkt.channel), sroi, 16.0f, cvstream);
		} else if (isFloatChannel(rchan)) {
			sroi.convertTo(frame.getBuffer<cv::cuda::GpuMat>(spkt.channel), CV_32FC1, 1.0f/1000.0f, cvstream);
		} else {
			cv::cuda::cvtColor(sroi, frame.getBuffer<cv::cuda::GpuMat>(spkt.channel), cv::COLOR_RGBA2BGRA, 0, cvstream);
		}
	}

	// Must ensure all processing is finished before completing a frame.
	cudaSafeCall(cudaStreamSynchronize(decoder->stream()));

	for (int i=0; i<pkt.frame_count; ++i) {
		InternalVideoStates &vidstate = _getVideoFrame(spkt,i);
		auto &frame = builder_[spkt.streamID].get(spkt.timestamp, spkt.frame_number+i);

		const auto *cs = stream_;
		auto sel = stream_->selected(spkt.frameSetID()) & cs->available(spkt.frameSetID());

		frame.create<cv::cuda::GpuMat>(spkt.channel);

		if (i == 0) {
			Packet tmppkt = pkt;
			frame.pushPacket(spkt.channel, tmppkt);
		}
		
		UNIQUE_LOCK(vidstate.mutex, lk);
		//if (frame.timestamp == spkt.timestamp) {
			//frame.completed += spkt.channel;
			
			// Complete if all requested channels are found
			if ((frame.getChannels() & sel) == sel) {
				timestamp_ = spkt.timestamp;
				//frame.reset.clear();

				//LOG(INFO) << "BUILDER PUSH: " << timestamp_ << ", " << spkt.frameNumber() << ", " << (int)pkt.frame_count;

				if (vidstate.state.getLeft().width == 0) {
					LOG(WARNING) << "Missing calibration for frame";
				}

				// TODO: Have multiple builders for different framesets.
				//builder_.push(frame.timestamp, spkt.frameNumber()+i, frame.frame);
				builder_[spkt.streamID].completed(spkt.timestamp, spkt.frame_number+i);

				// Check for any state changes and send them back
				//if (vidstate.state.hasChanged(Channel::Pose)) injectPose(stream_, frame, spkt.timestamp, spkt.frameNumber()+i);
				//if (vidstate.state.hasChanged(Channel::Calibration)) injectCalibration(stream_, frame, spkt.timestamp, spkt.streamID, spkt.frameNumber()+i);
				//if (vidstate.state.hasChanged(Channel::Calibration2)) injectCalibration(stream_, frame, spkt.timestamp, spkt.streamID, spkt.frameNumber()+i, true);

				//frame.reset();
				//frame.completed.clear();
				//frame.timestamp = -1;
			}
		//} else {
		//	LOG(ERROR) << "Frame timestamps mistmatch";
		//}
	}
}

void Receiver::setStream(ftl::stream::Stream *s) {
	if (stream_) {
		stream_->onPacket(nullptr);
	}

	stream_ = s;

	s->onPacket([this](const StreamPacket &spkt, const Packet &pkt) {	
		const unsigned int channum = (unsigned int)spkt.channel;

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
		if (spkt.frameNumber() >= value("max_frames",32)) return;

		// Dummy no data packet.
		if (pkt.data.size() == 0) return;


		if (channum >= 2048) {
			_processData(spkt,pkt);
		} else if (channum >= 64) {
			_processState(spkt,pkt);
		} else if (channum >= 32 && channum < 64) {
			_processAudio(spkt,pkt);
		} else {
			_processVideo(spkt,pkt);
		}
	});
}

size_t Receiver::size() {
	return builder_[0].size();
}

ftl::rgbd::FrameState &Receiver::state(size_t ix) {
	return builder_[0].state(ix);
}

void Receiver::onFrameSet(const ftl::rgbd::VideoCallback &cb) {
	for (int i=0; i<ftl::stream::kMaxStreams; ++i)
		builder_[i].onFrameSet(cb);
}

void Receiver::onFrameSet(int s, const ftl::rgbd::VideoCallback &cb) {
	if (s >= 0 && s < ftl::stream::kMaxStreams)
		builder_[s].onFrameSet(cb);
}
