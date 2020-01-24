#include <ftl/streams/receiver.hpp>
#include <ftl/codecs/depth_convert_cuda.hpp>

#include "parsers.hpp"
#include "injectors.hpp"

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

Receiver::Receiver(nlohmann::json &config) : ftl::Configurable(config), stream_(nullptr) {
	timestamp_ = 0;
	second_channel_ = Channel::Depth;
}

Receiver::~Receiver() {
	//if (stream_) {
	//	stream_->onPacket(nullptr);
	//}

	builder_.onFrameSet(nullptr);
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
	//UNIQUE_LOCK(mutex_,lk);
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
}

Receiver::InternalVideoStates &Receiver::_getVideoFrame(const StreamPacket &spkt, int ix) {
	int fn = spkt.frameNumber()+ix;

	UNIQUE_LOCK(mutex_, lk);
	while (video_frames_.size() <= fn) {
		//frames_.resize(spkt.frameNumber()+1);
		video_frames_.push_back(new InternalVideoStates);
		video_frames_[video_frames_.size()-1]->state.set("name",std::string("Source ")+std::to_string(fn+1));
	}
	auto &f = *video_frames_[fn];
	if (!f.frame.origin()) f.frame.setOrigin(&f.state);
	return f;
}

Receiver::InternalAudioStates::InternalAudioStates() {
	
}

Receiver::InternalAudioStates &Receiver::_getAudioFrame(const StreamPacket &spkt, int ix) {
	int fn = spkt.frameNumber()+ix;

	UNIQUE_LOCK(mutex_, lk);
	while (audio_frames_.size() <= fn) {
		//frames_.resize(spkt.frameNumber()+1);
		audio_frames_.push_back(new InternalAudioStates);
		audio_frames_[audio_frames_.size()-1]->state.set("name",std::string("Source ")+std::to_string(fn+1));
	}
	auto &f = *audio_frames_[fn];
	if (!f.frame.origin()) f.frame.setOrigin(&f.state);
	return f;
}

void Receiver::_processState(const StreamPacket &spkt, const Packet &pkt) {
	for (int i=0; i<pkt.frame_count; ++i) {
		InternalVideoStates &frame = _getVideoFrame(spkt,i);

		// Deal with the special channels...
		switch (spkt.channel) {
		case Channel::Configuration		: frame.state.getConfig() = nlohmann::json::parse(parseConfig(pkt)); break;
		case Channel::Calibration		: frame.state.getLeft() = parseCalibration(pkt); break;
		case Channel::Calibration2		: frame.state.getRight() = parseCalibration(pkt); break;
		case Channel::Pose				: frame.state.getPose() = parsePose(pkt); break;
		default: break;
		}
	}
}

void Receiver::_processData(const StreamPacket &spkt, const Packet &pkt) {
	InternalVideoStates &frame = _getVideoFrame(spkt);
	frame.frame.createRawData(spkt.channel, pkt.data);
}

void Receiver::_processAudio(const StreamPacket &spkt, const Packet &pkt) {
	// Audio Data
	InternalAudioStates &frame = _getAudioFrame(spkt);

	frame.timestamp = spkt.timestamp;
	auto &audio = frame.frame.create<ftl::audio::Audio>(spkt.channel);
	size_t size = pkt.data.size()/sizeof(short);
	audio.data().resize(size);
	auto *ptr = (short*)pkt.data.data();
	for (int i=0; i<size; i++) audio.data()[i] = ptr[i];

	if (audio_cb_) {
		// Create an audio frameset wrapper.
		ftl::audio::FrameSet fs;
		fs.id = 0;
		fs.timestamp = frame.timestamp;
		fs.count = 1;
		fs.stale = false;
		frame.frame.swapTo(fs.frames.emplace_back());

		audio_cb_(fs);
	}
}

void Receiver::_processVideo(const StreamPacket &spkt, const Packet &pkt) {
	//ftl::cuda::setDevice();
	//int dev;
	//cudaSafeCall(cudaGetDevice(&dev));
	//LOG(INFO) << "Cuda device = " << dev;

	const ftl::codecs::Channel rchan = spkt.channel;
	const unsigned int channum = (unsigned int)spkt.channel;
	InternalVideoStates &iframe = _getVideoFrame(spkt);

	auto [tx,ty] = ftl::codecs::chooseTileConfig(pkt.frame_count);
	int width = ftl::codecs::getWidth(pkt.definition);
	int height = ftl::codecs::getHeight(pkt.definition);

	for (int i=0; i<pkt.frame_count; ++i) {
		InternalVideoStates &frame = _getVideoFrame(spkt,i);

		// Packets are for unwanted channel.
		//if (rchan != Channel::Colour && rchan != chan) return;

		if (frame.frame.hasChannel(spkt.channel)) {
			UNIQUE_LOCK(frame.mutex, lk);
			//if (frame.frame.hasChannel(spkt.channel)) {
				// FIXME: Is this a corruption in recording or in playback?
				// Seems to occur in same place in ftl file, one channel is missing
				LOG(WARNING) << "Previous frame not complete: " << frame.timestamp;
				//LOG(ERROR) << " --- " << (string)spkt;
				//frame.frame.reset();
				//frame.completed.clear();
			//}
		}
		frame.timestamp = spkt.timestamp;

		// Add channel to frame and allocate memory if required
		const cv::Size size = cv::Size(width, height);
		frame.frame.create<cv::cuda::GpuMat>(spkt.channel).create(size, (isFloatChannel(rchan) ? CV_32FC1 : CV_8UC4));
	}

	Packet tmppkt = pkt;
	iframe.frame.pushPacket(spkt.channel, tmppkt);

	//LOG(INFO) << " CODEC = " << (int)pkt.codec << " " << (int)pkt.flags << " " << (int)spkt.channel;
	//LOG(INFO) << "Decode surface: " << (width*tx) << "x" << (height*ty);

	auto &surface = iframe.surface[static_cast<int>(spkt.channel)];

	surface.create(height*ty, width*tx, ((isFloatChannel(spkt.channel)) ? ((pkt.flags & 0x2) ? CV_16UC4 : CV_16U) : CV_8UC4));

	// Do the actual decode
	_createDecoder(iframe, channum, pkt);
	auto *decoder = iframe.decoders[channum];
	if (!decoder) {
		LOG(ERROR) << "No frame decoder available";
		return;
	}

	try {
		//LOG(INFO) << "TYPE = " << frame.frame.get<cv::cuda::GpuMat>(spkt.channel).type();
		if (!decoder->decode(pkt, surface)) {
			LOG(ERROR) << "Decode failed";
			//return;
		}
	} catch (std::exception &e) {
		LOG(ERROR) << "Decode failed for " << spkt.timestamp << ": " << e.what();
		//return;
	}

	auto cvstream = cv::cuda::StreamAccessor::wrapStream(decoder->stream());

	/*if (spkt.channel == Channel::Depth && (pkt.flags & 0x2)) {
	cv::Mat tmp;
	surface.download(tmp);
	cv::imshow("Test", tmp);
	cv::waitKey(1);
	}*/

	for (int i=0; i<pkt.frame_count; ++i) {
		InternalVideoStates &frame = _getVideoFrame(spkt,i);

		cv::Rect roi((i % tx)*width, (i / tx)*height, width, height);
		cv::cuda::GpuMat sroi = surface(roi);
		
		// Do colour conversion
		if (isFloatChannel(rchan) && (pkt.flags & 0x2)) {
			//LOG(INFO) << "VUYA Convert";
			ftl::cuda::vuya_to_depth(frame.frame.get<cv::cuda::GpuMat>(spkt.channel), sroi, 16.0f, cvstream);
		} else if (isFloatChannel(rchan)) {
			sroi.convertTo(frame.frame.get<cv::cuda::GpuMat>(spkt.channel), CV_32FC1, 1.0f/1000.0f, cvstream);
		} else {
			cv::cuda::cvtColor(sroi, frame.frame.get<cv::cuda::GpuMat>(spkt.channel), cv::COLOR_RGBA2BGRA, 0, cvstream);
		}
	}

	cudaSafeCall(cudaStreamSynchronize(decoder->stream()));

	for (int i=0; i<pkt.frame_count; ++i) {
		InternalVideoStates &frame = _getVideoFrame(spkt,i);
		
		frame.completed += spkt.channel;
	
		// Complete if all requested frames are found
		auto sel = stream_->selected(spkt.frameSetID());

		if ((frame.completed & sel) == sel) {
			UNIQUE_LOCK(frame.mutex, lk);
			if ((frame.completed & sel) == sel) {
				timestamp_ = frame.timestamp;

				//LOG(INFO) << "BUILDER PUSH: " << timestamp_ << ", " << spkt.frameNumber() << ", " << (int)pkt.frame_count;

				if (frame.state.getLeft().width == 0) {
					LOG(WARNING) << "Missing calibration, skipping frame";
					//frame.frame.reset();
					//frame.completed.clear();
					//return;
				}

				// TODO: Have multiple builders for different framesets.
				builder_.push(frame.timestamp, spkt.frameNumber()+i, frame.frame);

				// Check for any state changes and send them back
				if (frame.state.hasChanged(Channel::Pose)) injectPose(stream_, frame.frame, frame.timestamp, spkt.frameNumber()+i);
				if (frame.state.hasChanged(Channel::Calibration)) injectCalibration(stream_, frame.frame, frame.timestamp, spkt.frameNumber()+i);
				if (frame.state.hasChanged(Channel::Calibration2)) injectCalibration(stream_, frame.frame, frame.timestamp, spkt.frameNumber()+i, true);

				frame.frame.reset();
				frame.completed.clear();
			}
		}
	}
}

void Receiver::setStream(ftl::stream::Stream *s) {
	if (stream_) {
		stream_->onPacket(nullptr);
	}

	stream_ = s;

	s->onPacket([this](const StreamPacket &spkt, const Packet &pkt) {	
		//const ftl::codecs::Channel chan = second_channel_;
		const ftl::codecs::Channel rchan = spkt.channel;
		const unsigned int channum = (unsigned int)spkt.channel;

		//LOG(INFO) << "PACKET: " << spkt.timestamp << ", " << (int)spkt.channel << ", " << (int)pkt.codec << ", " << (int)pkt.definition;

		// TODO: Allow for multiple framesets
		if (spkt.frameSetID() > 0) return;

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
	return builder_.size();
}

ftl::rgbd::FrameState &Receiver::state(int ix) {
	return builder_.state(ix);
}

void Receiver::onFrameSet(const ftl::rgbd::VideoCallback &cb) {
	builder_.onFrameSet(cb);
}
