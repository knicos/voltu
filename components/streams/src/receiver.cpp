#include <ftl/streams/receiver.hpp>

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

}

/*void Receiver::_processConfig(InternalStates &frame, const ftl::codecs::Packet &pkt) {
	std::string cfg;
	auto unpacked = msgpack::unpack((const char*)pkt.data.data(), pkt.data.size());
	unpacked.get().convert(cfg);

	LOG(INFO) << "Config Received: " << cfg;
	// TODO: This needs to be put in safer / better location
	//host_->set(std::get<0>(cfg), nlohmann::json::parse(std::get<1>(cfg)));
}*/

void Receiver::_createDecoder(InternalStates &frame, int chan, const ftl::codecs::Packet &pkt) {
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

Receiver::InternalStates::InternalStates() {
	for (int i=0; i<32; ++i) decoders[i] = nullptr;
}

Receiver::InternalStates &Receiver::_getFrame(const StreamPacket &spkt) {
	UNIQUE_LOCK(mutex_, lk);
	while (frames_.size() <= spkt.frameNumber()) {
		//frames_.resize(spkt.frameNumber()+1);
		frames_.push_back(new InternalStates);
		frames_[frames_.size()-1]->state.set("name",std::string("Source ")+std::to_string(spkt.frameNumber()+1));
	}
	auto &f = *frames_[spkt.frameNumber()];
	if (!f.frame.origin()) f.frame.setOrigin(&f.state);
	return f;
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

		//LOG(INFO) << "PACKET: " << spkt.timestamp << ", " << (int)spkt.channel << ", " << (int)pkt.codec;

		// TODO: Allow for multiple framesets
		if (spkt.frameSetID() > 0) return;

		// Too many frames, so ignore.
		if (spkt.frameNumber() >= value("max_frames",32)) return;

		// Dummy no data packet.
		if (pkt.data.size() == 0) return;

		InternalStates &frame = _getFrame(spkt);

		//if (spkt.timestamp > frame.timestamp && !frame.completed.empty()) {
		//	LOG(WARNING) << "Next frame received";
			//return;
		//}

		// Deal with the special channels...
		switch (rchan) {
		case Channel::Configuration		: frame.state.getConfig() = nlohmann::json::parse(parseConfig(pkt)); return;
		case Channel::Calibration		: frame.state.getLeft() = parseCalibration(pkt); return;
		case Channel::Calibration2		: frame.state.getRight() = parseCalibration(pkt); return;
		case Channel::Pose				: frame.state.getPose() = parsePose(pkt); return;
		default: break;
		}

		// Packets are for unwanted channel.
		//if (rchan != Channel::Colour && rchan != chan) return;

		if (frame.frame.hasChannel(spkt.channel)) {
			// FIXME: Is this a corruption in recording or in playback?
			// Seems to occur in same place in ftl file, one channel is missing
			LOG(ERROR) << "Previous frame not complete: " << frame.timestamp;
			//LOG(ERROR) << " --- " << (string)spkt;
			UNIQUE_LOCK(frame.mutex, lk);
			frame.frame.reset();
			frame.completed.clear();
		}
		frame.timestamp = spkt.timestamp;

		// Add channel to frame and allocate memory if required
		const cv::Size size = cv::Size(ftl::codecs::getWidth(pkt.definition), ftl::codecs::getHeight(pkt.definition));
		frame.frame.create<cv::cuda::GpuMat>(spkt.channel).create(size, (isFloatChannel(rchan) ? CV_32FC1 : CV_8UC4));

		Packet tmppkt = pkt;
		frame.frame.pushPacket(spkt.channel, tmppkt);

		//LOG(INFO) << " CODEC = " << (int)pkt.codec << " " << (int)pkt.flags << " " << (int)spkt.channel;

		// Do the actual decode
		_createDecoder(frame, channum, pkt);
		auto *decoder = frame.decoders[channum];
		if (!decoder) {
			LOG(ERROR) << "No frame decoder available";
			return;
		}

		try {
			//LOG(INFO) << "TYPE = " << frame.frame.get<cv::cuda::GpuMat>(spkt.channel).type();
			decoder->decode(pkt, frame.frame.get<cv::cuda::GpuMat>(spkt.channel));
		} catch (std::exception &e) {
			LOG(ERROR) << "Decode failed for " << spkt.timestamp << ": " << e.what();
		}

		frame.completed += spkt.channel;
		
		// Complete if all requested frames are found
		auto sel = stream_->selected(spkt.frameSetID());

		if ((frame.completed & sel) == sel) {
			UNIQUE_LOCK(frame.mutex, lk);
			if ((frame.completed & sel) == sel) {
				timestamp_ = frame.timestamp;

				//LOG(INFO) << "BUILDER PUSH: " << timestamp_ << ", " << spkt.frameNumber();

				if (frame.state.getLeft().width == 0) {
					LOG(WARNING) << "Missing calibration, skipping frame";
					//frame.frame.reset();
					//frame.completed.clear();
					//return;
				}

				// TODO: Have multiple builders for different framesets.
				builder_.push(frame.timestamp, spkt.frameNumber(), frame.frame);

				// Check for any state changes and send them back
				if (frame.state.hasChanged(Channel::Pose)) injectPose(stream_, frame.frame, frame.timestamp, spkt.frameNumber());
				if (frame.state.hasChanged(Channel::Calibration)) injectCalibration(stream_, frame.frame, frame.timestamp, spkt.frameNumber());
				if (frame.state.hasChanged(Channel::Calibration2)) injectCalibration(stream_, frame.frame, frame.timestamp, spkt.frameNumber(), true);

				frame.frame.reset();
				frame.completed.clear();
			}
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
