#include <ftl/streams/sender.hpp>

#include "injectors.hpp"

using ftl::stream::Sender;
using ftl::codecs::StreamPacket;
using ftl::codecs::Packet;
using ftl::codecs::Channels;
using ftl::codecs::Channel;
using ftl::codecs::definition_t;
using ftl::codecs::device_t;
using ftl::codecs::codec_t;
using ftl::stream::injectCalibration;
using ftl::stream::injectPose;
using ftl::stream::injectConfig;

Sender::Sender(nlohmann::json &config) : ftl::Configurable(config), stream_(nullptr) {
	//do_inject_ = false;
}

Sender::~Sender() {
    // Delete all encoders
}

/*void Sender::onStateChange(const std::function<void(ftl::codecs::Channel,const ftl::rgbd::FrameState&)> &cb) {
	if (cb && state_cb_) throw ftl::exception("State change callback already set");
	state_cb_ = cb;
}*/

void Sender::setStream(ftl::stream::Stream*s) {
	if (stream_) stream_->onPacket(nullptr);
    stream_ = s;
	stream_->onPacket([this](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		LOG(INFO) << "SENDER REQUEST : " << (int)spkt.channel;

		//if (state_cb_) state_cb_(spkt.channel, spkt.streamID, spkt.frame_number);

		// Inject state packets
		//do_inject_ = true;
		do_inject_.clear();
	});
}

template <typename T>
static void writeValue(std::vector<unsigned char> &data, T value) {
	unsigned char *pvalue_start = (unsigned char*)&value;
	data.insert(data.end(), pvalue_start, pvalue_start+sizeof(T));
}

static void mergeNALUnits(const std::list<ftl::codecs::Packet> &pkts, ftl::codecs::Packet &pkt) {
	size_t size = 0;
	for (auto i=pkts.begin(); i!=pkts.end(); ++i) size += (*i).data.size();

	// TODO: Check Codec, jpg etc can just use last frame.
	// TODO: Also if iframe, can just use that instead

	const auto &first = pkts.front();
	pkt.codec = first.codec;
	pkt.definition = first.definition;
	pkt.frame_count = first.frame_count;
	pkt.bitrate = first.bitrate;
	pkt.flags = first.flags | ftl::codecs::kFlagMultiple;  // means merged packets
	pkt.data.reserve(size+pkts.size()*sizeof(int));

	for (auto i=pkts.begin(); i!=pkts.end(); ++i) {
		writeValue<int>(pkt.data, (*i).data.size());
		//LOG(INFO) << "NAL Count = " << (*i).data.size();
		pkt.data.insert(pkt.data.end(), (*i).data.begin(), (*i).data.end());
	}
}

void Sender::post(const ftl::rgbd::FrameSet &fs) {
    if (!stream_) return;

    Channels selected;
	if (stream_->size() > 0) selected = stream_->selected(0);

	Channels available;  // but not selected and actually sent.

	bool do_inject = !do_inject_.test_and_set();
	//do_inject_ = false;

    for (int i=0; i<fs.frames.size(); ++i) {
        const auto &frame = fs.frames[i];

		if (do_inject) {
			//LOG(INFO) << "Force inject calibration";
			injectCalibration(stream_, fs, i);
			injectCalibration(stream_, fs, i, true);
			injectPose(stream_, fs, i);
			injectConfig(stream_, fs, i);
		} else {
			if (frame.hasChanged(Channel::Pose)) injectPose(stream_, fs, i);
			if (frame.hasChanged(Channel::Calibration)) injectCalibration(stream_, fs, i);
			if (frame.hasChanged(Channel::Calibration2)) injectCalibration(stream_, fs, i, true);
			if (frame.hasChanged(Channel::Configuration)) injectConfig(stream_, fs, i);
		}

        for (auto c : frame.getChannels()) {
			if (selected.has(c)) {
				// FIXME: Sends high res colour, but receive end currently broken
				//auto cc = (c == Channel::Colour && frame.hasChannel(Channel::ColourHighRes)) ? Channel::ColourHighRes : Channel::Colour;
				auto cc = c;

				StreamPacket spkt;
				spkt.version = 4;
				spkt.timestamp = fs.timestamp;
				spkt.streamID = 0; //fs.id;
				spkt.frame_number = i;
				spkt.channel = c;

				// Check if there are existing encoded packets
				const auto &packets = frame.getPackets(cc);
				if (packets.size() > 0) {
					if (packets.size() > 1) {
						LOG(WARNING) << "Multi-packet send";
						ftl::codecs::Packet pkt;
						mergeNALUnits(packets, pkt);
						stream_->post(spkt, pkt);
					} else {
						// Send existing encoding instead of re-encoding
						//for (auto &pkt : packets) {
						stream_->post(spkt, packets.front());
						//}
					}
				} else  {
					auto *enc = _getEncoder(fs.id, i, cc);

					if (enc) {
						// FIXME: Timestamps may not always be aligned to interval.
						//if (do_inject || fs.timestamp % (10*ftl::timer::getInterval()) == 0) enc->reset();
						if (do_inject) enc->reset();
						try {
							enc->encode(frame.get<cv::cuda::GpuMat>(cc), 0, [this,&spkt](const ftl::codecs::Packet &pkt){
								stream_->post(spkt, pkt);
							});
						} catch (std::exception &e) {
							LOG(ERROR) << "Exception in encoder: " << e.what();
						}
					} else {
						LOG(ERROR) << "No encoder";
					}
				}
			} else {
				available += c;
			}
        }
    }

	for (auto c : available) {
		// Not selected so send an empty packet...
		StreamPacket spkt;
		spkt.version = 4;
		spkt.timestamp = fs.timestamp;
		spkt.streamID = 0; // FIXME: fs.id;
		spkt.frame_number = 255;
		spkt.channel = c;

		Packet pkt;
		pkt.codec = codec_t::Any;
		pkt.definition = definition_t::Any;
		pkt.frame_count = 1;
		pkt.bitrate = 0;
		stream_->post(spkt, pkt);
	}

	//do_inject_ = false;
}

ftl::codecs::Encoder *Sender::_getEncoder(int fsid, int fid, Channel c) {
	int id = (fsid << 16) | (fid << 8) | (int)c;

	{
		SHARED_LOCK(mutex_, lk);
		auto i = encoders_.find(id);
		if (i != encoders_.end()) {
			return (*i).second;
		}
	}

	auto *enc = ftl::codecs::allocateEncoder(
					definition_t::HD1080, device_t::Any, codec_t::Any);
	UNIQUE_LOCK(mutex_, lk);
	encoders_[id] = enc;
	return enc;
}
