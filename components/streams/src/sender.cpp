#include <ftl/streams/sender.hpp>
#include <ftl/codecs/depth_convert_cuda.hpp>

#include "injectors.hpp"

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
}

Sender::~Sender() {
    // Delete all encoders
	for (auto c : state_) {
		if (c.second.encoder[0]) ftl::codecs::free(c.second.encoder[0]);
		if (c.second.encoder[1]) ftl::codecs::free(c.second.encoder[1]);
	}
}

void Sender::setStream(ftl::stream::Stream*s) {
	if (stream_) stream_->onPacket(nullptr);
    stream_ = s;
	stream_->onPacket([this](const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		//LOG(INFO) << "SENDER REQUEST : " << (int)spkt.channel;

		//if (state_cb_) state_cb_(spkt.channel, spkt.streamID, spkt.frame_number);
		if (reqcb_) reqcb_(spkt,pkt);

		// Inject state packets
		//do_inject_ = true;
		do_inject_.clear();
	});
}

void Sender::onRequest(const ftl::stream::StreamCallback &cb) {
	reqcb_ = cb;
}

void Sender::post(const ftl::audio::FrameSet &fs) {
    if (!stream_) return;

	//if (fs.stale) return;
	//fs.stale = true;

	for (size_t i=0; i<fs.frames.size(); ++i) {
		if (!fs.frames[i].hasChannel(Channel::Audio)) continue;

		auto &data = fs.frames[i].get<ftl::audio::Audio>(Channel::Audio);

		StreamPacket spkt;
		spkt.version = 4;
		spkt.timestamp = fs.timestamp;
		spkt.streamID = fs.id;
		spkt.frame_number = i;
		spkt.channel = Channel::Audio;

		ftl::codecs::Packet pkt;
		pkt.codec = ftl::codecs::codec_t::RAW;
		pkt.definition = ftl::codecs::definition_t::Any;
		pkt.frame_count = 1;
		pkt.flags = 0;
		pkt.bitrate = 0;

		const unsigned char *ptr = (unsigned char*)data.data().data();
		pkt.data = std::move(std::vector<unsigned char>(ptr, ptr+data.size()));  // TODO: Reduce copy...

		stream_->post(spkt, pkt);

		//LOG(INFO) << "SENT AUDIO: " << fs.timestamp << " - " << pkt.data.size();
	}
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
	Channels available;  // but not selected and actually sent.
	Channels needencoding;

	if (stream_->size() > 0) selected = stream_->selected(0);

	bool do_inject = !do_inject_.test_and_set();

    for (size_t i=0; i<fs.frames.size(); ++i) {
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
					needencoding += c;
				}
			} else {
				available += c;
			}
        }

		// FIXME: Allow data channel selection rather than always send
		for (auto c : frame.getDataChannels()) {
			StreamPacket spkt;
			spkt.version = 4;
			spkt.timestamp = fs.timestamp;
			spkt.streamID = 0; //fs.id;
			spkt.frame_number = i;
			spkt.channel = c;

			ftl::codecs::Packet pkt;
			pkt.codec = ftl::codecs::codec_t::MSGPACK;
			pkt.definition = ftl::codecs::definition_t::Any;
			pkt.frame_count = 1;
			pkt.flags = 0;
			pkt.bitrate = 0;
			pkt.data = frame.getRawData(c);
			stream_->post(spkt, pkt);
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

	for (auto c : needencoding) {
		// TODO: One thread per channel.
		_encodeChannel(fs, c, do_inject);
	}

	//do_inject_ = false;
}

void Sender::_encodeChannel(const ftl::rgbd::FrameSet &fs, Channel c, bool reset) {
	bool lossless = value("lossless", false);
	int max_bitrate = std::max(0, std::min(255, value("max_bitrate", 255)));
	//int min_bitrate = std::max(0, std::min(255, value("min_bitrate", 0)));  // TODO: Use this
	codec_t codec = value("codec", codec_t::Any);

	uint32_t offset = 0;
	while (offset < fs.frames.size()) {
		StreamPacket spkt;
		spkt.version = 4;
		spkt.timestamp = fs.timestamp;
		spkt.streamID = 0; // FIXME: fs.id;
		spkt.frame_number = offset;
		spkt.channel = c;

		auto &tile = _getTile(fs.id, c);

		ftl::codecs::Encoder *enc = tile.encoder[(offset==0)?0:1];
		if (!enc) {
			enc = ftl::codecs::allocateEncoder(
				definition_t::HD1080, device_t::Any, codec);
			tile.encoder[(offset==0)?0:1] = enc;
		}

		if (!enc) {
			LOG(ERROR) << "No encoder";
			return;
		}

		int count = _generateTiles(fs, offset, c, enc->stream(), lossless);
		if (count <= 0) {
			LOG(ERROR) << "Could not generate tiles.";
			break;
		}

		if (enc) {
			// FIXME: Timestamps may not always be aligned to interval.
			//if (do_inject || fs.timestamp % (10*ftl::timer::getInterval()) == 0) enc->reset();
			if (reset) enc->reset();

			try {
				ftl::codecs::Packet pkt;
				pkt.frame_count = count;
				pkt.codec = codec;
				pkt.definition = definition_t::Any;
				pkt.bitrate = max_bitrate;
				pkt.flags = 0;

				if (!lossless && ftl::codecs::isFloatChannel(c)) pkt.flags = ftl::codecs::kFlagFloat | ftl::codecs::kFlagMappedDepth;
				else if (lossless && ftl::codecs::isFloatChannel(c)) pkt.flags = ftl::codecs::kFlagFloat;
				else pkt.flags = ftl::codecs::kFlagFlipRGB;

				// Choose correct region of interest into the surface.
				cv::Rect roi = _generateROI(fs, c, offset);
				cv::cuda::GpuMat sroi = tile.surface(roi);

				if (enc->encode(sroi, pkt)) {
					stream_->post(spkt, pkt);

					/*cv::Mat tmp;
					tile.surface.download(tmp);
					cv::imshow("Test", tmp);
					cv::waitKey(1);*/
				} else {
					LOG(ERROR) << "Encoding failed";
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

cv::Rect Sender::_generateROI(const ftl::rgbd::FrameSet &fs, ftl::codecs::Channel c, int offset) {
	const ftl::rgbd::Frame *cframe = &fs.frames[offset];
	int rwidth = cframe->get<cv::cuda::GpuMat>(c).cols;
	int rheight = cframe->get<cv::cuda::GpuMat>(c).rows;
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

int Sender::_generateTiles(const ftl::rgbd::FrameSet &fs, int offset, Channel c, cv::cuda::Stream &stream, bool lossless) {
	auto &surface = _getTile(fs.id, c);

	const ftl::rgbd::Frame *cframe = &fs.frames[offset];
	auto &m = cframe->get<cv::cuda::GpuMat>(c);

	// Choose tile configuration and allocate memory
	auto [tx,ty] = ftl::codecs::chooseTileConfig(fs.frames.size());
	int rwidth = cframe->get<cv::cuda::GpuMat>(c).cols;
	int rheight = cframe->get<cv::cuda::GpuMat>(c).rows;
	int width = tx * rwidth;
	int height = ty * rheight;
	int tilecount = tx*ty;
	uint32_t count = 0;

	surface.surface.create(height, width, (lossless && m.type() == CV_32F) ? CV_16U : CV_8UC4);

	// Loop over tiles with ROI mats and do colour conversions.
	while (tilecount > 0 && count+offset < fs.frames.size()) {
		auto &m = cframe->get<cv::cuda::GpuMat>(c);
		cv::Rect roi((count % tx)*rwidth, (count / tx)*rheight, rwidth, rheight);
		cv::cuda::GpuMat sroi = surface.surface(roi);

		if (m.type() == CV_32F) {
			if (lossless) {
				m.convertTo(sroi, CV_16UC1, 1000, stream);
			} else {
				ftl::cuda::depth_to_vuya(m, sroi, _selectFloatMax(c), stream);
			}
		} else if (m.type() == CV_8UC4) {
			cv::cuda::cvtColor(m, sroi, cv::COLOR_BGRA2RGBA, 0, stream);
		} else {
			LOG(ERROR) << "Unsupported colour format";
			return 0;
		}

		++count;
		--tilecount;
		cframe = &fs.frames[offset+count];
	}

	return count;
}
