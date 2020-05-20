#include <ftl/streams/sender.hpp>
#include <ftl/codecs/depth_convert_cuda.hpp>
#include <ftl/profiler.hpp>

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
	iframe_ = 1;
	add_iframes_ = value("iframes", 0);

	on("iframes", [this](const ftl::config::Event &e) {
		add_iframes_ = value("iframes", 0);
	});
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
		if (!(fs.frames[i].hasChannel(Channel::AudioMono) || fs.frames[i].hasChannel(Channel::AudioStereo))) continue;

		auto &data = (fs.frames[i].hasChannel(Channel::AudioStereo)) ?
			fs.frames[i].get<ftl::audio::Audio>(Channel::AudioStereo) :
			fs.frames[i].get<ftl::audio::Audio>(Channel::AudioMono);

		auto &settings = fs.frames[i].getSettings();

		StreamPacket spkt;
		spkt.version = 4;
		spkt.timestamp = fs.timestamp;
		spkt.streamID = fs.id;
		spkt.frame_number = i;
		spkt.channel = (fs.frames[i].hasChannel(Channel::AudioStereo)) ? Channel::AudioStereo : Channel::AudioMono;

		ftl::codecs::Packet pkt;
		pkt.codec = ftl::codecs::codec_t::RAW;
		pkt.definition = ftl::codecs::definition_t::Any;

		switch (settings.sample_rate) {
		case 48000		: pkt.definition = ftl::codecs::definition_t::hz48000; break;
		case 44100		: pkt.definition = ftl::codecs::definition_t::hz44100; break;
		default: break;
		}

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

void Sender::post(ftl::rgbd::FrameSet &fs) {
	if (!stream_) return;

	Channels selected;
	Channels available;  // but not selected and actually sent.
	Channels needencoding;

	if (stream_->size() > 0) selected = stream_->selected(0);

	bool do_inject = !do_inject_.test_and_set();

	// Add an iframe at the requested frequency.
	if (add_iframes_ > 0) iframe_ = (iframe_+1) % add_iframes_;

	FTL_Profile("SenderPost", 0.02);

	// Send any frameset data channels
	for (auto c : fs.getDataChannels()) {
		StreamPacket spkt;
		spkt.version = 4;
		spkt.timestamp = fs.timestamp;
		spkt.streamID = 0; //fs.id;
		spkt.frame_number = 255;
		spkt.channel = c;

		ftl::codecs::Packet pkt;
		pkt.codec = ftl::codecs::codec_t::MSGPACK;
		pkt.definition = ftl::codecs::definition_t::Any;
		pkt.frame_count = 1;
		pkt.flags = 0;
		pkt.bitrate = 0;
		pkt.data = fs.getRawData(c);
		stream_->post(spkt, pkt);
	}

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

		for (auto c : frame.getChannels()) {
			if (selected.has(c)) {
				// FIXME: Sends high res colour, but receive end currently broken
				auto cc = (c == Channel::Colour && frame.hasChannel(Channel::ColourHighRes)) ? Channel::ColourHighRes : c;
				//auto cc = c;

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
						LOG(WARNING) << "Multi-packet send: " << (int)cc;
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
		_encodeChannel(fs, c, do_inject || iframe_ == 0);
	}

	//do_inject_ = false;
}

void Sender::_encodeChannel(ftl::rgbd::FrameSet &fs, Channel c, bool reset) {
	bool lossless = value("lossless", false);
	int max_bitrate = std::max(0, std::min(255, value("max_bitrate", 255)));
	//int min_bitrate = std::max(0, std::min(255, value("min_bitrate", 0)));  // TODO: Use this
	codec_t codec = static_cast<codec_t>(value("codec", static_cast<int>(codec_t::Any)));
	device_t device = static_cast<device_t>(value("encoder_device", static_cast<int>(device_t::Any)));

	// TODO: Support high res
	bool is_stereo = value("stereo", false) && c == Channel::Colour && fs.firstFrame().hasChannel(Channel::Colour2);

	uint32_t offset = 0;
	while (offset < fs.frames.size()) {
		Channel cc = c;
		if ((cc == Channel::Colour) && fs.firstFrame().hasChannel(Channel::ColourHighRes)) {
			cc = Channel::ColourHighRes;
		}
		
		if ((cc == Channel::Right) && fs.firstFrame().hasChannel(Channel::RightHighRes)) {
			cc = Channel::RightHighRes;
			fs.frames[offset].upload(cc);
		}

		StreamPacket spkt;
		spkt.version = 4;
		spkt.timestamp = fs.timestamp;
		spkt.streamID = 0; // FIXME: fs.id;
		spkt.frame_number = offset;
		spkt.channel = c;

		auto &tile = _getTile(fs.id, cc);

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

		// Upload if in host memory
		for (auto &f : fs.frames) {
			if (!fs.hasFrame(f.id)) continue;
			if (f.isCPU(c)) {
				f.upload(Channels<0>(cc), cv::cuda::StreamAccessor::getStream(enc->stream()));
			}
		}

		int count = _generateTiles(fs, offset, cc, enc->stream(), lossless, is_stereo);
		if (count <= 0) {
			LOG(ERROR) << "Could not generate tiles.";
			break;
		}

		//cudaSafeCall(cudaStreamSynchronize(enc->stream()));
		enc->stream().waitForCompletion();

		if (enc) {
			if (reset) enc->reset();

			try {
				ftl::codecs::Packet pkt;
				pkt.frame_count = count;
				pkt.codec = codec;
				pkt.definition = definition_t::Any;
				pkt.bitrate = (!lossless && ftl::codecs::isFloatChannel(cc)) ? max_bitrate : max_bitrate/2;
				pkt.flags = 0;

				if (!lossless && ftl::codecs::isFloatChannel(cc)) pkt.flags = ftl::codecs::kFlagFloat | ftl::codecs::kFlagMappedDepth;
				else if (lossless && ftl::codecs::isFloatChannel(cc)) pkt.flags = ftl::codecs::kFlagFloat;
				else pkt.flags = ftl::codecs::kFlagFlipRGB;
				if (is_stereo) pkt.flags |= ftl::codecs::kFlagStereo;

				// In the event of partial frames, add a flag to indicate that
				if (static_cast<size_t>(fs.count) < fs.frames.size()) pkt.flags |= ftl::codecs::kFlagPartial;

				// Choose correct region of interest into the surface.
				cv::Rect roi = _generateROI(fs, cc, offset, is_stereo);
				cv::cuda::GpuMat sroi = tile.surface(roi);

				FTL_Profile("Encoder",0.02);

				LOG(INFO) << "Enocode bitrate: " << (int)pkt.bitrate;

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

cv::Rect Sender::_generateROI(const ftl::rgbd::FrameSet &fs, ftl::codecs::Channel c, int offset, bool stereo) {
	const ftl::rgbd::Frame &cframe = fs.firstFrame();
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

int Sender::_generateTiles(const ftl::rgbd::FrameSet &fs, int offset, Channel c, cv::cuda::Stream &stream, bool lossless, bool stereo) {
	auto &surface = _getTile(fs.id, c);

	const ftl::rgbd::Frame *cframe = nullptr; //&fs.frames[offset];

	const auto &m = fs.firstFrame().get<cv::cuda::GpuMat>(c);

	// Choose tile configuration and allocate memory
	auto [tx,ty] = ftl::codecs::chooseTileConfig(fs.frames.size());
	int rwidth = (stereo) ? m.cols*2 : m.cols;
	int rheight = m.rows;
	int width = tx * rwidth;
	int height = ty * rheight;
	int tilecount = tx*ty;
	uint32_t count = 0;

	int cvtype = CV_8UC4;
	switch (m.type()) {
	case CV_32F		:	cvtype = (lossless && m.type() == CV_32F) ? CV_16U : CV_8UC4; break;
	case CV_8UC1	:	cvtype = CV_8UC1; break;
	default			:	cvtype = CV_8UC4;
	}

	surface.surface.create(height, width, cvtype);

	// Loop over tiles with ROI mats and do colour conversions.
	while (tilecount > 0 && count+offset < fs.frames.size()) {
		if (fs.hasFrame(offset+count)) {
			cframe = &fs.frames[offset+count];
			auto &m = cframe->get<cv::cuda::GpuMat>(c);
			cv::Rect roi((count % tx)*rwidth, (count / tx)*rheight, (stereo) ? rwidth/2 : rwidth, rheight);
			cv::cuda::GpuMat sroi = surface.surface(roi);

			if (m.type() == CV_32F) {
				if (lossless) {
					m.convertTo(sroi, CV_16UC1, 1000, stream);
				} else {
					ftl::cuda::depth_to_vuya(m, sroi, _selectFloatMax(c), stream);
				}
			} else if (m.type() == CV_8UC4) {
				cv::cuda::cvtColor(m, sroi, cv::COLOR_BGRA2RGBA, 0, stream);
			} else if (m.type() == CV_8UC3) {
				cv::cuda::cvtColor(m, sroi, cv::COLOR_BGR2RGBA, 0, stream);
			} else if (m.type() == CV_8UC1) {
				m.copyTo(sroi, stream);
			} else {
				LOG(ERROR) << "Unsupported colour format: " << m.type();
				return 0;
			}

			// Do the right channel
			if (stereo) {
				auto &m = cframe->get<cv::cuda::GpuMat>((c == Channel::Colour) ? Channel::Colour2 : Channel::Colour2HighRes);
				cv::Rect roi((count % tx)*rwidth + (rwidth/2), (count / tx)*rheight, rwidth/2, rheight);
				cv::cuda::GpuMat sroi = surface.surface(roi);

				if (m.type() == CV_32F) {
					if (lossless) {
						m.convertTo(sroi, CV_16UC1, 1000, stream);
					} else {
						ftl::cuda::depth_to_vuya(m, sroi, _selectFloatMax(c), stream);
					}
				} else if (m.type() == CV_8UC4) {
					cv::cuda::cvtColor(m, sroi, cv::COLOR_BGRA2RGBA, 0, stream);
				} else if (m.type() == CV_8UC3) {
					cv::cuda::cvtColor(m, sroi, cv::COLOR_BGR2RGBA, 0, stream);
				} else if (m.type() == CV_8UC1) {
					m.copyTo(sroi, stream);
				} else {
					LOG(ERROR) << "Unsupported colour format: " << m.type();
					return 0;
				}
			}
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
