#include "injectors.hpp"
#include <ftl/utility/vectorbuffer.hpp>

using ftl::codecs::Channel;
using ftl::util::FTLVectorBuffer;

void ftl::stream::injectCalibration(ftl::stream::Stream *stream, const ftl::rgbd::FrameSet &fs, int ix, bool right) {
	ftl::stream::injectCalibration(stream, fs.frames[ix], fs.timestamp, ix, right);
}

void ftl::stream::injectPose(ftl::stream::Stream *stream, const ftl::rgbd::FrameSet &fs, int ix) {
	ftl::stream::injectPose(stream, fs.frames[ix], fs.timestamp, ix);
}

void ftl::stream::injectConfig(ftl::stream::Stream *stream, const ftl::rgbd::FrameSet &fs, int ix) {
	ftl::codecs::StreamPacket spkt = {
		4,
		fs.timestamp,
		0,
		static_cast<uint8_t>(ix),
		Channel::Configuration
	};

	ftl::codecs::Packet pkt;
	pkt.codec = ftl::codecs::codec_t::MSGPACK;
	pkt.definition = ftl::codecs::definition_t::Any;
	pkt.bitrate = 0;
	pkt.frame_count = 1;
	pkt.flags = 0;

	FTLVectorBuffer buf(pkt.data);
	msgpack::pack(buf, fs.frames[ix].getConfigString());

	stream->post(spkt, pkt);
}

void ftl::stream::injectPose(ftl::stream::Stream *stream, const ftl::rgbd::Frame &f, int64_t ts, int ix) {
    ftl::codecs::StreamPacket spkt = {
		4,
		ts,
		0,
		static_cast<uint8_t>(ix),
		Channel::Pose
	};

	ftl::codecs::Packet pkt;
	pkt.codec = ftl::codecs::codec_t::MSGPACK;
	pkt.definition = ftl::codecs::definition_t::Any;
	pkt.bitrate = 0;
	pkt.frame_count = 1;
	pkt.flags = 0;

	auto &pose = f.getPose();
	std::vector<double> data(pose.data(), pose.data() + 4*4);
	FTLVectorBuffer buf(pkt.data);
	msgpack::pack(buf, data);

	stream->post(spkt, pkt);
}

void ftl::stream::injectCalibration(ftl::stream::Stream *stream, const ftl::rgbd::Frame &f, int64_t ts, int ix, bool right) {
    ftl::codecs::StreamPacket spkt = {
		4,
		ts,
		0,
		static_cast<uint8_t>(ix),
		(right) ? Channel::Calibration2 : Channel::Calibration
	};

	auto data = (right) ?
		std::make_tuple(f.getRightCamera(), Channel::Right, 0) :
		std::make_tuple(f.getLeftCamera(), Channel::Left, 0);

	ftl::codecs::Packet pkt;
	pkt.codec = ftl::codecs::codec_t::MSGPACK;
	pkt.definition = ftl::codecs::definition_t::Any;
	pkt.bitrate = 0;
	pkt.frame_count = 1;
	pkt.flags = 0;

	FTLVectorBuffer buf(pkt.data);
	msgpack::pack(buf, data);
	stream->post(spkt, pkt);
}
