#include "catch.hpp"

#include <ftl/streams/sender.hpp>
#include <ftl/codecs/hevc.hpp>
#include <ftl/data/framepool.hpp>

#include <nlohmann/json.hpp>

#include <loguru.hpp>

using ftl::codecs::definition_t;
using ftl::codecs::codec_t;
using ftl::stream::Sender;
using ftl::data::Frame;
using ftl::data::FrameSet;
using ftl::codecs::Channel;
using ftl::codecs::Channels;
using ftl::config::json_t;

class TestStream : public ftl::stream::Stream {
	public:
	explicit TestStream(nlohmann::json &config) : ftl::stream::Stream(config) {};
	~TestStream() {};

	bool onIntercept(const std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> &cb) {
		icb_ = cb;
		return true;
	}

	bool post(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		available(spkt.streamID) += spkt.channel;
		if (pkt.data.size() == 0) {
			if (spkt.frameSetID() == 255) {
				for (size_t i=0; i<size(); ++i) {
					select(i, selected(i) + spkt.channel);
				}
			} else {
				select(spkt.frameSetID(), selected(spkt.frameSetID()) + spkt.channel);
			}
			cb_.trigger(spkt, pkt);
		}
		if (icb_) icb_(spkt, pkt);
		return true;
	}

	bool begin() override { return true; }
	bool end() override { return true; }
	bool active() override { return true; }

	private:
	//std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> cb_;
	std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> icb_;
};


TEST_CASE( "Sender::post() video frames" ) {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *sender = ftl::create<Sender>(cfg);
	
	ftl::data::Pool pool(4,6);
	Frame f = pool.allocate(ftl::data::FrameID(0,0), 1000);
	f.store();
	auto fsptr = FrameSet::fromFrame(f);
	FrameSet &fs = *fsptr;

	json_t cfg2 = json_t{
		{"$id","ftl://test/2"}
	};
	TestStream stream(cfg2);
	sender->setStream(&stream);

	ftl::codecs::StreamPacket prev_spkt;
	ftl::codecs::StreamPacket spkt;
	ftl::codecs::Packet pkt;
	int count = 0;

	stream.onIntercept([&count,&spkt,&pkt,&prev_spkt](const ftl::codecs::StreamPacket &pspkt, const ftl::codecs::Packet &ppkt) {
		prev_spkt = spkt;
		spkt = pspkt;
		pkt = ppkt;
		++count;
	});

	SECTION("a single colour frame") {
		stream.select(0, {Channel::Colour}, true);

		fs.mask = 1;
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Colour).create(cv::Size(1280,720), CV_8UC4);
		fs.frames[0].set<cv::cuda::GpuMat>(Channel::Colour).setTo(cv::Scalar(0));

		sender->post(fs, Channel::Colour);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 5 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Colour );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.frame_count == 1 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}

	SECTION("two colour frames tiled") {
		stream.select(0, {Channel::Colour}, true);

		fs.resize(2);
		fs.frames[1].store();

		fs.mask = 3;
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Colour).create(cv::Size(1280,720), CV_8UC4);
		fs.frames[0].set<cv::cuda::GpuMat>(Channel::Colour).setTo(cv::Scalar(0));
		fs.frames[1].create<cv::cuda::GpuMat>(Channel::Colour).create(cv::Size(1280,720), CV_8UC4);
		fs.frames[1].set<cv::cuda::GpuMat>(Channel::Colour).setTo(cv::Scalar(0));

		sender->post(fs, Channel::Colour);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 5 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Colour );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.frame_count == 2 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}

	SECTION("two depth frames tiled") {
		stream.select(0, {Channel::Depth}, true);

		fs.resize(2);
		fs.frames[1].store();

		fs.mask = 3;
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
		fs.frames[0].set<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));
		fs.frames[1].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
		fs.frames[1].set<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));

		sender->post(fs, Channel::Depth);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 5 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Depth );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.flags == (ftl::codecs::kFlagFloat | ftl::codecs::kFlagMappedDepth) );
		REQUIRE( pkt.frame_count == 2 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}

	SECTION("10 depth frames tiled") {
		stream.select(0, {Channel::Depth}, true);

		fs.resize(10);

		fs.mask = 0x3FF;
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
		fs.frames[0].set<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));

		for (int i=1; i<10; ++i) {
			fs.frames[i].store();
			fs.frames[i].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
			fs.frames[i].set<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));
		}

		sender->post(fs, Channel::Depth);

		REQUIRE( count == 2 );
		REQUIRE( spkt.version == 5 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 9 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Depth );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.flags == (ftl::codecs::kFlagFloat | ftl::codecs::kFlagMappedDepth) );
		REQUIRE( pkt.frame_count == 1 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}

	SECTION("4 depth frames tiled, missing first") {
		stream.select(0, {Channel::Depth}, true);

		fs.resize(4);

		fs.mask = 0xF;
		//fs.frames[0].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
		//fs.frames[0].set<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));

		for (int i=1; i<4; ++i) {
			fs.frames[i].store();
			fs.frames[i].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
			fs.frames[i].set<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));
		}

		sender->post(fs, Channel::Depth);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 5 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 1 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Depth );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.flags == (ftl::codecs::kFlagFloat | ftl::codecs::kFlagMappedDepth) );
		REQUIRE( pkt.frame_count == 3 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}

	SECTION("two lossless depth frames tiled") {
		stream.select(0, {Channel::Depth}, true);

		fs.resize(2);
		fs.frames[1].store();

		fs.mask = 3;
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
		fs.frames[0].set<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));
		fs.frames[1].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
		fs.frames[1].set<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));

		sender->set("codec_float", (int)codec_t::HEVC_LOSSLESS);
		sender->post(fs, Channel::Depth);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 5 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Depth );
		REQUIRE( pkt.codec == codec_t::HEVC_LOSSLESS );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.flags == (ftl::codecs::kFlagFloat) );
		REQUIRE( pkt.frame_count == 2 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}

	SECTION("one frame and two channels") {
		stream.select(0, Channel::Colour + Channel::Depth, true);

		fs.mask = 1;
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Colour).create(cv::Size(1280,720), CV_8UC4);
		fs.frames[0].set<cv::cuda::GpuMat>(Channel::Colour).setTo(cv::Scalar(0));
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
		fs.frames[0].set<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));

		sender->post(fs, Channel::Colour);
		sender->post(fs, Channel::Depth);

		REQUIRE( count == 2 );
		REQUIRE( spkt.version == 5 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Depth );
		REQUIRE( prev_spkt.channel == Channel::Colour );
		REQUIRE( prev_spkt.timestamp == 1000 );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.frame_count == 1 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}

	//ftl::config::cleanup();
	delete sender;
}

TEST_CASE( "Sender request to control encoding" ) {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *sender = ftl::create<Sender>(cfg);
	
	ftl::data::Pool pool(4,6);
	Frame f = pool.allocate(ftl::data::FrameID(0,0), 1000);
	f.store();
	auto fsptr = FrameSet::fromFrame(f);
	FrameSet &fs = *fsptr;

	json_t cfg2 = json_t{
		{"$id","ftl://test/2"}
	};
	TestStream stream(cfg2);
	sender->setStream(&stream);

	ftl::codecs::StreamPacket spkt;
	ftl::codecs::Packet pkt;
	int count = 0;

	stream.onIntercept([&count,&spkt,&pkt](const ftl::codecs::StreamPacket &pspkt, const ftl::codecs::Packet &ppkt) {
		spkt = pspkt;
		pkt = ppkt;
		++count;
	});

	SECTION("a single colour frame request") {
		//stream.select(0, Channels(Channel::Colour), true);

		stream.post({
			4, 1000, 0, 255, Channel::Colour
		},{
			codec_t::Any, 0, 255, 255, 0, {}
		});

		fs.mask = 1;
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Colour).create(cv::Size(1280,720), CV_8UC4);
		fs.frames[0].set<cv::cuda::GpuMat>(Channel::Colour).setTo(cv::Scalar(0));

		fs.frames[0].create<std::tuple<ftl::rgbd::Camera, Channel, int>>(Channel::Calibration);
		fs.frames[0].create<Eigen::Matrix4d>(Channel::Pose);

		count = 0;
		sender->post(fs, Channel::Colour);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 5 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Colour );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.frame_count == 1 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}
}

TEST_CASE( "Sender::post() data channels" ) {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *sender = ftl::create<Sender>(cfg);
	
	ftl::data::Pool pool(4,6);
	Frame f = pool.allocate(ftl::data::FrameID(0,0), 1000);
	f.store();
	auto fsptr = FrameSet::fromFrame(f);
	FrameSet &fs = *fsptr;

	json_t cfg2 = json_t{
		{"$id","ftl://test/2"}
	};
	TestStream stream(cfg2);
	sender->setStream(&stream);

	ftl::codecs::StreamPacket spkt;
	ftl::codecs::Packet pkt;
	int count = 0;

	stream.onIntercept([&count,&spkt,&pkt](const ftl::codecs::StreamPacket &pspkt, const ftl::codecs::Packet &ppkt) {
		spkt = pspkt;
		pkt = ppkt;
		++count;
	});

	SECTION("a single calibration channel") {
		stream.select(0, {Channel::Calibration}, true);

		fs.mask = 1;
		auto &calib = std::get<0>(fs.frames[0].create<std::tuple<ftl::rgbd::Camera, Channel, int>>(Channel::Calibration));
		calib.width = 1024;

		fs.frames[0].flush();
		sender->post(fs, Channel::Calibration);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 5 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Calibration );
		REQUIRE( pkt.codec == codec_t::MSGPACK );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.frame_count == 1 );
	}

	SECTION("a single pose channel") {
		stream.select(0, {Channel::Pose}, true);

		fs.mask = 1;
		fs.frames[0].create<Eigen::Matrix4d>(Channel::Pose);

		fs.frames[0].flush();
		sender->post(fs, Channel::Pose);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 5 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Pose );
		REQUIRE( pkt.codec == codec_t::MSGPACK );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.frame_count == 1 );
	}

	SECTION("a single custom channel") {
		stream.select(0, {Channel::Configuration}, true);

		fs.mask = 1;
		auto &vf = fs.frames[0].create<std::vector<float>>(Channel::Configuration);
		vf.push_back(5.0f);
		vf.push_back(33.0f);

		fs.frames[0].flush();
		sender->post(fs, Channel::Configuration);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 5 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Configuration );
		REQUIRE( pkt.codec == codec_t::MSGPACK );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.frame_count == 1 );
		// TODO: Check decodes correctly.
	}

	SECTION("a single list channel") {
		stream.select(0, {Channel::Configuration}, true);

		fs.mask = 1;
		auto vf = fs.frames[0].create<std::list<float>>(Channel::Configuration);
		vf = 5.0f;
		vf = 33.0f;

		fs.frames[0].flush();
		sender->post(fs, Channel::Configuration);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 5 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Configuration );
		REQUIRE( pkt.codec == codec_t::MSGPACK );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.frame_count == 1 );
		// TODO: Check decodes correctly.
	}
}

TEST_CASE( "Sender::post() audio channels" ) {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *sender = ftl::create<Sender>(cfg);
	
	ftl::data::Pool pool(4,6);
	Frame f = pool.allocate(ftl::data::FrameID(0,0), 1000);
	f.store();
	auto fsptr = FrameSet::fromFrame(f);
	FrameSet &fs = *fsptr;

	json_t cfg2 = json_t{
		{"$id","ftl://test/2"}
	};
	TestStream stream(cfg2);
	sender->setStream(&stream);

	ftl::codecs::StreamPacket spkt;
	ftl::codecs::Packet pkt;
	int count = 0;

	stream.onIntercept([&count,&spkt,&pkt](const ftl::codecs::StreamPacket &pspkt, const ftl::codecs::Packet &ppkt) {
		spkt = pspkt;
		pkt = ppkt;
		++count;
	});

	SECTION("a single mono audio channel") {
		ftl::data::make_type<ftl::rgbd::Camera>();

		stream.select(0, {Channel::AudioMono}, true);

		fs.mask = 1;
		auto audio = fs.frames[0].create<std::list<ftl::audio::AudioFrame>>(Channel::AudioMono);

		// Fake 3 audio frames
		ftl::audio::AudioFrame aframe;
		aframe.data().resize(3*ftl::audio::kFrameSize);
		audio = std::move(aframe);

		fs.frames[0].flush();
		sender->post(fs, Channel::AudioMono);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 5 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::AudioMono );
		REQUIRE( pkt.codec == codec_t::OPUS );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.frame_count == 1 );
	}

	SECTION("multi frame mono audio channel") {
		ftl::data::make_type<ftl::rgbd::Camera>();

		stream.select(0, {Channel::AudioMono}, true);

		fs.mask = 1;
		auto audio = fs.frames[0].create<std::list<ftl::audio::AudioFrame>>(Channel::AudioMono);

		// Fake 3 audio frames
		ftl::audio::AudioFrame aframe1;
		aframe1.data().resize(3*ftl::audio::kFrameSize);
		audio = std::move(aframe1);

		sender->post(fs, Channel::AudioMono);
		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 5 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::AudioMono );
		REQUIRE( pkt.codec == codec_t::OPUS );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.frame_count == 1 );

		size_t firstsize = pkt.data.size();
		pkt.data.clear();

		ftl::audio::AudioFrame aframe2;
		aframe2.data().resize(2*ftl::audio::kFrameSize);
		audio = std::move(aframe2);

		//fs.frames[0].flush();
		sender->post(fs, Channel::AudioMono);

		REQUIRE( count == 2 );
		REQUIRE( pkt.data.size() > firstsize );
	}

	SECTION("a single stereo audio channel") {
		ftl::data::make_type<ftl::rgbd::Camera>();

		stream.select(0, {Channel::AudioStereo}, true);

		fs.mask = 1;
		auto audio = fs.frames[0].create<std::list<ftl::audio::AudioFrame>>(Channel::AudioStereo);

		// Fake 3 audio frames
		ftl::audio::AudioFrame aframe;
		aframe.data().resize(2*3*ftl::audio::kFrameSize);
		audio = std::move(aframe);

		fs.frames[0].flush();
		sender->post(fs, Channel::AudioStereo);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 5 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::AudioStereo );
		REQUIRE( pkt.codec == codec_t::OPUS );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.frame_count == 1 );
	}
}
