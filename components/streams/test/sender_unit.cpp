#include "catch.hpp"

#include <ftl/streams/sender.hpp>
#include <ftl/codecs/hevc.hpp>

using ftl::codecs::definition_t;
using ftl::codecs::codec_t;
using ftl::stream::Sender;
using ftl::rgbd::Frame;
using ftl::rgbd::FrameSet;
using ftl::codecs::Channel;
using ftl::codecs::Channels;
using ftl::config::json_t;

class TestStream : public ftl::stream::Stream {
	public:
	explicit TestStream(nlohmann::json &config) : ftl::stream::Stream(config) {};
	~TestStream() {};

	bool onPacket(const std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> &cb) {
		cb_ = cb;
		return true;
	}

	bool onIntercept(const std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> &cb) {
		icb_ = cb;
		return true;
	}

	bool post(const ftl::codecs::StreamPacket &spkt, const ftl::codecs::Packet &pkt) {
		available(spkt.streamID) += spkt.channel;
		if (pkt.data.size() == 0) {
			if (spkt.frameSetID() == 255) {
				for (int i=0; i<size(); ++i) {
					select(i, selected(i) + spkt.channel);
				}
			} else {
				select(spkt.frameSetID(), selected(spkt.frameSetID()) + spkt.channel);
			}
			if (cb_) cb_(spkt, pkt);
		}
		if (icb_) icb_(spkt, pkt);
		return true;
	}

	bool begin() override { return true; }
	bool end() override { return true; }
	bool active() override { return true; }

	private:
	std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> cb_;
	std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> icb_;
};


TEST_CASE( "Sender::post() video frames" ) {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *sender = ftl::create<Sender>(cfg);
	
	FrameSet fs;
	fs.frames.emplace_back();
	fs.timestamp = 1000;

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

	SECTION("a single colour frame") {
		stream.select(0, Channels(Channel::Colour), true);

		fs.count = 1;
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Colour).create(cv::Size(1280,720), CV_8UC4);
		fs.frames[0].get<cv::cuda::GpuMat>(Channel::Colour).setTo(cv::Scalar(0));

		sender->post(fs);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 4 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Colour );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.definition == definition_t::HD720 );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.frame_count == 1 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}

	SECTION("two colour frames tiled") {
		stream.select(0, Channels(Channel::Colour), true);

		fs.count = 2;
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Colour).create(cv::Size(1280,720), CV_8UC4);
		fs.frames[0].get<cv::cuda::GpuMat>(Channel::Colour).setTo(cv::Scalar(0));
		fs.frames.emplace_back();
		fs.frames[1].create<cv::cuda::GpuMat>(Channel::Colour).create(cv::Size(1280,720), CV_8UC4);
		fs.frames[1].get<cv::cuda::GpuMat>(Channel::Colour).setTo(cv::Scalar(0));

		sender->post(fs);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 4 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Colour );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.definition == definition_t::HD720 );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.frame_count == 2 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}

	SECTION("two depth frames tiled") {
		stream.select(0, Channels(Channel::Depth), true);

		fs.count = 2;
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
		fs.frames[0].get<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));
		fs.frames.emplace_back();
		fs.frames[1].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
		fs.frames[1].get<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));

		sender->post(fs);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 4 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Depth );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.definition == definition_t::HD720 );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.flags == (ftl::codecs::kFlagFloat | ftl::codecs::kFlagMappedDepth) );
		REQUIRE( pkt.frame_count == 2 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}

	SECTION("10 depth frames tiled") {
		stream.select(0, Channels(Channel::Depth), true);

		fs.count = 10;
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
		fs.frames[0].get<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));

		for (int i=1; i<10; ++i) {
			fs.frames.emplace_back();
			fs.frames[i].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
			fs.frames[i].get<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));
		}

		sender->post(fs);

		REQUIRE( count == 2 );
		REQUIRE( spkt.version == 4 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 9 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Depth );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.definition == definition_t::HD720 );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.flags == (ftl::codecs::kFlagFloat | ftl::codecs::kFlagMappedDepth) );
		REQUIRE( pkt.frame_count == 1 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}

	SECTION("two lossless depth frames tiled") {
		stream.select(0, Channels(Channel::Depth), true);

		fs.count = 2;
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
		fs.frames[0].get<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));
		fs.frames.emplace_back();
		fs.frames[1].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
		fs.frames[1].get<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));

		sender->set("lossless", true);
		sender->post(fs);

		REQUIRE( count == 1 );
		REQUIRE( spkt.version == 4 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Depth );
		REQUIRE( pkt.codec == codec_t::HEVC_LOSSLESS );
		REQUIRE( pkt.definition == definition_t::HD720 );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.flags == (ftl::codecs::kFlagFloat) );
		REQUIRE( pkt.frame_count == 2 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}

	SECTION("one frame and two channels") {
		stream.select(0, Channel::Colour + Channel::Depth, true);

		fs.count = 1;
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Colour).create(cv::Size(1280,720), CV_8UC4);
		fs.frames[0].get<cv::cuda::GpuMat>(Channel::Colour).setTo(cv::Scalar(0));
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Depth).create(cv::Size(1280,720), CV_32F);
		fs.frames[0].get<cv::cuda::GpuMat>(Channel::Depth).setTo(cv::Scalar(0.0f));

		sender->post(fs);

		REQUIRE( count == 2 );
		REQUIRE( spkt.version == 4 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Depth );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.definition == definition_t::HD720 );
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
	
	FrameSet fs;
	fs.frames.emplace_back();
	fs.timestamp = 1000;

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
			codec_t::Any, definition_t::Any, 255, 255, 0, {}
		});

		fs.count = 1;
		fs.frames[0].create<cv::cuda::GpuMat>(Channel::Colour).create(cv::Size(1280,720), CV_8UC4);
		fs.frames[0].get<cv::cuda::GpuMat>(Channel::Colour).setTo(cv::Scalar(0));

		count = 0;
		sender->post(fs);

		REQUIRE( count == 5 );
		REQUIRE( spkt.version == 4 );
		REQUIRE( spkt.timestamp == 1000 );
		REQUIRE( (int)spkt.frame_number == 0 );
		REQUIRE( spkt.streamID == 0 );
		REQUIRE( spkt.channel == Channel::Colour );
		REQUIRE( pkt.codec == codec_t::HEVC );
		REQUIRE( pkt.definition == definition_t::HD720 );
		REQUIRE( pkt.data.size() > 0 );
		REQUIRE( pkt.frame_count == 1 );
		REQUIRE( ftl::codecs::hevc::validNAL(pkt.data.data(), pkt.data.size()) );
	}
}
