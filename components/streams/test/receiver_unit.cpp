#include "catch.hpp"

#include <ftl/streams/receiver.hpp>
#include <ftl/codecs/nvidia_encoder.hpp>
#include <ftl/streams/injectors.hpp>

#include <nlohmann/json.hpp>

using ftl::codecs::definition_t;
using ftl::codecs::codec_t;
using ftl::stream::Receiver;
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
		}
		if (cb_) cb_(spkt, pkt);
		return true;
	}

	bool begin() override { return true; }
	bool end() override { return true; }
	bool active() override { return true; }

	private:
	std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> cb_;
};


TEST_CASE( "Receiver generating onFrameSet" ) {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *receiver = ftl::create<Receiver>(cfg);

	json_t cfg2 = json_t{
		{"$id","ftl://test/2"}
	};
	TestStream stream(cfg2);
	receiver->setStream(&stream);
	receiver->set("frameset_buffer_size", 0);

	ftl::codecs::NvidiaEncoder encoder(definition_t::HD1080, definition_t::SD480);

	ftl::codecs::Packet pkt;
	pkt.codec = codec_t::Any;
	pkt.bitrate = 255;
	pkt.flags = 0;
	pkt.frame_count = 1;

	ftl::codecs::StreamPacket spkt;
	spkt.version = 4;
	spkt.timestamp = 10;
	spkt.frame_number = 0;
	spkt.channel = Channel::Colour;
	spkt.streamID = 0;

	ftl::rgbd::Frame dummy;
	ftl::rgbd::FrameState state;
	state.getLeft().width = 1280;
	state.getLeft().height = 720;
	dummy.setOrigin(&state);
	ftl::stream::injectCalibration(&stream, dummy, 0, 0, 0);

	ftl::timer::start(false);

	SECTION("a single colour frame") {
		cv::cuda::GpuMat m(cv::Size(1280,720), CV_8UC4, cv::Scalar(0));

		bool r = encoder.encode(m, pkt);
		REQUIRE( r );

		stream.post(spkt, pkt);

		int count = 0;
		receiver->onFrameSet([&count](ftl::rgbd::FrameSet &fs) {
			++count;

			REQUIRE( fs.timestamp == 10 );
			REQUIRE( fs.frames.size() == 1 );
			REQUIRE( fs.frames[0].hasChannel(Channel::Colour) );
			REQUIRE( fs.frames[0].get<cv::cuda::GpuMat>(Channel::Colour).rows == 720 );
			REQUIRE( fs.frames[0].get<cv::cuda::GpuMat>(Channel::Colour).type() == CV_8UC4 );

			return true;
		});

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
	}

	SECTION("multi-frameset") {
		cv::cuda::GpuMat m(cv::Size(1280,720), CV_8UC4, cv::Scalar(0));
		ftl::stream::injectCalibration(&stream, dummy, 1, 1, 0);

		bool r = encoder.encode(m, pkt);
		REQUIRE( r );

		stream.post(spkt, pkt);

		std::atomic<int> mask = 0;
		receiver->onFrameSet([&mask](ftl::rgbd::FrameSet &fs) {
			mask |= 1 << fs.id;
			return true;
		});

		spkt.streamID = 1;
		stream.post(spkt, pkt);

		int i=10;
		while (i-- > 0 && mask != 3) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( mask == 3 );
	}

	SECTION("a tiled colour frame") {
		cv::cuda::GpuMat m(cv::Size(2560,720), CV_8UC4, cv::Scalar(0));
		ftl::stream::injectCalibration(&stream, dummy, 0, 0, 1);

		pkt.frame_count = 2;
		bool r = encoder.encode(m, pkt);
		REQUIRE( r );

		stream.post(spkt, pkt);

		int count = 0;
		receiver->onFrameSet([&count](ftl::rgbd::FrameSet &fs) {
			++count;

			REQUIRE( fs.timestamp == 10 );
			REQUIRE( fs.frames.size() == 2 );
			REQUIRE( fs.frames[0].hasChannel(Channel::Colour) );
			REQUIRE( fs.frames[0].get<cv::cuda::GpuMat>(Channel::Colour).rows == 720 );
			REQUIRE( fs.frames[0].get<cv::cuda::GpuMat>(Channel::Colour).type() == CV_8UC4 );
			REQUIRE( fs.frames[1].hasChannel(Channel::Colour) );
			REQUIRE( fs.frames[1].get<cv::cuda::GpuMat>(Channel::Colour).rows == 720 );
			REQUIRE( fs.frames[1].get<cv::cuda::GpuMat>(Channel::Colour).type() == CV_8UC4 );

			return true;
		});

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
	}

	SECTION("a tiled lossy depth frame") {
		cv::cuda::GpuMat m(cv::Size(2560,720), CV_32F, cv::Scalar(0));
		ftl::stream::injectCalibration(&stream, dummy, 0, 0, 1);

		spkt.channel = Channel::Depth;
		pkt.frame_count = 2;
		pkt.flags = 0;
		bool r = encoder.encode(m, pkt);
		REQUIRE( r );

		stream.post(spkt, pkt);

		int count = 0;
		receiver->onFrameSet([&count](ftl::rgbd::FrameSet &fs) {
			++count;

			REQUIRE( fs.timestamp == 10 );
			REQUIRE( fs.frames.size() == 2 );
			REQUIRE( fs.frames[0].hasChannel(Channel::Depth) );
			REQUIRE( fs.frames[0].get<cv::cuda::GpuMat>(Channel::Depth).rows == 720 );
			REQUIRE( fs.frames[0].get<cv::cuda::GpuMat>(Channel::Depth).type() == CV_32F );
			REQUIRE( fs.frames[1].hasChannel(Channel::Depth) );
			REQUIRE( fs.frames[1].get<cv::cuda::GpuMat>(Channel::Depth).rows == 720 );
			REQUIRE( fs.frames[1].get<cv::cuda::GpuMat>(Channel::Depth).type() == CV_32F );

			return true;
		});

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
	}

	SECTION("a tiled lossless depth frame") {
		cv::cuda::GpuMat m(cv::Size(2560,720), CV_32F, cv::Scalar(0));
		ftl::stream::injectCalibration(&stream, dummy, 0, 0, 1);

		spkt.channel = Channel::Depth;
		pkt.frame_count = 2;
		pkt.flags = 0;
		pkt.codec = codec_t::HEVC_LOSSLESS;
		bool r = encoder.encode(m, pkt);
		REQUIRE( r );

		stream.post(spkt, pkt);

		int count = 0;
		receiver->onFrameSet([&count](ftl::rgbd::FrameSet &fs) {
			++count;

			REQUIRE( fs.timestamp == 10 );
			REQUIRE( fs.frames.size() == 2 );
			REQUIRE( fs.frames[0].hasChannel(Channel::Depth) );
			REQUIRE( fs.frames[0].get<cv::cuda::GpuMat>(Channel::Depth).rows == 720 );
			REQUIRE( fs.frames[0].get<cv::cuda::GpuMat>(Channel::Depth).type() == CV_32F );
			REQUIRE( fs.frames[1].hasChannel(Channel::Depth) );
			REQUIRE( fs.frames[1].get<cv::cuda::GpuMat>(Channel::Depth).rows == 720 );
			REQUIRE( fs.frames[1].get<cv::cuda::GpuMat>(Channel::Depth).type() == CV_32F );

			return true;
		});

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
	}

	ftl::timer::stop(true);
	//while (ftl::pool.n_idle() != ftl::pool.size()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
	delete receiver;
	//ftl::config::cleanup();
}

TEST_CASE( "Receiver sync bugs" ) {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *receiver = ftl::create<Receiver>(cfg);

	json_t cfg2 = json_t{
		{"$id","ftl://test/2"}
	};
	TestStream stream(cfg2);
	receiver->setStream(&stream);
	receiver->set("frameset_buffer_size", 0);

	ftl::codecs::NvidiaEncoder encoder(definition_t::HD1080, definition_t::SD480);

	ftl::codecs::Packet pkt;
	pkt.codec = codec_t::Any;
	pkt.bitrate = 255;
	pkt.flags = 0;
	pkt.frame_count = 1;

	ftl::codecs::StreamPacket spkt;
	spkt.version = 4;
	spkt.timestamp = 10;
	spkt.frame_number = 0;
	spkt.channel = Channel::Colour;
	spkt.streamID = 0;

	ftl::rgbd::Frame dummy;
	ftl::rgbd::FrameState state;
	state.getLeft().width = 1280;
	state.getLeft().height = 720;
	dummy.setOrigin(&state);
	ftl::stream::injectCalibration(&stream, dummy, 0, 0, 0);

	ftl::timer::start(false);

	stream.select(0, Channel::Colour + Channel::Colour2);

	SECTION("out of phase packets") {
		cv::cuda::GpuMat m(cv::Size(1280,720), CV_8UC4, cv::Scalar(0));

		bool r = encoder.encode(m, pkt);
		REQUIRE( r );

		int count = 0;
		int64_t ts = 0;
		bool haswrongchan = false;
		receiver->onFrameSet([&count,&ts,&haswrongchan](ftl::rgbd::FrameSet &fs) {
			++count;

			ts = fs.timestamp;
			haswrongchan = fs.frames[0].hasChannel(Channel::ColourHighRes);

			return true;
		});

		try { stream.post(spkt, pkt); } catch(...) {}
		spkt.timestamp = 10;
		spkt.channel = Channel::ColourHighRes;
		try { stream.post(spkt, pkt); } catch(...) {}
		spkt.timestamp = 20;
		spkt.channel = Channel::Colour2;
		try { stream.post(spkt, pkt); } catch(...) {}
		spkt.timestamp = 20;
		spkt.channel = Channel::Colour;
		try { stream.post(spkt, pkt); } catch(...) {}

		int i=10;
		while (i-- > 0 && count < 2) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 2 );
		REQUIRE( ts == 20 );
		REQUIRE( !haswrongchan );
	}

	ftl::timer::stop(true);
	//while (ftl::pool.n_idle() != ftl::pool.size()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
	delete receiver;
}

TEST_CASE( "Receiver non zero buffer" ) {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *receiver = ftl::create<Receiver>(cfg);

	json_t cfg2 = json_t{
		{"$id","ftl://test/2"}
	};
	TestStream stream(cfg2);
	receiver->setStream(&stream);
	receiver->set("frameset_buffer_size", 1);

	ftl::codecs::NvidiaEncoder encoder(definition_t::HD1080, definition_t::SD480);

	ftl::codecs::Packet pkt;
	pkt.codec = codec_t::Any;
	pkt.bitrate = 255;
	pkt.flags = 0;
	pkt.frame_count = 1;

	ftl::codecs::StreamPacket spkt;
	spkt.version = 4;
	spkt.timestamp = 10;
	spkt.frame_number = 0;
	spkt.channel = Channel::Colour;
	spkt.streamID = 0;

	ftl::rgbd::Frame dummy;
	ftl::rgbd::FrameState state;
	state.getLeft().width = 1280;
	state.getLeft().height = 720;
	dummy.setOrigin(&state);
	ftl::stream::injectCalibration(&stream, dummy, 0, 0, 0);

	ftl::timer::start(false);

	SECTION("Fixed buffer delay") {
		cv::cuda::GpuMat m(cv::Size(1280,720), CV_8UC4, cv::Scalar(0));

		bool r = encoder.encode(m, pkt);
		REQUIRE( r );

		int count = 0;
		receiver->onFrameSet([&count](ftl::rgbd::FrameSet &fs) {
			++count;

			REQUIRE( fs.timestamp == 10 );
			REQUIRE( fs.frames.size() == 1 );
			REQUIRE( fs.frames[0].hasChannel(Channel::Colour) );
			REQUIRE( fs.frames[0].get<cv::cuda::GpuMat>(Channel::Colour).rows == 720 );
			REQUIRE( fs.frames[0].get<cv::cuda::GpuMat>(Channel::Colour).type() == CV_8UC4 );

			return true;
		});

		stream.post(spkt, pkt);
		spkt.timestamp += 10;
		stream.post(spkt, pkt);

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
	}

	ftl::timer::stop(true);
	//while (ftl::pool.n_idle() != ftl::pool.size()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
	delete receiver;
}
