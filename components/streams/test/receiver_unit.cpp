#include "catch.hpp"

#include <ftl/streams/receiver.hpp>
#include <ftl/codecs/nvidia_encoder.hpp>
#include <ftl/streams/injectors.hpp>
#include <ftl/rgbd/frame.hpp>

#include <nlohmann/json.hpp>

#include <loguru.hpp>

using ftl::codecs::definition_t;
using ftl::codecs::codec_t;
using ftl::stream::Receiver;
using ftl::rgbd::Frame;
using ftl::rgbd::FrameSet;
using ftl::codecs::Channel;
using ftl::codecs::Channels;
using ftl::config::json_t;
using ftl::data::FrameID;

class TestStream : public ftl::stream::Stream {
	public:
	explicit TestStream(nlohmann::json &config) : ftl::stream::Stream(config) {};
	~TestStream() {};

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
		cb_.trigger(spkt, pkt);
		return true;
	}

	bool begin() override { return true; }
	bool end() override { return true; }
	bool active() override { return true; }

	//private:
	//std::function<void(const ftl::codecs::StreamPacket &, const ftl::codecs::Packet &)> cb_;
};


TEST_CASE( "Receiver generating onFrameSet" ) {
	//ftl::data::make_channel<ftl::rgbd::Camera>(Channel::Calibration, "calibration", ftl::data::StorageMode::PERSISTENT);
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	ftl::data::Pool pool(5,7);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *receiver = ftl::create<Receiver>(cfg, &pool);

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

	ftl::data::Frame dummy = pool.allocate(FrameID(0,0),10);
	dummy.store();
	ftl::rgbd::Frame &state = dummy.cast<ftl::rgbd::Frame>();
	state.setLeft().width = 1280;
	state.setLeft().height = 720;

	// Must tell it to wait for colour before completing.
	stream.select(0, {Channel::Colour}, true);
	ftl::stream::injectCalibration(&stream, state, 10, 0, 0);

	ftl::timer::start(false);

	SECTION("a single colour frame") {
		cv::cuda::GpuMat m(cv::Size(1280,720), CV_8UC4, cv::Scalar(0));

		bool r = encoder.encode(m, pkt);
		REQUIRE( r );

		spkt.flags |= ftl::codecs::kFlagCompleted;
		stream.post(spkt, pkt);

		int count = 0;
		auto h = receiver->onFrameSet([&count](const ftl::data::FrameSetPtr& fs) {
			++count;

			REQUIRE( fs->timestamp() == 10 );
			REQUIRE( fs->frames.size() == 1 );
			REQUIRE( fs->frames[0].hasChannel(Channel::Colour) );
			REQUIRE( fs->frames[0].get<cv::cuda::GpuMat>(Channel::Colour).rows == 720 );
			REQUIRE( fs->frames[0].get<cv::cuda::GpuMat>(Channel::Colour).type() == CV_8UC4 );

			return true;
		});

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
	}

	SECTION("multi-frameset") {
		cv::cuda::GpuMat m(cv::Size(1280,720), CV_8UC4, cv::Scalar(0));

		stream.select(1, {Channel::Colour}, true);
		ftl::stream::injectCalibration(&stream, state, 10, 1, 0);

		bool r = encoder.encode(m, pkt);
		REQUIRE( r );

		spkt.flags |= ftl::codecs::kFlagCompleted;
		stream.post(spkt, pkt);

		std::atomic<int> mask = 0;
		auto h = receiver->onFrameSet([&mask](const ftl::data::FrameSetPtr& fs) {
			mask |= 1 << fs->frameset();
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
		ftl::stream::injectCalibration(&stream, state, 10, 0, 1);

		pkt.frame_count = 2;
		bool r = encoder.encode(m, pkt);
		REQUIRE( r );

		spkt.flags |= ftl::codecs::kFlagCompleted;
		stream.post(spkt, pkt);

		int count = 0;
		auto h = receiver->onFrameSet([&count](const ftl::data::FrameSetPtr& fs) {
			++count;

			REQUIRE( fs->timestamp() == 10 );
			REQUIRE( fs->frames.size() == 2 );
			REQUIRE( fs->frames[0].hasChannel(Channel::Colour) );
			REQUIRE( fs->frames[0].get<cv::cuda::GpuMat>(Channel::Colour).rows == 720 );
			REQUIRE( fs->frames[0].get<cv::cuda::GpuMat>(Channel::Colour).type() == CV_8UC4 );
			REQUIRE( fs->frames[1].hasChannel(Channel::Colour) );
			REQUIRE( fs->frames[1].get<cv::cuda::GpuMat>(Channel::Colour).rows == 720 );
			REQUIRE( fs->frames[1].get<cv::cuda::GpuMat>(Channel::Colour).type() == CV_8UC4 );

			return true;
		});

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
	}

	SECTION("a tiled lossy depth frame") {
		cv::cuda::GpuMat m(cv::Size(2560,720), CV_32F, cv::Scalar(0));
		ftl::stream::injectCalibration(&stream, state, 10, 0, 1);

		stream.select(0, {Channel::Depth}, true);

		spkt.channel = Channel::Depth;
		pkt.frame_count = 2;
		pkt.flags = 0;
		bool r = encoder.encode(m, pkt);
		REQUIRE( r );

		spkt.flags |= ftl::codecs::kFlagCompleted;
		stream.post(spkt, pkt);

		int count = 0;
		auto h = receiver->onFrameSet([&count](const ftl::data::FrameSetPtr& fs) {
			++count;

			REQUIRE( fs->timestamp() == 10 );
			REQUIRE( fs->frames.size() == 2 );
			REQUIRE( fs->frames[0].hasChannel(Channel::Depth) );
			REQUIRE( fs->frames[0].get<cv::cuda::GpuMat>(Channel::Depth).rows == 720 );
			REQUIRE( fs->frames[0].get<cv::cuda::GpuMat>(Channel::Depth).type() == CV_32F );
			REQUIRE( fs->frames[1].hasChannel(Channel::Depth) );
			REQUIRE( fs->frames[1].get<cv::cuda::GpuMat>(Channel::Depth).rows == 720 );
			REQUIRE( fs->frames[1].get<cv::cuda::GpuMat>(Channel::Depth).type() == CV_32F );

			return true;
		});

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
	}

	SECTION("a tiled lossless depth frame") {
		cv::cuda::GpuMat m(cv::Size(2560,720), CV_32F, cv::Scalar(0));
		ftl::stream::injectCalibration(&stream, state, 10, 0, 1);

		stream.select(0, {Channel::Depth}, true);

		spkt.channel = Channel::Depth;
		pkt.frame_count = 2;
		pkt.flags = 0;
		pkt.codec = codec_t::HEVC_LOSSLESS;
		bool r = encoder.encode(m, pkt);
		REQUIRE( r );

		spkt.flags |= ftl::codecs::kFlagCompleted;
		stream.post(spkt, pkt);

		int count = 0;
		auto h = receiver->onFrameSet([&count](const ftl::data::FrameSetPtr& fs) {
			++count;

			REQUIRE( fs->timestamp() == 10 );
			REQUIRE( fs->frames.size() == 2 );
			REQUIRE( fs->frames[0].hasChannel(Channel::Depth) );
			REQUIRE( fs->frames[0].get<cv::cuda::GpuMat>(Channel::Depth).rows == 720 );
			REQUIRE( fs->frames[0].get<cv::cuda::GpuMat>(Channel::Depth).type() == CV_32F );
			REQUIRE( fs->frames[1].hasChannel(Channel::Depth) );
			REQUIRE( fs->frames[1].get<cv::cuda::GpuMat>(Channel::Depth).rows == 720 );
			REQUIRE( fs->frames[1].get<cv::cuda::GpuMat>(Channel::Depth).type() == CV_32F );

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
	//ftl::data::clearRegistry();
}

TEST_CASE( "Receiver sync bugs" ) {
	//ftl::data::make_channel<ftl::rgbd::Camera>(Channel::Calibration, "calibration", ftl::data::StorageMode::PERSISTENT);
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	ftl::data::Pool pool(5,7);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *receiver = ftl::create<Receiver>(cfg, &pool);

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

	ftl::data::Frame dummy = pool.allocate(FrameID(0,0),10);
	dummy.store();
	ftl::rgbd::Frame &state = dummy.cast<ftl::rgbd::Frame>();
	state.setLeft().width = 1280;
	state.setLeft().height = 720;

	stream.select(0, Channel::Colour + Channel::Colour2, true);
	ftl::stream::injectCalibration(&stream, state, 10, 0, 0);

	ftl::timer::start(false);

	SECTION("out of phase packets") {
		cv::cuda::GpuMat m(cv::Size(1280,720), CV_8UC4, cv::Scalar(0));

		bool r = encoder.encode(m, pkt);
		REQUIRE( r );

		int count = 0;
		int64_t ts = 0;
		bool haswrongchan = false;
		auto h = receiver->onFrameSet([&count,&ts,&haswrongchan](const ftl::data::FrameSetPtr& fs) {

			ts = fs->timestamp();
			haswrongchan = fs->frames[0].hasChannel(Channel::ColourHighRes);

			++count;

			return true;
		});

		try { stream.post(spkt, pkt); } catch(...) {}
		spkt.timestamp = 10;
		spkt.channel = Channel::ColourHighRes;
		spkt.flags |= ftl::codecs::kFlagCompleted;
		try { stream.post(spkt, pkt); } catch(...) {}
		spkt.timestamp = 20;
		spkt.channel = Channel::Colour2;
		try { stream.post(spkt, pkt); } catch(...) {}
		spkt.timestamp = 20;
		spkt.channel = Channel::Colour;
		spkt.flags |= ftl::codecs::kFlagCompleted;
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
	//ftl::data::clearRegistry();
}

TEST_CASE( "Receiver non zero buffer" ) {
	//ftl::data::make_channel<ftl::rgbd::Camera>(Channel::Calibration, "calibration", ftl::data::StorageMode::PERSISTENT);
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	ftl::data::Pool pool(5,7);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *receiver = ftl::create<Receiver>(cfg, &pool);

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

	ftl::data::Frame dummy = pool.allocate(FrameID(0,0),10);
	dummy.store();
	ftl::rgbd::Frame &state = dummy.cast<ftl::rgbd::Frame>();
	state.setLeft().width = 1280;
	state.setLeft().height = 720;
	ftl::stream::injectCalibration(&stream, state, 10, 0, 0);

	ftl::timer::start(false);

	SECTION("Fixed buffer delay") {
		cv::cuda::GpuMat m(cv::Size(1280,720), CV_8UC4, cv::Scalar(0));

		bool r = encoder.encode(m, pkt);
		REQUIRE( r );

		int count = 0;
		auto h = receiver->onFrameSet([&count](const ftl::data::FrameSetPtr& fs) {
			++count;

			REQUIRE( fs->timestamp() == 10 );
			REQUIRE( fs->frames.size() == 1 );
			REQUIRE( fs->frames[0].hasChannel(Channel::Colour) );
			REQUIRE( fs->frames[0].get<cv::cuda::GpuMat>(Channel::Colour).rows == 720 );
			REQUIRE( fs->frames[0].get<cv::cuda::GpuMat>(Channel::Colour).type() == CV_8UC4 );

			return true;
		});

		stream.post(spkt, pkt);
		spkt.flags |= ftl::codecs::kFlagCompleted;
		spkt.timestamp += 10;
		spkt.flags |= ftl::codecs::kFlagCompleted;
		stream.post(spkt, pkt);

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
	}

	ftl::timer::stop(true);
	//while (ftl::pool.n_idle() != ftl::pool.size()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
	delete receiver;
	//ftl::data::clearRegistry();
}

TEST_CASE( "Receiver for data channels" ) {
	//ftl::data::make_channel<ftl::rgbd::Camera>(Channel::Calibration, "calibration", ftl::data::StorageMode::PERSISTENT);
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	ftl::data::Pool pool(5,7);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *receiver = ftl::create<Receiver>(cfg, &pool);

	json_t cfg2 = json_t{
		{"$id","ftl://test/2"}
	};
	TestStream stream(cfg2);
	receiver->setStream(&stream);
	receiver->set("frameset_buffer_size", 0);

	ftl::codecs::Packet pkt;
	pkt.codec = codec_t::MSGPACK;
	pkt.bitrate = 255;
	pkt.flags = 0;
	pkt.frame_count = 1;

	ftl::codecs::StreamPacket spkt;
	spkt.version = 4;
	spkt.timestamp = 10;
	spkt.frame_number = 0;
	spkt.channel = Channel::Data;
	spkt.streamID = 0;

	ftl::timer::start(false);

	SECTION("a single data packet") {

		pkt.data.resize(0);
		ftl::util::FTLVectorBuffer buf(pkt.data);
		msgpack::pack(buf, 5.0f);

		spkt.flags |= ftl::codecs::kFlagCompleted;
		stream.post(spkt, pkt);

		int count = 0;
		auto h = receiver->onFrameSet([&count](const ftl::data::FrameSetPtr& fs) {
			++count;

			REQUIRE( fs->timestamp() == 10 );
			REQUIRE( fs->frames.size() == 1 );
			REQUIRE( fs->frames[0].hasChannel(Channel::Data) );
			REQUIRE( fs->frames[0].get<float>(Channel::Data) == 5.0f );

			return true;
		});

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
	}

	SECTION("a single data packet in overall frameset") {

		spkt.frame_number = 255;
		pkt.data.resize(0);
		ftl::util::FTLVectorBuffer buf(pkt.data);
		msgpack::pack(buf, 5.0f);

		stream.post(spkt, pkt);

		// Need to have at least one frame for this to work
		spkt.frame_number = 0;
		spkt.flags |= ftl::codecs::kFlagCompleted;
		stream.post(spkt, pkt);

		int count = 0;
		auto h = receiver->onFrameSet([&count](const std::shared_ptr<ftl::data::FrameSet>& fs) {
			++count;

			REQUIRE( fs->timestamp() == 10 );
			REQUIRE( fs->frames.size() == 1 );
			REQUIRE( fs->hasChannel(Channel::Data) );
			REQUIRE( fs->get<float>(Channel::Data) == 5.0f );

			return true;
		});

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
	}

	SECTION("a single calibration packet") {

		pkt.data.resize(0);
		ftl::util::FTLVectorBuffer buf(pkt.data);
		ftl::rgbd::Camera calib;
		calib.width = 1024;
		msgpack::pack(buf, calib);

		spkt.flags |= ftl::codecs::kFlagCompleted;
		stream.post(spkt, pkt);

		int count = 0;
		auto h = receiver->onFrameSet([&count](const ftl::data::FrameSetPtr& fs) {
			++count;

			REQUIRE( fs->timestamp() == 10 );
			REQUIRE( fs->frames.size() == 1 );
			REQUIRE( fs->frames[0].hasChannel(Channel::Data) );
			REQUIRE( fs->frames[0].get<ftl::rgbd::Camera>(Channel::Data).width == 1024 );

			return true;
		});

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
	}

	SECTION("a single pose packet") {

		pkt.data.resize(0);
		ftl::util::FTLVectorBuffer buf(pkt.data);
		Eigen::Matrix4d pose;
		msgpack::pack(buf, pose);

		spkt.flags |= ftl::codecs::kFlagCompleted;
		stream.post(spkt, pkt);

		int count = 0;
		auto h = receiver->onFrameSet([&count](const std::shared_ptr<ftl::data::FrameSet>& fs) {
			++count;

			REQUIRE( fs->timestamp() == 10 );
			REQUIRE( fs->frames.size() == 1 );
			REQUIRE( fs->frames[0].hasChannel(Channel::Data) );
			fs->frames[0].get<Eigen::Matrix4d>(Channel::Data);

			return true;
		});

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
	}

	ftl::timer::stop(true);
	delete receiver;
}

// TODO: Annoying to test because I need to create valid audio encoding
/*TEST_CASE( "Receiver for audio channels" ) {
	//ftl::data::make_channel<ftl::rgbd::Camera>(Channel::Calibration, "calibration", ftl::data::StorageMode::PERSISTENT);
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	ftl::data::Pool pool(5,7);

	json_t cfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *receiver = ftl::create<Receiver>(cfg, &pool);

	json_t cfg2 = json_t{
		{"$id","ftl://test/2"}
	};
	TestStream stream(cfg2);
	receiver->setStream(&stream);
	receiver->set("frameset_buffer_size", 0);

	ftl::codecs::Packet pkt;
	pkt.codec = codec_t::OPUS;
	pkt.bitrate = 255;
	pkt.flags = 0;
	pkt.frame_count = 1;

	ftl::codecs::StreamPacket spkt;
	spkt.version = 4;
	spkt.timestamp = 10;
	spkt.frame_number = 0;
	spkt.channel = Channel::AudioMono;
	spkt.streamID = 0;

	ftl::timer::start(false);

	SECTION("a single data packet") {

		pkt.data.resize(0);
		ftl::util::FTLVectorBuffer buf(pkt.data);
		msgpack::pack(buf, 5.0f);

		stream.post(spkt, pkt);

		int count = 0;
		auto h = receiver->onFrameSet([&count](const std::shared_ptr<ftl::data::FrameSet>& fs) {
			++count;

			REQUIRE( fs->timestamp() == 10 );
			REQUIRE( fs->frames.size() == 1 );
			REQUIRE( fs->frames[0].hasChannel(Channel::Data) );
			REQUIRE( fs->frames[0].get<float>(Channel::Data) == 5.0f );

			return true;
		});

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
	}

	

	ftl::timer::stop(true);
	delete receiver;
}*/
