#include "catch.hpp"

#include <ftl/streams/receiver.hpp>
#include <ftl/streams/sender.hpp>
#include <ftl/rgbd/frame.hpp>

#include <nlohmann/json.hpp>

#include <loguru.hpp>

using ftl::codecs::definition_t;
using ftl::codecs::codec_t;
using ftl::stream::Receiver;
using ftl::stream::Sender;
using ftl::data::Frame;
using ftl::data::FrameSet;
using ftl::codecs::Channel;
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
};

class DummySource : public ftl::data::DiscreteSource {
	public:

	bool capture(int64_t ts) override { return true; }
	bool retrieve(ftl::data::Frame &f) override { return true; }
};


TEST_CASE( "Send and receiver via encoding" ) {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	ftl::data::Pool pool(5,7);

	json_t rcfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *receiver = ftl::create<Receiver>(rcfg, &pool);

	json_t scfg = json_t{
		{"$id","ftl://test/2"}
	};
	auto *sender = ftl::create<Sender>(scfg);

	json_t cfg2 = json_t{
		{"$id","ftl://test/3"}
	};

	TestStream stream(cfg2);

	receiver->setStream(&stream);
	receiver->set("frameset_buffer_size", 0);
	sender->setStream(&stream);

	ftl::timer::start(false);

	SECTION("a single data only frame") {
		stream.select(0, {Channel::Control}, true);

		Frame f = pool.allocate(ftl::data::FrameID(0,0), 1000);
		f.store();
		auto fsptr = FrameSet::fromFrame(f);
		FrameSet &fs = *fsptr;

		fs.frames[0].create<int>(Channel::Control) = 57;

		int count = 0;
		ftl::data::FrameSetPtr result;
		auto h = receiver->onFrameSet([&count,&result](const ftl::data::FrameSetPtr &fs) {
			count++;
			result = fs;
			return true;
		});

		sender->post(fs, Channel::Control);
		sender->post(fs, Channel::EndFrame);

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 1 );
		REQUIRE( result->frames[0].has(Channel::Control) );
		REQUIRE( result->frames[0].getChangeType(Channel::Control) == ftl::data::ChangeType::COMPLETED );
		REQUIRE( result->frames[0].get<int>(Channel::Control) == 57 );
	}

	ftl::timer::stop(true);

	delete receiver;
	delete sender;
}

TEST_CASE( "Multi-thread stability testing" ) {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	ftl::data::Pool pool(5,7);

	json_t rcfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *receiver = ftl::create<Receiver>(rcfg, &pool);

	json_t scfg = json_t{
		{"$id","ftl://test/2"}
	};
	auto *sender = ftl::create<Sender>(scfg);

	json_t cfg2 = json_t{
		{"$id","ftl://test/3"}
	};

	TestStream stream(cfg2);

	receiver->setStream(&stream);
	receiver->set("frameset_buffer_size", 0);
	sender->setStream(&stream);
	sender->resetSender();  // FIXME: Why is this needed?

	ftl::timer::setInterval(20);
	ftl::timer::start(false);

	SECTION("One frame, two channel") {
		stream.select(0, {Channel::Colour}, true);

		auto h1 = pool.onFlushSet([sender](ftl::data::FrameSet &fs, ftl::codecs::Channel c) {
			if (!fs.test(ftl::data::FSFlag::AUTO_SEND)) return true;

			//LOG(INFO) << "FLUSH: " << fs.timestamp() << ", " << int(c);
			sender->post(fs, c);
			return true;
		});

		int count = 0;
		ftl::data::FrameSetPtr result = nullptr;
		auto h = receiver->onFrameSet([&count,&result](const ftl::data::FrameSetPtr &fs) {
			LOG(INFO) << "FS RECV: " << fs->timestamp();
			count++;
			if (result) REQUIRE( result->timestamp() <= fs->timestamp()-20 );
			REQUIRE( fs->frames.size() == 1 );
			REQUIRE( fs->frames[0].hasChannel(Channel::Colour) );
			result = fs;
			return true;
		});

		auto h2 = ftl::timer::add(ftl::timer::timerlevel_t::kTimerMain, [&pool](int64_t ts) {
			Frame f = pool.allocate(ftl::data::FrameID(0,0), ts);
			f.store();
			auto &mat = f.create<cv::cuda::GpuMat>(Channel::Colour);
			mat.create(480, 640, CV_8UC4);
			mat.setTo(cv::Scalar(0,0,0,0));

			auto &calib = f.cast<ftl::rgbd::Frame>().setLeft();
			calib.width = 640;
			calib.height = 480;

			auto fsptr = FrameSet::fromFrame(f);
			FrameSet &fs = *fsptr;
			fs.set(ftl::data::FSFlag::AUTO_SEND);
			fsptr->flush(Channel::Calibration);
			ftl::pool.push([fsptr](int id) { fsptr->flush(Channel::Colour); });
			return true;
		});

		int i=1000;
		while (i-- > 0 && count < 100) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count >= 100 );
		
	}

	SECTION("Two frame, two channel") {
		stream.select(0, {Channel::Colour}, true);

		auto h1 = pool.onFlushSet([sender](ftl::data::FrameSet &fs, ftl::codecs::Channel c) {
			if (!fs.test(ftl::data::FSFlag::AUTO_SEND)) return true;

			//LOG(INFO) << "FLUSH: " << fs.timestamp() << ", " << int(c) << ", " << fs.frames[0].source();
			sender->post(fs, c);
			return true;
		});

		int count = 0;
		ftl::data::FrameSetPtr result = nullptr;
		auto h = receiver->onFrameSet([&count,&result](const ftl::data::FrameSetPtr &fs) {
			LOG(INFO) << "FS RECV: " << fs->timestamp();
			count++;
			if (result) {
				REQUIRE( result->timestamp() <= fs->timestamp()-20 );
				//REQUIRE( fs->frames.size() == 2 );
				REQUIRE( fs->isComplete() );
				REQUIRE( fs->frames[0].hasChannel(Channel::Colour) );
				if (fs->frames.size() > 1) REQUIRE( fs->frames[1].hasChannel(Channel::Colour) );
			}
			result = fs;
			return true;
		});

		ftl::data::Pool pool2(5,7);

		auto h2 = ftl::timer::add(ftl::timer::timerlevel_t::kTimerMain, [&pool,&pool2](int64_t ts) {
			ftl::pool.push([&pool, ts](int id) {
				Frame f = pool.allocate(ftl::data::FrameID(0,0), ts);
				f.store();
				auto &mat = f.create<cv::cuda::GpuMat>(Channel::Colour);
				mat.create(480, 640, CV_8UC4);
				mat.setTo(cv::Scalar(0,0,0,0));

				auto &calib = f.cast<ftl::rgbd::Frame>().setLeft();
				calib.width = 640;
				calib.height = 480;

				auto fsptr = FrameSet::fromFrame(f);
				FrameSet &fs = *fsptr;
				fs.set(ftl::data::FSFlag::AUTO_SEND);
				fsptr->flush(Channel::Calibration);
				ftl::pool.push([fsptr](int id) { fsptr->flush(Channel::Colour); });
			});

			ftl::pool.push([&pool, ts](int id) {
				Frame f = pool.allocate(ftl::data::FrameID(0,1), ts);
				f.store();
				auto &mat = f.create<cv::cuda::GpuMat>(Channel::Colour);
				mat.create(480, 640, CV_8UC4);
				mat.setTo(cv::Scalar(0,0,0,0));

				auto &calib = f.cast<ftl::rgbd::Frame>().setLeft();
				calib.width = 640;
				calib.height = 480;

				auto fsptr = FrameSet::fromFrame(f);
				FrameSet &fs = *fsptr;
				fs.set(ftl::data::FSFlag::AUTO_SEND);
				fsptr->flush(Channel::Calibration);
				ftl::pool.push([fsptr](int id) { fsptr->flush(Channel::Colour); });
			});
			return true;
		});

		int i=1000;
		while (i-- > 0 && count < 100) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count >= 100 );
		
	}

	LOG(INFO) << "DONE";

	ftl::timer::reset();
	ftl::timer::setInterval(50);
	ftl::pool.clear_queue();
	while (ftl::pool.n_idle() != ftl::pool.size()) std::this_thread::sleep_for(std::chrono::milliseconds(10));

	delete receiver;
	delete sender;
}

TEST_CASE( "Response via loopback" ) {
	json_t global = json_t{{"$id","ftl://test"}};
	ftl::config::configure(global);

	ftl::data::Pool pool(5,7);

	json_t rcfg = json_t{
		{"$id","ftl://test/1"}
	};
	auto *receiver = ftl::create<Receiver>(rcfg, &pool);

	json_t cfg2 = json_t{
		{"$id","ftl://test/3"}
	};

	TestStream stream(cfg2);

	receiver->setStream(&stream);
	receiver->set("frameset_buffer_size", 0);

	auto hh = pool.onFlush([receiver](ftl::data::Frame &f, Channel c) {
		receiver->loopback(f, c);
		return true;
	});

	ftl::timer::start(false);

	SECTION("a single data only frame") {
		DummySource source;

		stream.select(0, {Channel::Control}, true);

		auto *builder = new ftl::streams::ManualSourceBuilder(&pool, 0, &source);
		builder->setFrameRate(10000);
		std::shared_ptr<ftl::streams::BaseBuilder> builderptr(builder);
		receiver->registerBuilder(builderptr);

		int count = 0;
		ftl::data::FrameSetPtr result;
		auto h = receiver->onFrameSet([&count,&result](const ftl::data::FrameSetPtr &fs) {
			count++;
			result = fs;
			auto response = fs->frames[0].response();
			response.create<int>(Channel::Control) = count;
			return true;
		});

		builder->tick();
		builder->tick();

		int i=10;
		while (i-- > 0 && count < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));

		REQUIRE( count == 2 );
		REQUIRE( result->frames[0].has(Channel::Control) );
		REQUIRE( result->frames[0].getChangeType(Channel::Control) == ftl::data::ChangeType::FOREIGN );
		REQUIRE( result->frames[0].get<int>(Channel::Control) == 1 );
	}

	ftl::timer::stop(true);

	delete receiver;
}
