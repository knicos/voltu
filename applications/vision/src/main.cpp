/*
 * Copyright 2019 Nicolas Pope. All rights reserved.
 *
 * See LICENSE.
 */

#define LOGURU_WITH_STREAMS 1
#include <loguru.hpp>
#include <ftl/configuration.hpp>
#include <ctpl_stl.h>

#include <string>
#include <map>
#include <vector>
#include <fstream>
#include <thread>
#include <set>

#include <opencv2/opencv.hpp>
#include <ftl/rgbd.hpp>
#include <ftl/data/framepool.hpp>
#include <ftl/streams/builder.hpp>
//#include <ftl/middlebury.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/master.hpp>
#include <nlohmann/json.hpp>
#include <ftl/operators/disparity.hpp>
#include <ftl/operators/detectandtrack.hpp>
#include <ftl/operators/clipping.hpp>

#include <ftl/streams/netstream.hpp>
#include <ftl/streams/sender.hpp>
#include <ftl/streams/receiver.hpp>

#include <ftl/audio/source.hpp>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#ifdef HAVE_PYLON
#include <pylon/PylonIncludes.h>
#endif

#ifdef WIN32
#pragma comment(lib, "Rpcrt4.lib")
#endif

using ftl::rgbd::Source;
using ftl::rgbd::Camera;
using ftl::codecs::Channel;
using ftl::net::Universe;
using std::string;
using std::vector;
using std::map;
using std::condition_variable;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using cv::Mat;
using json = nlohmann::json;

static bool quiet = false;

static void run(ftl::Configurable *root) {
	Universe *net = ftl::create<Universe>(root, "net");
	ftl::ctrl::Master ctrl(root, net);

	ftl::timer::setHighPrecision(true);

	if (root->value("time_master", false)) {
		net->bind("time_master", [net]() {
			return net->id();
		});
		LOG(INFO) << "Becoming a time master";
	}

	if (root->get<string>("time_peer")) {
		if (!net->connect(*root->get<string>("time_peer"))->waitConnection()) {
			LOG(ERROR) << "Could not connect to time master";
		} else {
			LOG(INFO) << "Connected to time master";
		}
	}

	auto opt_time_master = net->findOne<ftl::UUID>(string("time_master"));
	ftl::UUID time_peer(0);
	if (opt_time_master) {
		time_peer = *opt_time_master;
		LOG(INFO) << "Found a time master: " << time_peer.to_string();
	}
	int sync_counter = 0;

	// Sync clocks!
	auto timer = ftl::timer::add(ftl::timer::kTimerMain, [&time_peer,&sync_counter,net](int64_t ts) {
		if (sync_counter-- <= 0 && time_peer != ftl::UUID(0) ) {
			sync_counter = 20;
			auto start = std::chrono::high_resolution_clock::now();

			try {
				net->asyncCall<int64_t>(time_peer, "__ping__", [start](const int64_t &mastertime) {
					auto elapsed = std::chrono::high_resolution_clock::now() - start;
					int64_t latency = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
					auto clock_adjust = mastertime + ((latency+500)/2000) - ftl::timer::get_time();

					//LOG(INFO) << "LATENCY: " << float(latency)/1000.0f << "ms";

					if (clock_adjust != 0) {
						LOG(INFO) << "Clock adjustment: " << clock_adjust << ", latency=" << float(latency)/2000.0f << "ms";
						ftl::timer::setClockAdjustment(clock_adjust);
					}
				});
			} catch (const std::exception &e) {
				LOG(ERROR) << "Ping failed, could not time sync: " << e.what();
				return true;
			}
		}
		return true;
	});

	auto paths = root->get<vector<string>>("paths");
	string file = "";

	for (auto &x : *paths) {
		if (x != "") {
			ftl::URI uri(x);
			if (uri.isValid()) {
				switch (uri.getScheme()) {
				case ftl::URI::SCHEME_WS		:
				case ftl::URI::SCHEME_TCP		: net->connect(x)->waitConnection(); break;
				case ftl::URI::SCHEME_DEVICE	:
				case ftl::URI::SCHEME_FILE		: file = x; break;
				default: break;
				}
			}
		}
	}

	if (file != "") {
		ftl::URI uri(file);
		uri.to_json(root->getConfig()["source"]);
	}
	Source *source = nullptr;
	source = ftl::create<Source>(root, "source");

	ftl::stream::Sender *sender = ftl::create<ftl::stream::Sender>(root, "sender");
	ftl::stream::Net *outstream = ftl::create<ftl::stream::Net>(root, "stream", net);
	outstream->set("uri", root->value("uri", outstream->getID()));
	outstream->begin();
	sender->setStream(outstream);

	ftl::audio::Source *audioSrc = ftl::create<ftl::audio::Source>(root, "audio");

	ftl::data::Pool pool(root->value("mempool_min", 2),root->value("mempool_max", 5));
	auto *creator = new ftl::streams::IntervalSourceBuilder(&pool, 0, {source, audioSrc});
	std::shared_ptr<ftl::streams::BaseBuilder> creatorptr(creator);

	ftl::stream::Receiver *receiver = ftl::create<ftl::stream::Receiver>(root, "receiver", &pool);
	receiver->setStream(outstream);
	receiver->registerBuilder(creatorptr);

	// Which channels should be encoded
	std::set<Channel> encodable;
	std::set<Channel> previous_encodable;

	// Send channels on flush
	auto flushhandle = pool.onFlushSet([sender,&encodable](ftl::data::FrameSet &fs, ftl::codecs::Channel c) {
		//if (c != Channel::EndFrame && !fs.test(ftl::data::FSFlag::AUTO_SEND)) return true;

		// Always send data channels
		if ((int)c >= 32) sender->post(fs, c);
		else {
			// Only encode some of the video channels
			if (encodable.count(c)) {
				sender->post(fs, c);
			} else {
				sender->post(fs, c, true);
			}
		}
		return true;
	});

	int stats_count = 0;
	int frames = 0;
	float latency = 0.0f;
	int64_t stats_time = 0;

	root->on("quiet", quiet, false);

	auto *pipeline = ftl::config::create<ftl::operators::Graph>(root, "pipeline");
	pipeline->append<ftl::operators::ArUco>("aruco")->value("enabled", false);
	pipeline->append<ftl::operators::DetectAndTrack>("facedetection")->value("enabled", false);
	pipeline->append<ftl::operators::DepthChannel>("depth");  // Ensure there is a depth channel
	//pipeline->append<ftl::operators::ClipScene>("clipping")->value("enabled", false);
	pipeline->restore("vision_pipeline", { "clipping" });

	auto h = creator->onFrameSet([sender,outstream,&stats_count,&latency,&frames,&stats_time,pipeline,&encodable,&previous_encodable](const ftl::data::FrameSetPtr &fs) {

		// Decide what to encode here, based upon what remote users select
		const auto sel = outstream->selectedNoExcept(fs->frameset());
		encodable.clear();
		encodable.insert(sel.begin(), sel.end());

		// Only allow the two encoders to exist, remove the rest
		int max_encodeable = sender->value("max_encodeable", 2);

		if (encodable.size() > max_encodeable) {
			auto enciter = encodable.begin();
			std::advance(enciter, max_encodeable);
			encodable.erase(enciter, encodable.end());
		}

		// This ensures we cleanup other encoders
		if (encodable != previous_encodable) sender->resetEncoders(fs->frameset());
		previous_encodable = encodable;

		fs->set(ftl::data::FSFlag::AUTO_SEND);

		bool did_pipe = pipeline->apply(*fs, *fs, [fs,&frames,&latency]() {
			if (fs->hasAnyChanged(Channel::Depth)) fs->flush(Channel::Depth);
			++frames;
			latency += float(ftl::timer::get_time() - fs->timestamp());
			const_cast<ftl::data::FrameSetPtr&>(fs).reset();
		});

		if (!did_pipe) {
			LOG(WARNING) << "Depth pipeline drop: " << fs->timestamp();
			fs->firstFrame().message(ftl::data::Message::Warning_PIPELINE_DROP, "Depth pipeline drop");
		}


		// Do some encoding (eg. colour) whilst pipeline runs
		ftl::pool.push([fs,&stats_count,&latency,&frames,&stats_time](int id){
			if (fs->hasAnyChanged(Channel::Audio)) {
				fs->flush(ftl::codecs::Channel::Audio);
			}

			// Make sure upload has completed.
			cudaSafeCall(cudaEventSynchronize(fs->frames[0].uploadEvent()));
			// TODO: Try depth pipeline again here if failed first time.
			fs->flush(ftl::codecs::Channel::Colour);

			const_cast<ftl::data::FrameSetPtr&>(fs).reset();

			if (!quiet && --stats_count <= 0) {
				latency /= float(frames);
				int64_t nowtime = ftl::timer::get_time();
				stats_time = nowtime - stats_time;
				float fps = float(frames) / (float(stats_time) / 1000.0f);
				LOG(INFO) << "Frame rate: " << fps << ", Latency: " << latency;
				stats_count = 20;
				frames = 0;
				latency = 0.0f;
				stats_time = nowtime;
			}
		});

		const_cast<ftl::data::FrameSetPtr&>(fs).reset();

		return true;
	});

	// Start the timed generation of frames
	creator->start();

	// Only now start listening for connections
	net->start();

	LOG(INFO) << "Running...";
	ftl::timer::start(true);  // Blocks
	LOG(INFO) << "Stopping...";
	ctrl.stop();

	ftl::config::save();

	net->shutdown();

	ftl::pool.stop();

	delete source;
	delete receiver;
	delete sender;
	delete pipeline;
	delete audioSrc;
	delete outstream;

	delete net;
}

int main(int argc, char **argv) {
#ifdef HAVE_PYLON
	Pylon::PylonAutoInitTerm autoInitTerm;
#endif

#ifdef WIN32
	SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
#endif
	std::cout << "FTL Vision Node " << FTL_VERSION_LONG << std::endl;

	try {
		auto root = ftl::configure(argc, argv, "vision_default", {
			"uri",
			"fps",
			"time_master",
			"time_peer",
			"quiet"
		});

		root->value("restart", 0);

		// Allow config controlled restart
		root->on("restart", [root]() {
			auto val = root->get<int>("restart");
			if (val) {
				ftl::exit_code = *val;
				ftl::running = false;
			}
		});

		// Use other GPU if available.
		//ftl::cuda::setDevice(ftl::cuda::deviceCount()-1);

		std::cout << "Loading..." << std::endl;
		run(root);

		delete root;

		ftl::config::cleanup();

		LOG(INFO) << "Terminating with code " << ftl::exit_code;
		LOG(INFO) << "Branch: " << ftl::branch_name;
	} catch (const std::exception &e) {
		LOG(ERROR) << "Main Exception: " << e.what();
		return -1;
	}

	return ftl::exit_code;
}

