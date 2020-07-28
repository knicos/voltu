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

	ftl::ctrl::Master ctrl(root, net);

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
						LOG(INFO) << "Clock adjustment: " << clock_adjust << ", latency=" << float(latency)/1000.0f << "ms";
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

	ftl::audio::Source *audioSrc = ftl::create<ftl::audio::Source>(root, "audio_test");

	ftl::data::Pool pool(root->value("mempool_min", 2),root->value("mempool_max", 5));
	auto *creator = new ftl::streams::IntervalSourceBuilder(&pool, 0, {source, audioSrc});
	std::shared_ptr<ftl::streams::BaseBuilder> creatorptr(creator);

	ftl::stream::Receiver *receiver = ftl::create<ftl::stream::Receiver>(root, "receiver", &pool);
	receiver->setStream(outstream);
	receiver->registerBuilder(creatorptr);

	// Which channels should be encoded
	std::unordered_set<Channel> encodable;

	// Send channels on flush
	auto flushhandle = pool.onFlushSet([sender,&encodable](ftl::data::FrameSet &fs, ftl::codecs::Channel c) {
		if ((int)c >= 32) sender->post(fs, c);
		else {
			if (encodable.count(c)) {
				sender->post(fs, c);
			} else {
				//switch (c) {
				//case Channel::Colour		:
				//case Channel::Colour2		:
				//case Channel::Depth			: 
				sender->post(fs, c, true); //break;
				//default						: sender->fakePost(fs, c);
				//}
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
	pipeline->append<ftl::operators::DetectAndTrack>("facedetection")->value("enabled", false);
	pipeline->append<ftl::operators::ArUco>("aruco")->value("enabled", false);
	pipeline->append<ftl::operators::DepthChannel>("depth");  // Ensure there is a depth channel

	bool busy = false;

	auto h = creator->onFrameSet([sender,outstream,&stats_count,&latency,&frames,&stats_time,pipeline,&busy,&encodable](const ftl::data::FrameSetPtr &fs) {
		if (busy) {
			LOG(WARNING) << "Depth pipeline drop: " << fs->timestamp();
			fs->firstFrame().message(ftl::data::Message::Warning_PIPELINE_DROP, "Depth pipeline drop");
			return true;
		}
		busy = true;

		encodable.clear();
		// Decide what to encode here.
		const auto sel = outstream->selectedNoExcept(fs->frameset());
		std::vector<Channel> sortedsel(sel.begin(), sel.end());
		std::sort(sortedsel.begin(),sortedsel.end());

		if (sortedsel.size() > 0) encodable.emplace(sortedsel[0]);
		if (sortedsel.size() > 1) encodable.emplace(sortedsel[1]);

		// Only allow the two encoders to exist
		// This ensures we cleanup other encoders
		sender->setActiveEncoders(fs->frameset(), encodable);

		// Do all processing in another thread... only if encoding of depth
		//if (encodable.find(Channel::Depth) != encodable.end()) {
			ftl::pool.push([sender,&stats_count,&latency,&frames,&stats_time,pipeline,&busy,fs](int id) mutable {
				// Do pipeline here...
				pipeline->apply(*fs, *fs);

				++frames;
				latency += float(ftl::timer::get_time() - fs->timestamp());

				// Destruct frameset as soon as possible to send the data...
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

				busy = false;
			});
		//} else {
			//LOG(INFO) << "NOT DOING DEPTH";
		//	sender->forceAvailable(*fs, Channel::Depth);
		//	busy = false;
		//}

		// Lock colour right now to encode in parallel
		fs->flush(ftl::codecs::Channel::Colour);
		fs->flush(ftl::codecs::Channel::AudioStereo);

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
	return ftl::exit_code;
}

