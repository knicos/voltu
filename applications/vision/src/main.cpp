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
//#include <ftl/middlebury.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/master.hpp>
#include <nlohmann/json.hpp>
#include <ftl/operators/disparity.hpp>
#include <ftl/operators/detectandtrack.hpp>

#include <ftl/streams/netstream.hpp>
#include <ftl/streams/sender.hpp>

#include <ftl/audio/source.hpp>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

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
	ftl::timer::add(ftl::timer::kTimerMain, [&time_peer,&sync_counter,net](int64_t ts) {
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
	if (paths && (*paths).size() > 0) file = (*paths)[(*paths).size()-1];

	Source *source = nullptr;
	source = ftl::create<Source>(root, "source", net);
	if (file != "") {
		//source->set("uri", file);
		ftl::URI uri(file);
		uri.to_json(source->getConfig());
		source->set("uri", uri.getBaseURI());
	}
	
	ftl::stream::Sender *sender = ftl::create<ftl::stream::Sender>(root, "sender");
	ftl::stream::Net *outstream = ftl::create<ftl::stream::Net>(root, "stream", net);
	outstream->set("uri", outstream->getID());
	outstream->begin();
	sender->setStream(outstream);

	auto *grp = new ftl::rgbd::Group();
	source->setChannel(Channel::Depth);
	grp->addSource(source);

	int stats_count = 0;

	grp->onFrameSet([sender,&stats_count](ftl::rgbd::FrameSet &fs) {
		fs.id = 0;
		sender->post(fs);

		if (--stats_count <= 0) {
			auto [fps,latency] = ftl::rgbd::Builder::getStatistics();
			LOG(INFO) << "Frame rate: " << fps << ", Latency: " << latency;
			stats_count = 20;
		}
		return true;
	});

	// TODO: TEMPORARY
	ftl::audio::Source *audioSrc = ftl::create<ftl::audio::Source>(root, "audio_test");
	audioSrc->onFrameSet([sender](ftl::audio::FrameSet &fs) {
		sender->post(fs);
		return true;
	});
	
	auto pipeline = ftl::config::create<ftl::operators::Graph>(root, "pipeline");
	pipeline->append<ftl::operators::DetectAndTrack>("facedetection")->value("enabled", false);
	pipeline->append<ftl::operators::ArUco>("aruco")->value("enabled", false);
	pipeline->append<ftl::operators::DepthChannel>("depth");  // Ensure there is a depth channel
	grp->addPipeline(pipeline);
	
	net->start();

	LOG(INFO) << "Running...";
	ftl::timer::start(true);
	LOG(INFO) << "Stopping...";
	ctrl.stop();
	
	net->shutdown();

	ftl::pool.stop();

	delete grp;
	delete sender;
	delete outstream;

	//delete source;  // TODO(Nick) Add ftl::destroy
	delete net;
}

int main(int argc, char **argv) {
#ifdef WIN32
	SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
#endif
	std::cout << "FTL Vision Node " << FTL_VERSION_LONG << std::endl;
	auto root = ftl::configure(argc, argv, "vision_default");
	
	std::cout << "Loading..." << std::endl;
	run(root);

	delete root;
	LOG(INFO) << "Terminating with code " << ftl::exit_code;
	LOG(INFO) << "Branch: " << ftl::branch_name;
	return ftl::exit_code;
}

