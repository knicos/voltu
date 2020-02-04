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
#include <ftl/middlebury.hpp>
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
	ftl::ctrl::Master ctrl(root, net);

	auto paths = root->get<vector<string>>("paths");
	string file = "";
	if (paths && (*paths).size() > 0) file = (*paths)[(*paths).size()-1];

	Source *source = nullptr;
	source = ftl::create<Source>(root, "source", net);
	if (file != "") source->set("uri", file);
	
	ftl::stream::Sender *sender = ftl::create<ftl::stream::Sender>(root, "sender");
	ftl::stream::Net *outstream = ftl::create<ftl::stream::Net>(root, "stream", net);
	outstream->set("uri", outstream->getID());
	outstream->begin();
	sender->setStream(outstream);

	auto *grp = new ftl::rgbd::Group();
	source->setChannel(Channel::Depth);
	grp->addSource(source);

	grp->onFrameSet([sender](ftl::rgbd::FrameSet &fs) {
		fs.id = 0;
		sender->post(fs);
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

