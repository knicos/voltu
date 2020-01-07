/*
 * Copyright 2019 Nicolas Pope. All rights reserved.
 *
 * See LICENSE.
 */

#define LOGURU_WITH_STREAMS 1
#include <loguru.hpp>
#include <ftl/configuration.hpp>
#include <ctpl_stl.h>
// #include <zlib.h>

#include <string>
#include <map>
#include <vector>
#include <fstream>
#include <thread>

#include <opencv2/opencv.hpp>
#include <ftl/rgbd.hpp>
#include <ftl/middlebury.hpp>
//#include <ftl/display.hpp>
#include <ftl/rgbd/streamer.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/master.hpp>
#include <nlohmann/json.hpp>
#include <ftl/operators/disparity.hpp>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#ifdef WIN32
#pragma comment(lib, "Rpcrt4.lib")
#endif

using ftl::rgbd::Source;
using ftl::rgbd::Camera;
//using ftl::Display;
using ftl::rgbd::Streamer;
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
	source = ftl::create<Source>(root, "source");

	if (file != "") source->set("uri", file);
	
	//Display *display = ftl::create<Display>(root, "display", "local");
	
	Streamer *stream = ftl::create<Streamer>(root, "stream", net);
	stream->add(source);

	auto pipeline = ftl::config::create<ftl::operators::Graph>(root, "pipeline");
	pipeline->append<ftl::operators::DepthChannel>("depth");  // Ensure there is a depth channel
	stream->group()->addPipeline(pipeline);
	
	net->start();

	LOG(INFO) << "Running...";
	/*if (display->hasDisplays()) {
		stream->run();
		while (ftl::running && display->active()) {
			cv::Mat rgb, depth;
			source->getFrames(rgb, depth);
			if (!rgb.empty()) display->render(rgb, depth, source->parameters());
			display->wait(10);
		}
	} else {*/
		stream->run(true);
	//}

	ftl::timer::start(true);

	LOG(INFO) << "Stopping...";
	ctrl.stop();
	stream->stop();
	net->shutdown();

	ftl::pool.stop();

	delete stream;
	//delete display;
	//delete source;  // TODO(Nick) Add ftl::destroy
	delete net;
}

int main(int argc, char **argv) {
#ifdef WIN32
	SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
#endif
	std::cout << "FTL Vision Node " << FTL_VERSION_LONG << std::endl;
	auto root = ftl::configure(argc, argv, "vision_default");
	
	//config["ftl://vision/default"]["paths"] = paths;

	// Choose normal or middlebury modes
	//if (config["middlebury"]["dataset"] == "") {
		std::cout << "Loading..." << std::endl;
		run(root);
	//} else {
	//	ftl::middlebury::test(config);
	//}

	delete root;
	LOG(INFO) << "Terminating with code " << ftl::exit_code;
	LOG(INFO) << "Branch: " << ftl::branch_name;
	return ftl::exit_code;
}

