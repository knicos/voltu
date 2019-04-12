/*
 * Copyright 2019 Nicolas Pope. All rights reserved.
 *
 * See LICENSE.
 */

#include <glog/logging.h>
#include <ftl/config.h>
#include <ctpl_stl.h>

#include <string>
#include <map>
#include <vector>
#include <fstream>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <opencv2/opencv.hpp>
#include <ftl/local.hpp>
#include <ftl/synched.hpp>
#include <ftl/calibrate.hpp>
#include <ftl/disparity.hpp>
#include <ftl/middlebury.hpp>
#include <ftl/display.hpp>
#include <ftl/streamer.hpp>
#include <ftl/net/universe.hpp>
#include <nlohmann/json.hpp>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#ifdef WIN32
#pragma comment(lib, "Rpcrt4.lib")
#endif

using ftl::Calibrate;
using ftl::LocalSource;
using ftl::Display;
using ftl::Streamer;
using ftl::Disparity;
using ftl::SyncSource;
using ftl::net::Universe;
using std::string;
using std::vector;
using std::map;
using std::condition_variable;
using std::mutex;
using std::unique_lock;
using cv::Mat;
using json = nlohmann::json;
using std::ifstream;

// Store loaded configuration
static json config;

/**
 * Find and load a JSON configuration file
 */
static bool findConfiguration(const string &file) {
	ifstream i;
	
	if (file != "") i.open(file);
	if (!i.is_open()) i.open("./config.json");
	if (!i.is_open()) i.open(FTL_LOCAL_CONFIG_ROOT "/config.json");
	if (!i.is_open()) i.open(FTL_GLOBAL_CONFIG_ROOT "/config.json");
	if (!i.is_open()) return false;
	i >> config;
	return true;
}

/**
 * Generate a map from command line option to value
 */
map<string, string> read_options(char ***argv, int *argc) {
	map<string, string> opts;

	while (*argc > 0) {
		string cmd((*argv)[0]);
		if (cmd[0] != '-') break;

		size_t p;
		if ((p = cmd.find("=")) == string::npos) {
			opts[cmd.substr(2)] = "true";
		} else {
			opts[cmd.substr(2, p-2)] = cmd.substr(p+1);
		}

		(*argc)--;
		(*argv)++;
	}

	return opts;
}

/**
 * Put command line options into json config. If config element does not exist
 * or is of a different type then report an error.
 */
static void process_options(const map<string, string> &opts) {
	for (auto opt : opts) {
		if (opt.first == "config") continue;

		if (opt.first == "version") {
			std::cout << "FTL Vision Node - v" << FTL_VERSION << std::endl;
			std::cout << FTL_VERSION_LONG << std::endl;
			exit(0);
		}

		try {
			auto ptr = json::json_pointer("/"+opt.first);
			// TODO(nick) Allow strings without quotes
			auto v = json::parse(opt.second);
			if (v.type() != config.at(ptr).type()) {
				LOG(ERROR) << "Incorrect type for argument " << opt.first;
				continue;
			}
			config.at(ptr) = v;
		} catch(...) {
			LOG(ERROR) << "Unrecognised option: " << opt.first;
		}
	}
}

static void run(const string &file) {
	ctpl::thread_pool pool(2);
	Universe net(config["net"]);

	LocalSource *lsrc;
	if (file != "") {
		// Load video file
		lsrc = new LocalSource(file, config["source"]);
	} else {
		// Use cameras
		lsrc = new LocalSource(config["source"]);
	}

	auto sync = new SyncSource();  // TODO(nick) Pass protocol object
	// Add any remote channels
	/* for (auto c : OPTION_channels) {
		sync->addChannel(c);
	} */

	// Perform or load calibration intrinsics + extrinsics
	Calibrate calibrate(lsrc, config["calibration"]);
	if (config["calibrate"]) calibrate.recalibrate();
	if (!calibrate.isCalibrated()) LOG(WARNING) << "Cameras are not calibrated!";
	
	cv::Mat Q_32F;
	calibrate.getQ().convertTo(Q_32F, CV_32F);
	
	// Allow remote users to access camera calibration matrix
	net.bind(string("ftl://utu.fi/")+(string)config["stream"]["name"]+string("/rgb-d/calibration"), [&calibrate,Q_32F]() -> vector<unsigned char> {
		vector<unsigned char> buf;
		buf.resize(Q_32F.step*Q_32F.rows);
		LOG(INFO) << "Calib buf size = " << buf.size();
		memcpy(buf.data(), Q_32F.data, buf.size());
		return buf;
	});

    // Choose and configure disparity algorithm
    auto disparity = Disparity::create(config["disparity"]);
    if (!disparity) LOG(FATAL) << "Unknown disparity algorithm : " << config["disparity"];

	Mat l, r, disp;
	Mat pl, pdisp;

	Display display(config["display"]);
	display.setCalibration(Q_32F);
	
	Streamer stream(net, config["stream"]);

	while (display.active()) {
		mutex m;
		condition_variable cv;
		int jobs = 0;

		pool.push([&](int id) {
			auto start = std::chrono::high_resolution_clock::now();
			// Read calibrated images.
			calibrate.rectified(l, r);

			// Feed into sync buffer and network forward
			sync->feed(ftl::LEFT, l, lsrc->getTimestamp());
			sync->feed(ftl::RIGHT, r, lsrc->getTimestamp());

			// Read back from buffer
			sync->get(ftl::LEFT, l);
			sync->get(ftl::RIGHT, r);

			// TODO(nick) Pipeline this
		    disparity->compute(l, r, disp);

			unique_lock<mutex> lk(m);
			jobs++;
			lk.unlock();
			cv.notify_one();

			std::chrono::duration<double> elapsed =
				std::chrono::high_resolution_clock::now() - start;
			LOG(INFO) << "Disparity in " << elapsed.count() << "s";
		});

		pool.push([&](int id) {
			auto start = std::chrono::high_resolution_clock::now();
			if (pl.rows != 0) stream.send(pl, pdisp);
			unique_lock<mutex> lk(m);
			jobs++;
			lk.unlock();
			cv.notify_one();

			std::chrono::duration<double> elapsed =
				std::chrono::high_resolution_clock::now() - start;
			LOG(INFO) << "Stream in " << elapsed.count() << "s";
		});

		// Send RGB+Depth images for local rendering
		display.render(l, disp);
		display.wait(1);

		unique_lock<mutex> lk(m);
		cv.wait(lk, [&jobs]{return jobs == 2;});

		l.copyTo(pl);
		disp.copyTo(pdisp);
	}
}

int main(int argc, char **argv) {
	argc--;
	argv++;

	// Process Arguments
	auto options = read_options(&argv, &argc);
	if (!findConfiguration(options["config"])) {
		LOG(FATAL) << "Could not find any configuration!";
	}
	process_options(options);

	// Choose normal or middlebury modes
	if (config["middlebury"]["dataset"] == "") {
		run((argc > 0) ? argv[0] : "");
	} else {
		ftl::middlebury::test(config);
	}
}

