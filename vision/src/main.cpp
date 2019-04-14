/*
 * Copyright 2019 Nicolas Pope. All rights reserved.
 *
 * See LICENSE.
 */

#include <glog/logging.h>
#include <ftl/configuration.hpp>
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
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::mutex;
using std::unique_lock;
using cv::Mat;
using json = nlohmann::json;
using ftl::config;


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
	net.bind(string("ftl://utu.fi/")+(string)config["stream"]["name"]+string("/rgb-d/calibration"), [Q_32F]() -> vector<unsigned char> {
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
			//sync->feed(ftl::LEFT, l, lsrc->getTimestamp());
			//sync->feed(ftl::RIGHT, r, lsrc->getTimestamp());

			// Read back from buffer
			//sync->get(ftl::LEFT, l);
			//sync->get(ftl::RIGHT, r);

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
		if (pl.rows > 0) display.render(pl, pdisp);
		display.wait(1);

		unique_lock<mutex> lk(m);
		cv.wait(lk, [&jobs]{return jobs == 2;});

		l.copyTo(pl);
		disp.copyTo(pdisp);
	}
}

int main(int argc, char **argv) {
	auto paths = ftl::configure(argc, argv, "vision");

	// Choose normal or middlebury modes
	if (config["middlebury"]["dataset"] == "") {
		run((paths.size() > 0) ? paths[0] : "");
	} else {
		ftl::middlebury::test(config);
	}
}

