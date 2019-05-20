/*
 * Copyright 2019 Nicolas Pope. All rights reserved.
 *
 * See LICENSE.
 */

#include <glog/logging.h>
#include <ftl/configuration.hpp>
#include <ctpl_stl.h>
// #include <zlib.h>

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

	LOG(INFO) << "Net started.";

	LocalSource *lsrc;
	if (ftl::is_video(file)) {
		// Load video file
		LOG(INFO) << "Using video file...";
		lsrc = new LocalSource(file, config["source"]);
	} else if (file != "") {
		auto vid = ftl::locateFile("video.mp4");
		if (!vid) {
			LOG(FATAL) << "No video.mp4 file found in provided paths";
		} else {
			LOG(INFO) << "Using test directory...";
			lsrc = new LocalSource(*vid, config["source"]);
		}
	} else {
		// Use cameras
		LOG(INFO) << "Using cameras...";
		lsrc = new LocalSource(config["source"]);
	}

	//auto sync = new SyncSource();  // TODO(nick) Pass protocol object
	// Add any remote channels
	/* for (auto c : OPTION_channels) {
		sync->addChannel(c);
	} */

	LOG(INFO) << "Locating calibration...";

	// Perform or load calibration intrinsics + extrinsics
	Calibrate calibrate(lsrc, config["calibration"]);
	if (config["calibrate"]) calibrate.recalibrate();
	if (!calibrate.isCalibrated()) LOG(WARNING) << "Cameras are not calibrated!";
	else LOG(INFO) << "Calibration initiated.";
	
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

	Mat l, r, disp;
	bool grabbed = false;
	mutex datam;
	condition_variable datacv;

	// Wait for grab message to sync camera capture
	net.bind("grab", [&calibrate,&l,&r,&datam,&datacv,&grabbed]() -> void {
		unique_lock<mutex> datalk(datam);
		if (grabbed) return;
		calibrate.rectified(l, r);
		grabbed = true;
		datacv.notify_one();
	});

    // Choose and configure disparity algorithm
    auto disparity = Disparity::create(config["disparity"]);
    if (!disparity) LOG(FATAL) << "Unknown disparity algorithm : " << config["disparity"];

	Mat pl, pdisp;
	vector<unsigned char> rgb_buf;
	vector<unsigned char> d_buf;
	string uri = string("ftl://utu.fi/")+(string)config["stream"]["name"]+string("/rgb-d");

	Display display(config["display"], "local");
	
	Streamer stream(net, config["stream"]);

	LOG(INFO) << "Beginning capture...";

	while (display.active()) {
		mutex m;
		condition_variable cv;
		int jobs = 0;

		// Fake grab if no peers to allow visualisation locally
		if (net.numberOfPeers() == 0) {
			grabbed = true;
			calibrate.rectified(l, r);
		}

		// Pipeline for disparity
		pool.push([&](int id) {
			// Wait for image grab
			unique_lock<mutex> datalk(datam);
			datacv.wait(datalk, [&grabbed](){ return grabbed; });
			grabbed = false;

			auto start = std::chrono::high_resolution_clock::now();
		    disparity->compute(l, r, disp);
			datalk.unlock();

			unique_lock<mutex> lk(m);
			jobs++;
			lk.unlock();
			cv.notify_one();

			std::chrono::duration<double> elapsed =
				std::chrono::high_resolution_clock::now() - start;
			LOG(INFO) << "Disparity in " << elapsed.count() << "s";
		});

		// Pipeline for stream compression
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
		if (pl.rows > 0) display.render(pl, r, pdisp, Q_32F);
		display.wait(1);

		// Wait for both pipelines to complete
		unique_lock<mutex> lk(m);
		cv.wait(lk, [&jobs]{return jobs == 2;});

		// Store previous frame for next stage compression
		l.copyTo(pl);
		disp.copyTo(pdisp);

		//net.publish(uri, rgb_buf, d_buf);
	}

	LOG(INFO) << "Finished.";
}

int main(int argc, char **argv) {
	auto paths = ftl::configure(argc, argv, "vision");
	
	config["paths"] = paths;

	// Choose normal or middlebury modes
	if (config["middlebury"]["dataset"] == "") {
		run((paths.size() > 0) ? paths[0] : "");
	} else {
		ftl::middlebury::test(config);
	}
}

