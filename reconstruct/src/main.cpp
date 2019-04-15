/*
 * Copyright 2019 Nicolas Pope. All rights reserved.
 *
 * See LICENSE.
 */

#include <glog/logging.h>
#include <ftl/config.h>
#include <ftl/configuration.hpp>
// #include <zlib.h>
// #include <lz4.h>

#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <ftl/net/universe.hpp>
#include <ftl/display.hpp>
#include <nlohmann/json.hpp>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#ifdef WIN32
#pragma comment(lib, "Rpcrt4.lib")
#endif

using ftl::net::Universe;
using ftl::Display;
using ftl::config;
using std::string;
using std::vector;
using cv::Mat;
using json = nlohmann::json;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::mutex;
using std::unique_lock;

static void run() {
	Universe net(config["net"]);
	Mat rgb, depth, Q;
	mutex m;
	
	Display disp(config["display"]);
	
	// Make sure connections are complete
	sleep_for(milliseconds(500));

	// TODO(nick) Allow for many sources
	net.subscribe(config["source"], [&rgb,&m,&depth](const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
		unique_lock<mutex> lk(m);
		cv::imdecode(jpg, cv::IMREAD_COLOR, &rgb);
		//LOG(INFO) << "Received JPG : " << rgb.cols;
		
		depth = Mat(rgb.size(), CV_16UC1);
		
		/*z_stream infstream;
		infstream.zalloc = Z_NULL;
		infstream.zfree = Z_NULL;
		infstream.opaque = Z_NULL;
		// setup "b" as the input and "c" as the compressed output
		infstream.avail_in = (uInt)d.size(); // size of input
		infstream.next_in = (Bytef *)d.data(); // input char array
		infstream.avail_out = (uInt)depth.step*depth.rows; // size of output
		infstream.next_out = (Bytef *)depth.data; // output char array
		 
		// the actual DE-compression work.
		inflateInit(&infstream);
		inflate(&infstream, Z_NO_FLUSH);
		inflateEnd(&infstream);*/
		
		//LZ4_decompress_safe((char*)d.data(), (char*)depth.data, d.size(), depth.step*depth.rows);
		
		cv::imdecode(d, cv::IMREAD_UNCHANGED, &depth);
		depth.convertTo(depth, CV_32FC1, 1.0f/16.0f);
	});
	
	while (disp.active()) {
		Mat idepth;
		
		unique_lock<mutex> lk(m);
		if (depth.cols > 0) {
			// If no calibration data then get it from the remote machine
			if (Q.rows == 0) {
				// Must unlock before calling findOne to prevent net block!!
				lk.unlock();
				auto buf = net.findOne<vector<unsigned char>>((string)config["source"]+"/calibration");
				lk.lock();
				if (buf) {
					Q = Mat(cv::Size(4,4), CV_32F);
					memcpy(Q.data, (*buf).data(), (*buf).size());
					if (Q.step*Q.rows != (*buf).size()) LOG(ERROR) << "Corrupted calibration";
					disp.setCalibration(Q);
				}
			}
		}
		
		if (rgb.cols > 0) {
			//cv::imshow("RGB", rgb);
			disp.render(rgb,depth);
		}
		
		// TODO(nick) Use a swap buffer so this can be unlocked earlier
		lk.unlock();
		//if (cv::waitKey(40) == 27) break;
		disp.wait(40);
	}
}

int main(int argc, char **argv) {
	ftl::configure(argc, argv, "representation"); // TODO(nick) change to "reconstruction"
	run();
}

