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
#include <ftl/rgbd.hpp>
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

using ftl::rgbd::RGBDSource;
using ftl::rgbd::CameraParameters;
using ftl::rgbd::StereoVideoSource;
using ftl::Display;
using ftl::Streamer;
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

namespace ftl {
void disparityToDepth(const cv::Mat &disparity, cv::Mat &depth, const cv::Mat &q) {
	cv::Matx44d _Q;
    q.convertTo(_Q, CV_64F);

	if (depth.empty()) depth = cv::Mat(disparity.size(), CV_32F);

	for( int y = 0; y < disparity.rows; y++ ) {
		const float *sptr = disparity.ptr<float>(y);
		float *dptr = depth.ptr<float>(y);

		for( int x = 0; x < disparity.cols; x++ ) {
			double d = sptr[x];
			cv::Vec4d homg_pt = _Q*cv::Vec4d(x, y, d, 1.0);
			//dptr[x] = Vec3d(homg_pt.val);
			//dptr[x] /= homg_pt[3];
			dptr[x] = (homg_pt[2] / homg_pt[3]) / 1000.0f; // Depth in meters

			if( fabs(d) <= FLT_EPSILON )
				dptr[x] = 1000.0f;
		}
	}
}
};


static void run(const string &file) {
	ctpl::thread_pool pool(2);
	Universe net(config["net"]);
	LOG(INFO) << "Net started.";

	StereoVideoSource *source = nullptr;
	source = new StereoVideoSource(config, file);
	
	// Allow remote users to access camera calibration matrix
	net.bind(string("ftl://utu.fi/")+(string)config["stream"]["name"]+string("/rgb-d/calibration"), [source]() -> vector<unsigned char> {
		vector<unsigned char> buf;
		buf.resize(sizeof(CameraParameters));
		LOG(INFO) << "Calib buf size = " << buf.size();
		memcpy(buf.data(), &source->getParameters(), buf.size());
		return buf;
	});

	Mat rgb, depth;
	bool grabbed = false;
	mutex datam;
	condition_variable datacv;

	// Wait for grab message to sync camera capture
	net.bind("grab", [&source,&datam,&datacv,&grabbed]() -> void {
		unique_lock<mutex> datalk(datam);
		if (grabbed) return;
		source->grab();
		grabbed = true;
		datacv.notify_one();
	});

	Mat prgb, pdepth;
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
			source->grab();
		}

		// Pipeline for disparity
		pool.push([&](int id) {
			// Wait for image grab
			unique_lock<mutex> datalk(datam);
			datacv.wait(datalk, [&grabbed](){ return grabbed; });
			grabbed = false;

			auto start = std::chrono::high_resolution_clock::now();
		    source->getRGBD(rgb, depth);
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
			if (prgb.rows != 0) stream.send(prgb, pdepth);
			unique_lock<mutex> lk(m);
			jobs++;
			lk.unlock();
			cv.notify_one();

			std::chrono::duration<double> elapsed =
				std::chrono::high_resolution_clock::now() - start;
			LOG(INFO) << "Stream in " << elapsed.count() << "s";
		});

		// Send RGB+Depth images for local rendering
		if (prgb.rows > 0) display.render(prgb, pdepth, source->getParameters());
		if (config["display"]["right"]) {
			Mat rgbr = source->getRight().clone();
			cv::namedWindow("Right: ", cv::WINDOW_KEEPRATIO);
			cv::line(rgbr, cv::Point(0, rgbr.rows/2), cv::Point(rgbr.cols-1, rgbr.rows/2), cv::Scalar(0,0,255), 1);
            cv::line(rgbr, cv::Point(rgbr.cols/2, 0), cv::Point(rgbr.cols/2, rgbr.rows-1), cv::Scalar(0,0,255), 1);
			cv::imshow("Right: ", rgbr);
		}
		
		display.wait(1);

		// Wait for both pipelines to complete
		unique_lock<mutex> lk(m);
		cv.wait(lk, [&jobs]{return jobs == 2;});

		// Store previous frame for next stage compression
		rgb.copyTo(prgb);
		depth.copyTo(pdepth);

		//net.publish(uri, rgb_buf, d_buf);
	}

	LOG(INFO) << "Finished.";
}

int main(int argc, char **argv) {
	std::cout << "FTL Vision Node " << FTL_VERSION_LONG << std::endl;
	auto paths = ftl::configure(argc, argv, "vision");
	
	config["paths"] = paths;

	// Choose normal or middlebury modes
	if (config["middlebury"]["dataset"] == "") {
		std::cout << "Loading..." << std::endl;
		run((paths.size() > 0) ? paths[0] : "");
	} else {
		ftl::middlebury::test(config);
	}
}

