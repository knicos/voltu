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
#include <mutex>
#include <condition_variable>

#include <opencv2/opencv.hpp>
#include <ftl/rgbd.hpp>
#include <ftl/middlebury.hpp>
#include <ftl/display.hpp>
#include <ftl/rgbd_streamer.hpp>
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
using ftl::rgbd::Streamer;
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


static void run(ftl::Configurable *root) {
	Universe *net = ftl::create<Universe>(root, "net");
	LOG(INFO) << "Net started.";

	auto paths = root->get<vector<string>>("paths");
	string file = "";
	if (paths && (*paths).size() > 0) file = (*paths)[0];

	StereoVideoSource *source = nullptr;
	source = ftl::create<StereoVideoSource>(root, "source", file);
	
	Display *display = ftl::create<Display>(root, "display", "local");
	
	Streamer *stream = ftl::create<Streamer>(root, "stream", net);
	stream->add(source);
	stream->run();

	while (display->active()) {
		cv::Mat rgb, depth;
		source->getRGBD(rgb, depth);
		if (!rgb.empty()) display->render(rgb, depth, source->getParameters());
		display->wait(10);
	}

	stream->stop();

	LOG(INFO) << "Finished.";
	delete stream;
	delete display;
	delete source;
	delete net;
}

int main(int argc, char **argv) {
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
}

