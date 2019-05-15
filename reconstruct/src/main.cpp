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

//#include <pcl/point_cloud.h>
//#include <pcl/common/transforms.h>
//#include <pcl/filters/uniform_sampling.h>

using ftl::net::Universe;
using ftl::Display;
using ftl::config;
using std::string;
using std::vector;

using json = nlohmann::json;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::mutex;
using std::unique_lock;

using cv::Mat; 

class SourceStereo {
private:
	Mat rgb, disp;
	Mat q;
	mutex m;

	/*
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr _getPC(Mat rgb, Mat depth, Mat q) {
		auto pc = matToPointXYZ(depth, rgb);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*pc, *pc, indices);
		return pc;
	}*/

public:
	string uri;
	
	void recv(const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
		unique_lock<mutex> lk(m);
		cv::imdecode(jpg, cv::IMREAD_COLOR, &rgb);
		Mat(rgb.size(), CV_16UC1);
		cv::imdecode(d, cv::IMREAD_UNCHANGED, &disp);
		disp.convertTo(disp, CV_32FC1, 1.0f/16.0f);
	}
	
	/* thread unsafe, use lock() */
	
	void setQ(Mat &Q) { q = Q; }
	Mat getQ() { return q; }
	
	/*
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPC() {
		return _getPC(rgb, getDepth(), q);
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPC(cv::Size size) {
		Mat rgb_, depth_;
		
		cv::resize(rgb, rgb_, size);
		cv::resize(getDepth(), depth_, size);
		return _getPC(rgb_, depth_, q);
	}
	*/
	
	Mat getDepth() {
		cv::Mat_<cv::Vec3f> depth(disp.rows, disp.cols);
		reprojectImageTo3D(disp, depth, q, false);
		return depth;
	}
	
	Mat getRGB() {
		return rgb;
	}
	
	Mat getDisparity() {
		return disp;
	}
	
	unique_lock<mutex> lock() {return unique_lock<mutex>(m);} // use recursive mutex instead (and move locking to methods)?
};

static void run() {
	Universe net(config["net"]);
	
	// Make sure connections are complete
	sleep_for(milliseconds(500));
	
	if (!config["sources"].is_array()) {
		LOG(ERROR) << "No sources configured!";
		return;
	}
	
	// todo: create display objects at the same time, store in pair/tuple?
	//
	std::deque<SourceStereo> sources; // mutex in SourceStereo
	std::deque<Display> displays;	
	
	for (auto &src : config["sources"]) {
		SourceStereo &in = sources.emplace_back();
		Display &display = displays.emplace_back(Display(config["display"], src));
		
		// get calibration parameters from nodes
		while(true) {
			auto buf = net.findOne<vector<unsigned char>>((string) src +"/calibration");
			if (buf) {
				Mat Q = Mat(cv::Size(4,4), CV_32F);
				memcpy(Q.data, (*buf).data(), (*buf).size());
				if (Q.step*Q.rows != (*buf).size()) LOG(ERROR) << "Corrupted calibration";
				in.setQ(Q);
				LOG(INFO) << "Calibration loaded for " << (string) src;
				break;
			}
			else {
				LOG(INFO) << "Could not get calibration, retrying";
				sleep_for(milliseconds(500));
			}
		}
		net.subscribe(src, [&in](const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
			in.recv(jpg, d);
		});
	}

	int active = displays.size();
	while (active > 0) {
		active = 0;
		
		//std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);
		
		for (size_t i = 0; i < sources.size(); i++) {
			Display &display = displays[i];
			SourceStereo &source = sources[i];
			
			if (!display.active()) continue;
			active += 1;
			
			auto lk = source.lock();
			Mat rgb = source.getRGB();
			Mat disparity = source.getDisparity();
			Mat q = source.getQ();
			lk.unlock();
			
			display.render(rgb, disparity, q);
			display.wait(50);
		}
		
		/*
		pcl::transformPointCloud(*clouds[1], *clouds[1], T);
		pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
		uniform_sampling.setInputCloud(pc);
		uniform_sampling.setRadiusSearch(0.1f);
		uniform_sampling.filter(*pc);
		*/
	}
}

int main(int argc, char **argv) {
	ftl::configure(argc, argv, "representation"); // TODO(nick) change to "reconstruction"
	run();
}