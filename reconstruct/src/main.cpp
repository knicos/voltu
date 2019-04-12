/*
 * Copyright 2019 Nicolas Pope. All rights reserved.
 *
 * See LICENSE.
 */

#include <glog/logging.h>
#include <ftl/config.h>
#include <zlib.h>

#include <string>
#include <map>
#include <vector>
#include <fstream>
#include <thread>
#include <chrono>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <ftl/net/universe.hpp>
#include <nlohmann/json.hpp>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#ifdef WIN32
#pragma comment(lib, "Rpcrt4.lib")
#endif

using ftl::net::Universe;
using std::string;
using std::vector;
using std::map;
using cv::Mat;
using json = nlohmann::json;
using std::ifstream;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::mutex;
using std::unique_lock;

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
	config = config["representation"];
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
	Universe net(config["net"]);
	Mat rgb, depth;
	mutex m;
	
	// Make sure connections are complete
	sleep_for(milliseconds(500));

	net.subscribe(config["source"], [&rgb,&m,&depth](const vector<unsigned char> &jpg, const vector<unsigned char> &d) {
		unique_lock<mutex> lk(m);
		cv::imdecode(jpg, cv::IMREAD_COLOR, &rgb);
		//LOG(INFO) << "Received JPG : " << rgb.cols;
		
		depth = Mat(rgb.size(), CV_32FC1);
		
		z_stream infstream;
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
		inflateEnd(&infstream);
	});
	
	while (true) {
		Mat idepth;
		
		unique_lock<mutex> lk(m);
		if (rgb.cols > 0) {
			cv::imshow("RGB", rgb);
		}
		if (depth.cols > 0) {
			depth.convertTo(idepth, CV_8U, 255.0f / 256.0f);  // TODO(nick)
    		applyColorMap(idepth, idepth, cv::COLORMAP_JET);
			cv::imshow("Depth", idepth);
		}
		lk.unlock();
		if (cv::waitKey(40) == 27) break;
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

	run((argc > 0) ? argv[0] : "");
}

