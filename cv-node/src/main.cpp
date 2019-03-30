#include <opencv2/opencv.hpp>
#include <ftl/local.hpp>
#include <ftl/synched.hpp>
#include <ftl/calibrate.hpp>
#include <ftl/disparity.hpp>
#include <ftl/middlebury.hpp>
#include <ftl/display.hpp>
#include <ftl/config.h>
#include <nlohmann/json.hpp>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <glog/logging.h>

#include <string>
#include <map>
#include <vector>

using namespace ftl;
using std::string;
using std::vector;
using std::map;
using cv::Mat;
using json = nlohmann::json;
using std::ifstream;
using namespace cv;

// Store loaded configuration
static json config;

/**
 * Find and load a JSON configuration file
 */
static bool findConfiguration(const string &file) {
	// TODO Check other locations
	ifstream i((file != "") ? file : FTL_CONFIG_ROOT "/config.json");
	if (!i.is_open()) return false;
	i >> config;
	return true;
}

/**
 * Generate a map from command line option to value
 */
map<string,string> read_options(char ***argv, int *argc) {
	map<string,string> opts;
	
	while (*argc > 0) {
		string cmd((*argv)[0]);
		if (cmd[0] != '-') break;
		
		size_t p;
		if ((p = cmd.find("=")) == string::npos) {
			opts[cmd.substr(2)] = "true";
		} else {
			opts[cmd.substr(2,p-2)] = cmd.substr(p+1);
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
static void process_options(const map<string,string> &opts) {
	for (auto opt : opts) {
		if (opt.first == "config") continue;
		
		try {
			auto ptr = json::json_pointer("/"+opt.first);
			// TODO Allow strings without quotes
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
	// TODO Initiate the network
	
	LocalSource *lsrc;
	if (file != "") {
		// Load video file
		lsrc = new LocalSource(file, config["source"]);
	} else {
		// Use cameras
		lsrc = new LocalSource(config["source"]);
	}
	
	auto sync = new SyncSource(); // TODO Pass protocol object
	// Add any remote channels
	/*for (auto c : OPTION_channels) {
		sync->addChannel(c);
	}*/
	
	// Perform or load calibration intrinsics + extrinsics
	Calibrate calibrate(lsrc, config["calibration"]);
	if (config["calibrate"]) calibrate.recalibrate();
	if (!calibrate.isCalibrated()) LOG(WARNING) << "Cameras are not calibrated!";
    
    // Choose and configure disparity algorithm
    auto disparity = Disparity::create(config["disparity"]);
	
	Mat l, r, disparity32F, depth32F, lbw, rbw;
	
	Display display(calibrate, config["display"]);
	
	float base_line = (float)config["camera"]["base_line"];
	float focal = (float)(config["camera"]["focal_length"]) / (float)(config["camera"]["sensor_width"]);
	Mat rot_vec = Mat::zeros(1,3,CV_32F);
	
	while (display.active()) {
		// Read calibrated images.
		calibrate.rectified(l,r);
		
		// Feed into sync buffer and network forward
		sync->feed(LEFT, l,lsrc->getTimestamp());
		sync->feed(RIGHT, r,lsrc->getTimestamp());
		
		// Read back from buffer
		sync->get(LEFT,l);
		sync->get(RIGHT,r);
        
        disparity->compute(l,r,disparity32F);
		
		// Clip the left edge
		//Rect rect((int)config["disparity"]["maximum"],7,disparity32F.cols-(int)config["disparity"]["maximum"],disparity32F.rows-14);

		// Send RGB+Depth images for local rendering
		display.render(l, disparity32F);

		//streamer.send(l, disparity32F);
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

