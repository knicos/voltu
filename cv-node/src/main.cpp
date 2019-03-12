#include <opencv2/opencv.hpp>
#include <ftl/local.hpp>
#include <ftl/synched.hpp>
#include <ftl/calibrate.hpp>

#include <glog/logging.h>

#include <string>
#include <vector>

using namespace ftl;
using std::string;
using std::vector;
using cv::Mat;

static vector<string> OPTION_peers;
static vector<string> OPTION_channels;
static string OPTION_calibration_config = FTL_CONFIG_ROOT "/calibration.xml";
static string OPTION_config;
static bool OPTION_display = false;
static bool OPTION_calibrate = false;
static bool OPTION_flip = false;
static bool OPTION_nostereo = false;

void handle_options(char ***argv, int *argc) {
	while (*argc > 0) {
		string cmd((*argv)[0]);
		if (cmd[0] != '-') break;
		
		if (cmd.find("--calibrate") == 0) {
			OPTION_calibrate = true;
		} else if (cmd.find("--peer=") == 0) {
			cmd = cmd.substr(cmd.find("=")+1);
			OPTION_peers.push_back(cmd);
		} else if (cmd.find("--channel=") == 0) {
			cmd = cmd.substr(cmd.find("=")+1);
			OPTION_channels.push_back(cmd);
		} else if (cmd.find("--calibration=") == 0) {
			cmd = cmd.substr(cmd.find("=")+1);
			OPTION_calibration_config = cmd;
		} else if (cmd.find("--config=") == 0) {
			cmd = cmd.substr(cmd.find("=")+1);
			OPTION_config = cmd;
		} else if (cmd.find("--display") == 0) {
			OPTION_display = true;
		} else if (cmd.find("--flip") == 0) {
			OPTION_flip = true;
		} else if (cmd.find("--no-stereo") == 0) {
			OPTION_nostereo = true;
		}
		
		(*argc)--;
		(*argv)++;
	}
}

int main(int argc, char **argv) {
	argc--;
	argv++;
	
	// Process Arguments
	handle_options(&argv, &argc);
	
	// TODO Initiate the network
	
	LocalSource *lsrc;
	
	if (argc) {
		// Load video file
		lsrc = new LocalSource(argv[0], OPTION_flip, OPTION_nostereo);
	} else {
		// Use cameras
		lsrc = new LocalSource(OPTION_flip, OPTION_nostereo);
	}
	
	auto sync = new SyncSource(); // TODO Pass protocol object
	// Add any remote channels
	for (auto c : OPTION_channels) {
		sync->addChannel(c);
	}
	
	Calibrate calibrate(lsrc, OPTION_calibration_config);
	
	if (OPTION_calibrate) {
		calibrate.recalibrate();
	}
	
	if (!calibrate.isCalibrated()) {
		LOG(WARNING) << "Cameras are not calibrated!";
	}
	
	while (true) {
		Mat l, r;
		
		calibrate.undistort(l,r);
		//lsrc->get(l,r);
		
		// Feed into sync buffer and network forward
		sync->feed(LEFT, l,lsrc->getTimestamp());
		sync->feed(RIGHT, r,lsrc->getTimestamp());
		
		// Read back from buffer
		sync->get(LEFT,l);
		sync->get(RIGHT,r);
		//double latency = sync->latency();
		
		// TODO Pose and disparity etc here...
		
		// TODO Send RGB+D data somewhere
		
		cv::imshow("Left",l);
		if (lsrc->isStereo()) cv::imshow("Right",r);
		
		if(cv::waitKey(100) == 27){
            //exit if ESC is pressed
            break;
        }
	}
}

