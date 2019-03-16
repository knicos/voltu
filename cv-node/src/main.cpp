#include <opencv2/opencv.hpp>
#include <ftl/local.hpp>
#include <ftl/synched.hpp>
#include <ftl/calibrate.hpp>
#include <ftl/disparity.hpp>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <glog/logging.h>

#include <string>
#include <vector>

using namespace ftl;
using std::string;
using std::vector;
using cv::Mat;

using namespace cv;
//using namespace cv::ximgproc;

static vector<string> OPTION_peers;
static vector<string> OPTION_channels;
static string OPTION_calibration_config = FTL_CONFIG_ROOT "/calibration.xml";
static string OPTION_config;
static string OPTION_algorithm = "rtcensus";
static bool OPTION_display = false;
static bool OPTION_calibrate = false;
static bool OPTION_flip = false;
static bool OPTION_nostereo = false;
static bool OPTION_noextrinsics = false;

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
		} else if (cmd.find("--algorithm=") == 0) {
			cmd = cmd.substr(cmd.find("=")+1);
			OPTION_algorithm = cmd;
		} else if (cmd.find("--display") == 0) {
			OPTION_display = true;
		} else if (cmd.find("--flip") == 0) {
			OPTION_flip = true;
		} else if (cmd.find("--no-stereo") == 0) {
			OPTION_nostereo = true;
		} else if (cmd.find("--no-extrinsics") == 0) {
			OPTION_noextrinsics = true;
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
	
	// Perform or load calibration intrinsics + extrinsics
	Calibrate calibrate(lsrc, OPTION_calibration_config);
	if (OPTION_calibrate) calibrate.recalibrate();
	if (!calibrate.isCalibrated()) LOG(WARNING) << "Cameras are not calibrated!";

	/*Ptr<StereoBM> left_matcher = StereoBM::create(max_disp,wsize);
	//left_matcher->setPreFilterCap(63);
            wls_filter = createDisparityWLSFilter(left_matcher);
            Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);*/
    
    // Choose and configure disparity algorithm
    auto disparity = Disparity::create(OPTION_algorithm);
    disparity->setMaxDisparity(208);
    
    //double fact = 4.051863857;
	
	Mat l, r, filtered_disp;
	
	while (true) {
		// Read calibrated images.
		calibrate.undistort(l,r);
		
		// Feed into sync buffer and network forward
		sync->feed(LEFT, l,lsrc->getTimestamp());
		sync->feed(RIGHT, r,lsrc->getTimestamp());
		
		// Read back from buffer
		sync->get(LEFT,l);
		sync->get(RIGHT,r);
		
		// Downscale
		//cv::resize(l, l, cv::Size(l.cols * 0.75,l.rows * 0.75), 0, 0, INTER_LINEAR);
		//cv::resize(r, r, cv::Size(r.cols * 0.75,r.rows * 0.75), 0, 0, INTER_LINEAR);
		
		// Black and white
        cvtColor(l,  l,  COLOR_BGR2GRAY);
        cvtColor(r, r, COLOR_BGR2GRAY);
        
        disparity->compute(l,r,filtered_disp);
		LOG(INFO) << "Disparity complete ";
		
		// TODO Send RGB+D data somewhere
		
		//left_disp = (fact * (double)l.cols * 0.1) / filtered_disp;

		normalize(filtered_disp, filtered_disp, 0, 255, NORM_MINMAX, CV_8U);
		
		cv::imshow("Disparity",filtered_disp);
		
		if(cv::waitKey(10) == 27){
            //exit if ESC is pressed
            break;
        }
	}
}

